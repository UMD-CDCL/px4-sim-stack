#!/usr/bin/env python3
"""One machine readable picture of the stack, as JSON.

`./px4sim state` prints one object. `./px4sim state --watch` prints one object
for each period, on its own line, until it is stopped. scripts/tui.py reads
that stream, and so can jq.

The front door gives this the fleet on standard input, one fact for each line.
Nothing here works a fleet number out a second time. Every reading comes from
the thing itself, never from a log message:

  services   the container engine's own state for each container
  streams    the video router's API, and the bytes each path really carried
  vehicles   MAVLink from the vehicle, read on the TCP port the routers publish
  bridges    a connection to each Foxglove port, opened and closed
  gpu        nvidia-smi, which counts the encoder sessions the cameras hold

A warning from the front door can come before the first object, because the
front door writes what it finds in .env before it runs anything. A reader takes
the lines that parse as JSON, and shows the rest as they are.

Read it with:  ./px4sim state | python3 -m json.tool
"""

from __future__ import annotations

import argparse
import errno
import json
import os
import selectors
import shlex
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mavlink  # noqa: E402

WATCH_PERIOD_S = 2.0
MINIMUM_PERIOD_S = 0.2
MINIMUM_PUMP_S = 0.25
FIRST_TICK_S = 0.5
LISTEN_S = 1.5
LINK_RETRY_S = 3.0
LINK_SILENT_S = 3.0
CONNECT_TIMEOUT_S = 2.0
IDLE_SLEEP_S = 0.2
PORT_PROBE_S = 0.4
BRIDGE_PERIOD_TICKS = 5
STREAMS_TIMEOUT_S = 1.5
STREAMS_RETRY_TICKS = 5
DOCKER_TIMEOUT_S = 15.0
GPU_TIMEOUT_S = 5.0
GPU_READINGS = "utilization.gpu,memory.used,memory.total,encoder.stats.sessionCount"
LOOPBACK = "127.0.0.1"
AUTOPILOT = 1  # MAV_COMP_ID_AUTOPILOT1, the component that holds the mode


def number_or_text(value: str):
    for kind in (int, float):
        try:
            return kind(value)
        except ValueError:
            pass
    return value


def read_context(text: str) -> dict:
    """The fleet as the front door states it. See the `state` arm of ./px4sim."""
    context: dict = {"fleet": []}
    for line in text.splitlines():
        if line.startswith("vehicle "):
            context["fleet"].append(
                {key: number_or_text(value) for key, _, value
                 in (pair.partition("=") for pair in line.split()[1:])})
        elif "=" in line:
            key, _, value = line.partition("=")
            context[key.strip()] = number_or_text(value.strip())
    return context


def as_list(value) -> list[str]:
    return [item for item in str(value).split(",") if item]


def parsed_records(text: str) -> list[dict]:
    """Compose prints an array or one object for each line, by version."""
    text = text.strip()
    if not text:
        return []
    try:
        loaded = json.loads(text)
        return loaded if isinstance(loaded, list) else [loaded]
    except json.JSONDecodeError:
        pass
    records = []
    for line in text.splitlines():
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return records


def containers(project_dir: Path, compose: str) -> tuple[dict[str, dict], str]:
    """Every container of this project, as the container engine reports it."""
    try:
        done = subprocess.run(
            [*shlex.split(compose), "ps", "--all", "--format", "json"],
            cwd=project_dir, capture_output=True, text=True, timeout=DOCKER_TIMEOUT_S)
    except (OSError, subprocess.SubprocessError) as failure:
        return {}, f"docker compose ps: {failure}"
    if done.returncode != 0:
        complaint = done.stderr.strip().splitlines()
        return {}, complaint[-1] if complaint else "docker compose ps failed"

    found = {}
    for record in parsed_records(done.stdout):
        service = record.get("Service", "")
        found[service] = {
            "service": service,
            "name": record.get("Name", ""),
            "state": record.get("State", ""),
            "status": record.get("Status", ""),
            "health": record.get("Health", ""),
            "exit_code": record.get("ExitCode", 0),
        }
    return found, ""


def service_rows(context: dict, found: dict[str, dict], error: str) -> list[dict]:
    """The services this stack has, in the order compose.yaml declares them.

    A service that the profiles select and the engine does not hold gets a row
    of its own. A stack that never started one then reads as absent.
    """
    declared = as_list(context.get("declared", ""))
    expected = set(as_list(context.get("expected", "")))
    names = [name for name in declared if name in found or name in expected]
    names += [name for name in sorted(found) if name not in declared]
    rows = []
    for name in names:
        row = found.get(name, {"service": name, "name": "", "health": "",
                               "exit_code": 0,
                               "state": "unknown" if error else "absent",
                               "status": "" if error else "not created"})
        row["wanted"] = name in expected
        rows.append(row)
    return rows


def graphics_card() -> dict:
    """What the card is doing. The cameras, the encoders and the detector share it.

    The first card answers, which is the one the containers are given. A host
    with no nvidia-smi says nothing, and the console then draws nothing.
    """
    try:
        done = subprocess.run(["nvidia-smi", f"--query-gpu={GPU_READINGS}",
                               "--format=csv,noheader,nounits"],
                              capture_output=True, text=True, timeout=GPU_TIMEOUT_S)
    except (OSError, subprocess.SubprocessError):
        return {}
    lines = done.stdout.strip().splitlines()
    if done.returncode != 0 or not lines:
        return {}
    readings = [word.strip() for word in lines[0].split(",")]
    try:
        return {"busy_percent": float(readings[0]),
                "memory_used_mb": float(readings[1]),
                "memory_total_mb": float(readings[2]),
                "encoder_sessions": int(readings[3])}
    except (IndexError, ValueError):
        return {}


class Streams:
    """The video paths, and the bytes each one carried between two reads."""

    def __init__(self, api: str, owners: dict[str, int]):
        self.api = api
        self.owners = owners
        self.carried: dict[str, tuple[float, int]] = {}
        self.quiet_until_tick = 0
        self.error = ""
        self.paths: list[dict] = []

    def read(self, tick: int) -> list[dict]:
        if tick < self.quiet_until_tick:
            return self.paths
        now = time.monotonic()
        try:
            with urllib.request.urlopen(f"{self.api}/v3/paths/list",
                                        timeout=STREAMS_TIMEOUT_S) as answer:
                items = json.load(answer)["items"]
            paths = [self.one_path(path, now) for path in items]
        except urllib.error.HTTPError as failure:
            return self.gave_up(f"the video router API answered {failure.code}", tick)
        except (urllib.error.URLError, OSError) as failure:
            return self.gave_up(f"cannot reach {self.api}: {failure}", tick)
        except (ValueError, TypeError, KeyError, AttributeError) as failure:
            return self.gave_up(f"{self.api} answered something this cannot"
                                f" read: {failure}", tick)

        self.error = ""
        self.paths = paths
        self.carried = {name: carried for name, carried in self.carried.items()
                        if name in {path["name"] for path in paths}}
        return self.paths

    def gave_up(self, why: str, tick: int) -> list[dict]:
        self.error = why
        self.quiet_until_tick = tick + STREAMS_RETRY_TICKS
        self.carried.clear()
        self.paths = []
        return self.paths

    def one_path(self, path: dict, now: float) -> dict:
        name = path.get("name", "?")
        received = int(path.get("bytesReceived", 0))
        before = self.carried.get(name)
        self.carried[name] = (now, received)
        kbits = None
        if before is not None:
            elapsed = now - before[0]
            if elapsed > 0 and received >= before[1]:
                kbits = (received - before[1]) * 8 / 1000.0 / elapsed
        return {
            "name": name,
            "vehicle": self.owners.get(name),
            "ready": bool(path.get("ready")),
            "readers": len(path.get("readers") or []),
            "kbits": kbits,
            "source": (path.get("source") or {}).get("type", ""),
        }


class Link:
    """A passive MAVLink client on one vehicle's published TCP port.

    It sends nothing. The vehicle number is the MAVLink system id, so a frame
    from another system counts as a system that is present and nothing more.
    """

    def __init__(self, number: int, port: int, system: int):
        self.number = number
        self.port = port
        self.system = system
        self.socket: socket.socket | None = None
        self.connected = False
        self.registered: int | None = None
        self.lost = ""
        self.error = ""
        self.retry_at = 0.0
        self.attempted_at = 0.0
        self.buffer = b""
        self.fields: dict = {}
        self.systems: set[int] = set()
        self.frames = 0
        self.reported_frames = 0
        self.reported_at = time.monotonic()
        self.heard_at = 0.0

    def wanted_events(self) -> int:
        return selectors.EVENT_READ if self.connected else selectors.EVENT_WRITE

    def open(self, now: float) -> None:
        if self.socket is not None or now < self.retry_at:
            return
        opening = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        opening.setblocking(False)
        result = opening.connect_ex((LOOPBACK, self.port))
        if result not in (0, errno.EINPROGRESS, errno.EALREADY, errno.EWOULDBLOCK):
            opening.close()
            self.error = errno.errorcode.get(result, str(result))
            self.retry_at = now + LINK_RETRY_S
            return
        self.socket = opening
        self.connected = result == 0
        if self.connected:
            self.error = ""
        self.attempted_at = now

    def close(self) -> None:
        """A closed link knows nothing. What it read last is not what is true now."""
        if self.socket is not None:
            self.socket.close()
        self.socket = None
        self.connected = False
        self.registered = None
        self.buffer = b""
        self.fields = {}
        self.heard_at = 0.0

    def timed_out(self, now: float) -> bool:
        return (self.socket is not None and not self.connected
                and now - self.attempted_at > CONNECT_TIMEOUT_S)

    def ready(self, now: float) -> None:
        """The selector says this socket can be used. Finish it, or read it."""
        if self.socket is None:
            return
        if not self.connected:
            result = self.socket.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
            if result != 0:
                self.lost = errno.errorcode.get(result, str(result))
                return
            self.connected = True
            self.error = ""
            return
        try:
            arrived = self.socket.recv(65536)
        except (BlockingIOError, InterruptedError):
            return
        except OSError as failure:
            self.lost = str(failure)
            return
        if not arrived:
            self.lost = "the router closed the connection"
            return
        self.take(arrived, now)

    def take(self, arrived: bytes, now: float) -> None:
        found, self.buffer = mavlink.frames(self.buffer + arrived)
        for frame in found:
            self.systems.add(frame.system)
            if frame.system != self.system:
                continue
            self.frames += 1
            self.heard_at = now
            if frame.message == mavlink.HEARTBEAT and frame.component != AUTOPILOT:
                continue
            self.fields.update(mavlink.fields(frame))

    def report(self, now: float) -> dict:
        elapsed = max(now - self.reported_at, 1e-6)
        rate = (self.frames - self.reported_frames) / elapsed
        self.reported_frames, self.reported_at = self.frames, now
        silent_for = now - self.heard_at if self.heard_at else None
        if not self.connected:
            link = "down"
        elif silent_for is None or silent_for > LINK_SILENT_S:
            link = "silent"
        else:
            link = "up"
        told = {"number": self.number, "port": self.port, "link": link,
                "silent_for": silent_for, "messages_per_s": rate,
                "systems": sorted(self.systems), "error": self.error}
        told.update(self.fields)
        self.systems = set()
        return told


class Links:
    """One reader for each vehicle, kept open between reports."""

    def __init__(self, fleet: list[dict]):
        self.links = [Link(vehicle["n"], vehicle["tcp"],
                           vehicle.get("sysid", vehicle["n"]))
                      for vehicle in fleet
                      if isinstance(vehicle.get("n"), int)
                      and isinstance(vehicle.get("tcp"), int)]
        self.watcher = selectors.DefaultSelector()

    def pump(self, deadline: float) -> None:
        while True:
            now = time.monotonic()
            if now >= deadline:
                return
            self.refresh(now)
            if not self.watcher.get_map():
                time.sleep(min(deadline - now, IDLE_SLEEP_S))
                continue
            for key, _events in self.watcher.select(timeout=deadline - now):
                key.data.ready(time.monotonic())

    def refresh(self, now: float) -> None:
        for link in self.links:
            if link.timed_out(now):
                link.lost = "no answer on the port"
            if link.lost:
                self.forget(link)
                link.close()
                link.error, link.lost = link.lost, ""
                link.retry_at = now + LINK_RETRY_S
            link.open(now)
            self.watch(link)

    def forget(self, link: Link) -> None:
        if link.registered is not None:
            self.watcher.unregister(link.socket)
            link.registered = None

    def watch(self, link: Link) -> None:
        if link.socket is None:
            return
        wanted = link.wanted_events()
        if link.registered == wanted:
            return
        if link.registered is None:
            self.watcher.register(link.socket, wanted, link)
        else:
            self.watcher.modify(link.socket, wanted, link)
        link.registered = wanted

    def report(self, now: float) -> list[dict]:
        return [link.report(now) for link in self.links]

    def close(self) -> None:
        for link in self.links:
            self.forget(link)
            link.close()
        self.watcher.close()


def listening(ports: list[int], timeout: float = PORT_PROBE_S) -> dict[int, bool]:
    """Which ports accept a connection right now. Every port is tried at once."""
    answers = {port: False for port in ports}
    trying = {}
    for port in ports:
        opening = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        opening.setblocking(False)
        result = opening.connect_ex((LOOPBACK, port))
        if result in (0, errno.EINPROGRESS, errno.EALREADY, errno.EWOULDBLOCK):
            trying[port] = opening
        else:
            opening.close()
    deadline = time.monotonic() + timeout
    with selectors.DefaultSelector() as watcher:
        for port, opening in trying.items():
            watcher.register(opening, selectors.EVENT_WRITE, port)
        while watcher.get_map():
            left = deadline - time.monotonic()
            if left <= 0:
                break
            for key, _events in watcher.select(timeout=left):
                port = key.data
                answers[port] = trying[port].getsockopt(
                    socket.SOL_SOCKET, socket.SO_ERROR) == 0
                watcher.unregister(key.fileobj)
    for opening in trying.values():
        opening.close()
    return answers


class Bridges:
    """The Foxglove ports an operator connects a panel to.

    This opens a connection to each port and closes it again, which is how it
    learns whether a bridge holds the port. It waits a few periods between
    tries, because the answer changes rarely and each try reaches a server that
    writes a line about it.
    """

    def __init__(self, context: dict):
        self.named = [{"name": "ground",
                       "port": int(context.get("ground_foxglove", 8765))}]
        for vehicle in context["fleet"]:
            if vehicle.get("companion"):
                self.named.append({"name": f"uas{vehicle['n']}",
                                   "port": int(vehicle["foxglove"])})
        self.open_ports: dict[int, bool] = {}

    def read(self, tick: int) -> list[dict]:
        if tick % BRIDGE_PERIOD_TICKS == 0 or not self.open_ports:
            self.open_ports = listening([bridge["port"] for bridge in self.named])
        return [dict(bridge, listening=self.open_ports.get(bridge["port"], False))
                for bridge in self.named]


def stream_owners(fleet: list[dict]) -> dict[str, int]:
    return {name: int(vehicle["n"]) for vehicle in fleet
            for name in as_list(vehicle.get("streams", ""))}


def snapshot(context: dict, project_dir: Path, streams: Streams, links: Links,
             bridges: Bridges, tick: int) -> dict:
    # The vehicles answer first. Reading the containers and the card takes
    # seconds on a busy host, and nothing reads a socket while it happens, so a
    # vehicle asked afterwards looks silent while its telemetry is arriving.
    vehicles = links.report(time.monotonic())
    found, services_error = containers(project_dir, str(context.get("compose",
                                                                   "docker compose")))
    return {
        "at": time.time(),
        "tick": tick,
        "config": context,
        "services": service_rows(context, found, services_error),
        "services_error": services_error,
        "streams": streams.read(tick),
        "streams_error": streams.error,
        "vehicles": vehicles,
        "bridges": bridges.read(tick),
        "gpu": graphics_card(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--watch", nargs="?", type=float, const=WATCH_PERIOD_S,
                        default=None, metavar="SECONDS",
                        help="keep reporting, one JSON object for each period")
    parser.add_argument("--listen", type=float, default=LISTEN_S,
                        help="seconds to hear the vehicles before a single report")
    arguments = parser.parse_args()

    context = read_context("" if sys.stdin.isatty() else sys.stdin.read())
    project_dir = Path(__file__).resolve().parents[1]
    streams = Streams(str(context.get("mediamtx", "http://localhost:9997")),
                      stream_owners(context["fleet"]))
    links = Links(context["fleet"])
    bridges = Bridges(context)

    def tell(tick: int) -> None:
        json.dump(snapshot(context, project_dir, streams, links, bridges, tick),
                  sys.stdout)
        sys.stdout.write("\n")
        sys.stdout.flush()

    try:
        if arguments.watch is None:
            links.pump(time.monotonic() + arguments.listen)
            tell(0)
            return 0
        period = max(MINIMUM_PERIOD_S, arguments.watch)
        deadline = time.monotonic() + FIRST_TICK_S
        tick = 0
        while True:
            links.pump(deadline)
            tell(tick)
            tick += 1
            deadline = max(deadline + period, time.monotonic() + MINIMUM_PUMP_S)
    except KeyboardInterrupt:
        return 0
    except BrokenPipeError:
        # The reader went away. Give this process a stdout that goes nowhere,
        # or the interpreter reports the same broken pipe again as it exits.
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
        return 0
    finally:
        links.close()


if __name__ == "__main__":
    sys.exit(main())
