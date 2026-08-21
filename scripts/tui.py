#!/usr/bin/env python3
"""A terminal console for px4sim: what the stack is doing, and one key to act.

`./px4sim ui` starts this. It draws the picture that `./px4sim state --watch`
reports, and every action it offers is a px4sim command that it runs for you.
It holds no second way to start a container, fly a vehicle or read a stream, so
what happens here is what happens at the prompt.

  tab      move between the services, the vehicles and the streams
  enter    what can be done to the selected thing
  .        what can be done to the stack
  pgup     read back through the output of the last command
  :        type any px4sim command
  ?        every action, and the command each one runs
  esc      stop the command that is running
  q        leave

It draws best in a terminal of 100 columns or more.
"""

from __future__ import annotations

import curses
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import NamedTuple

REPORT_PERIOD_S = 2.0
FEED_RETRY_S = 5.0
INPUT_PERIOD_MS = 200
OUTPUT_LINES = 500
OUTPUT_ROWS_SHARE = 0.34
STALE_REPORT_S = 8.0
NAME_COLUMN = 15
ESCAPE_CODES = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]|\x1b[=>]|[\x00-\x08\x0b\x0c\x0e-\x1f]")

FRONT_DOOR = Path(__file__).resolve().parents[1] / "px4sim"

STACK, SERVICE, VEHICLE, STREAM = "stack", "service", "vehicle", "stream"
PANES = (SERVICE, VEHICLE, STREAM)


class Ask(NamedTuple):
    prompt: str
    default: str = ""
    choices: tuple[str, ...] = ()
    choices_from: str = ""
    split: bool = False


class Action(NamedTuple):
    scope: str
    key: str
    label: str
    command: tuple[str, ...]
    ask: Ask | None = None
    confirm: str = ""
    foreground: bool = False
    refresh: bool = False


# Every action this console offers, and the px4sim command it runs. This table
# fills the menus, the hot keys, the footer and the help, so an action is
# written down once.
ACTIONS = (
    Action(STACK, "s", "start the stack", ("start",)),
    Action(STACK, "x", "stop everything", ("stop",), confirm="Stop every container?"),
    Action(STACK, "", "recreate every service", ("restart",),
           confirm="Recreate every container?"),
    Action(STACK, "=", "fly one more vehicle", ("fleet", "add", "{value}"),
           ask=Ask("airframe", choices_from="models"),
           confirm="Add a vehicle? The world reloads and every vehicle respawns.",
           refresh=True),
    Action(STACK, "B", "build the images", ("build",),
           confirm="Build every image? This takes about twenty minutes."),
    Action(STACK, "P", "put the fleet back at its start", ("place",),
           confirm="Reload the world and respawn every vehicle?"),
    Action(STACK, "A", "place the targets again", ("scenario",)),
    Action(STACK, "N", "switch the world", ("scene", "{value}"),
           ask=Ask("scene name", "{scene}"), refresh=True),
    Action(STACK, "T", "switch the targets", ("scenario", "{value}"),
           ask=Ask("scenario", "{scenario}", choices_from="scenarios"),
           refresh=True),
    Action(STACK, "F", "stand the survey marker off its survey",
           ("fiducial", "{value}"), ask=Ask("east north, in metres", "0 0", split=True)),
    Action(STACK, "V", "run the verification", ("verify",)),
    Action(STACK, "C", "check the front door and the docs", ("check",)),
    Action(STACK, "D", "check the host", ("doctor",)),
    Action(STACK, "L", "where the Foxglove layout lives", ("layout",)),
    Action(STACK, "K", "the PX4 console. Detach with Ctrl-P Ctrl-Q", ("console",),
           foreground=True),
    Action(STACK, "", "what the ground station's ROS graph carries",
           ("probe", "ground")),
    Action(STACK, "", "what Foxglove is offered on the ground",
           ("foxglove", "ground")),
    Action(STACK, "", "the addresses to use", ("endpoints",)),
    Action(STACK, "", "every vehicle, in full", ("fleet",)),
    Action(STACK, "", "the video paths", ("streams",)),
    Action(STACK, "", "the coordinates this scene flies at", ("origin",)),
    Action(STACK, "", "remove the containers and the volumes", ("clean",),
           confirm="Remove every container, network and volume?"),

    Action(SERVICE, "o", "follow the log", ("logs", "{service}")),
    Action(SERVICE, "h", "read the last fifteen minutes",
           ("logs", "--since", "15m", "{service}")),
    Action(SERVICE, "r", "restart it", ("restart", "{service}")),
    Action(SERVICE, "e", "open a shell in it", ("shell", "{service}"), foreground=True),

    Action(VEHICLE, "i", "what it says about itself", ("uas", "{n}", "status")),
    Action(VEHICLE, "t", "take off", ("uas", "{n}", "takeoff", "{value}"),
           ask=Ask("height in metres", "40")),
    Action(VEHICLE, "l", "land", ("uas", "{n}", "land")),
    Action(VEHICLE, "f", "respawn, then climb", ("fly", "{n}", "{value}"),
           ask=Ask("height in metres", "20"),
           confirm="Reload the world, then fly uas{n}?"),
    Action(VEHICLE, "m", "arm", ("uas", "{n}", "arm")),
    Action(VEHICLE, "g", "point the gimbal", ("uas", "{n}", "gimbal", "{value}"),
           ask=Ask("pitch in degrees, below the horizon is negative", "-30")),
    Action(VEHICLE, "z", "set the framing", ("zoom", "{n}", "{value}"),
           ask=Ask("framing", choices_from="zoom_presets")),
    Action(VEHICLE, "d", "continuous detection", ("uas", "{n}", "detect", "{value}"),
           ask=Ask("detection", choices=("on", "off"))),
    Action(VEHICLE, "c", "ask for a capture", ("capture", "{n}", "{value}"),
           ask=Ask("capture", choices_from="capture_kinds")),
    Action(VEHICLE, "p", "what its ROS graph carries", ("probe", "{n}")),
    Action(VEHICLE, "v", "play its gimbal camera", ("view", "{n}"), foreground=True),
    Action(VEHICLE, "w", "save one frame", ("snap", "{gimbal}")),
    Action(VEHICLE, "-", "retire this vehicle",
           ("fleet", "remove", "{n}", "--renumber"),
           confirm="Retire uas{n}? The world reloads. A vehicle before the last"
                   " renumbers every vehicle after it.",
           refresh=True),
    Action(VEHICLE, "o", "follow the companion log", ("logs", "{companion}")),
    Action(VEHICLE, "", "follow the router log", ("logs", "{router}")),
    Action(VEHICLE, "", "what it found, and where that landed",
           ("uas", "{n}", "detections")),
    Action(VEHICLE, "", "which way it and its camera point", ("uas", "{n}", "heading")),
    Action(VEHICLE, "", "the scene the 3D panel is given", ("uas", "{n}", "scene")),
    Action(VEHICLE, "", "go to a place over home", ("uas", "{n}", "goto", "{value}"),
           ask=Ask("east north up, in metres", "0 0 20", split=True)),
    Action(VEHICLE, "", "its ROS topics", ("topics", "{n}")),
    Action(VEHICLE, "", "what Foxglove is offered", ("foxglove", "{n}")),
    Action(VEHICLE, "", "one PX4 command", ("px4", "{n}", "{value}"),
           ask=Ask("PX4 command", "commander status", split=True)),
    Action(VEHICLE, "", "open a shell in the companion", ("onboard", "{n}"),
           foreground=True),

    Action(STREAM, "v", "play it", ("view", "{stream}"), foreground=True),
    Action(STREAM, "w", "save one frame", ("snap", "{stream}")),
)


def fleet_words(count: int) -> str:
    return f"{count} vehicle" if count == 1 else f"{count} vehicles" if count else ""


def strip_codes(line: str) -> str:
    return ESCAPE_CODES.sub("", line).expandtabs(8).rstrip()


def cursor(shown: int) -> None:
    """Some terminals cannot hide the cursor. That is no reason to stop."""
    try:
        curses.curs_set(shown)
    except curses.error:
        pass


def stop_process(process: subprocess.Popen | None) -> None:
    """Stop a command and everything it started.

    Each child gets a session of its own, so this reaches the shell, the tool
    it runs and anything that tool started. A signal to the shell alone leaves
    the tool behind, still holding its sockets.
    """
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except OSError:
        process.terminate()


class Feed:
    """The report stream from `px4sim state --watch`, kept running."""

    def __init__(self, period: float = REPORT_PERIOD_S):
        self.period = period
        self.report: dict | None = None
        self.at = 0.0
        self.notices: deque[str] = deque(maxlen=40)
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.restarting = False
        self.process: subprocess.Popen | None = None
        threading.Thread(target=self.run, daemon=True).start()

    def run(self) -> None:
        while not self.stopping.is_set():
            try:
                self.process = subprocess.Popen(
                    [str(FRONT_DOOR), "state", "--watch", str(self.period)],
                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE, text=True, errors="replace",
                    cwd=FRONT_DOOR.parent, start_new_session=True)
            except OSError as failure:
                self.note(f"cannot run {FRONT_DOOR}: {failure}")
                self.stopping.wait(FEED_RETRY_S)
                continue
            threading.Thread(target=self.drain, args=(self.process.stderr,),
                             daemon=True).start()
            for line in self.process.stdout:
                self.take(line)
            self.process.wait()
            if self.restarting:
                self.restarting = False
            elif not self.stopping.is_set():
                self.note(f"the report stream stopped ({self.process.returncode}). "
                          f"It starts again in {FEED_RETRY_S:.0f}s.")
                self.stopping.wait(FEED_RETRY_S)

    def take(self, line: str) -> None:
        line = line.strip()
        if not line:
            return
        try:
            report = json.loads(line)
        except json.JSONDecodeError:
            self.note(strip_codes(line))
            return
        with self.lock:
            self.report = report
            self.at = time.monotonic()

    def drain(self, pipe) -> None:
        for line in pipe:
            self.note(strip_codes(line))

    def note(self, line: str) -> None:
        if line:
            self.notices.append(line)

    def latest(self) -> tuple[dict | None, float]:
        with self.lock:
            return self.report, (time.monotonic() - self.at if self.report else 0.0)

    def restart(self) -> None:
        """Read the facts again. A new world flies at another place."""
        self.restarting = True
        stop_process(self.process)

    def close(self) -> None:
        self.stopping.set()
        stop_process(self.process)


class Runner:
    """One px4sim command at a time, with its output kept for the pane."""

    def __init__(self):
        self.lines: deque[str] = deque(maxlen=OUTPUT_LINES)
        self.command = ""
        self.started_at = 0.0
        self.finished_at = 0.0
        self.returncode: int | None = None
        self.process: subprocess.Popen | None = None
        self.lock = threading.Lock()

    @property
    def busy(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self, arguments: list[str]) -> bool:
        if self.busy:
            return False
        self.lines.clear()
        self.command = "./px4sim " + " ".join(arguments)
        self.started_at = time.monotonic()
        self.finished_at = 0.0
        self.returncode = None
        try:
            self.process = subprocess.Popen(
                [str(FRONT_DOOR), *arguments], stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                errors="replace", cwd=FRONT_DOOR.parent, start_new_session=True)
        except OSError as failure:
            self.lines.append(f"cannot run {FRONT_DOOR}: {failure}")
            self.returncode = 127
            self.finished_at = time.monotonic()
            return True
        threading.Thread(target=self.collect, args=(self.process,), daemon=True).start()
        return True

    def collect(self, process: subprocess.Popen) -> None:
        """Keep what this command says, for as long as it is the one running.

        A command started while the one before it still drains its pipe must
        not be given the older lines, or the older exit status.
        """
        for line in process.stdout:
            clean = strip_codes(line)
            with self.lock:
                if self.process is not process:
                    return
                if clean:
                    self.lines.append(clean)
        process.wait()
        with self.lock:
            if self.process is process:
                self.returncode = process.returncode
                self.finished_at = time.monotonic()

    def cancel(self) -> None:
        if not self.busy:
            return
        stop_process(self.process)
        with self.lock:
            self.lines.append("-- stopped --")

    def tail(self, rows: int, back: int = 0) -> list[str]:
        with self.lock:
            held = list(self.lines)
        end = max(rows, len(held) - back)
        return held[max(0, end - rows):end]

    def held(self) -> int:
        with self.lock:
            return len(self.lines)

    def state(self) -> str:
        if self.busy:
            return f"running  {time.monotonic() - self.started_at:.0f}s"
        if self.returncode is None:
            return ""
        took = self.finished_at - self.started_at
        return f"{'done' if self.returncode == 0 else f'failed {self.returncode}'}  {took:.0f}s"


def kbits_words(kbits: float | None) -> str:
    if kbits is None:
        return "-"
    if kbits >= 1000:
        return f"{kbits / 1000:.1f} Mbit/s"
    return f"{kbits:.0f} kbit/s"


def service_words(row: dict) -> tuple[str, str]:
    state = row.get("state", "")
    status = row.get("status") or state
    if state == "running":
        return status, "good"
    if state in ("restarting", "created", "paused"):
        return status, "watch"
    if state == "absent":
        return "not created", "faint"
    if state == "exited":
        return f"exited ({row.get('exit_code', 0)})", "bad"
    return status or state or "unknown", "bad"


def stream_words(path: dict) -> list[tuple[str, str]]:
    flowing = bool(path.get("ready")) and bool(path.get("kbits"))
    return [("online" if path.get("ready") else "offline",
             "good" if flowing else ("watch" if path.get("ready") else "faint")),
            (f"{path.get('readers', 0)} readers", "faint"),
            (kbits_words(path.get("kbits")), "good" if flowing else "faint")]


def vehicle_words(vehicle: dict) -> list[tuple[str, str]]:
    link = vehicle.get("link", "down")
    parts = [(f"link {link}", {"up": "good", "silent": "watch"}.get(link, "bad"))]
    if link == "up":
        parts.append((f"{vehicle.get('messages_per_s', 0):.0f}/s", "faint"))
        parts.append(("ARMED" if vehicle.get("armed") else "disarmed",
                      "watch" if vehicle.get("armed") else "faint"))
        parts.append((str(vehicle.get("mode", "-")), "plain"))
        height = vehicle.get("altitude_home")
        if height is not None:
            parts.append((f"{height:.1f} m", "plain"))
        pitch = vehicle.get("gimbal_pitch")
        if pitch is not None:
            parts.append((f"gimbal {pitch:+.0f}", "plain"))
    elif vehicle.get("error"):
        parts.append((vehicle["error"], "faint"))
    return parts


def vehicle_detail(vehicle: dict, model: str, zoom: str) -> str:
    if vehicle.get("link") != "up":
        return model
    words = [model]
    charge = vehicle.get("battery_percent")
    volts = vehicle.get("battery_volts")
    if charge is not None or volts is not None:
        words.append("battery " + " ".join(
            said for said in (f"{charge}%" if charge is not None else "",
                              f"{volts:.1f} V" if volts is not None else "") if said))
    if vehicle.get("gps_fix"):
        words.append(f"gps {vehicle['gps_fix']} / {vehicle.get('satellites', 0)}")
    if vehicle.get("landed"):
        words.append(str(vehicle["landed"]))
    if vehicle.get("heading") is not None:
        words.append(f"heading {vehicle['heading']:.0f}")
    if vehicle.get("gimbal_yaw") is not None and vehicle.get("gimbal_yaw_from"):
        words.append(f"gimbal yaw {vehicle['gimbal_yaw']:.0f}"
                     f" from {vehicle['gimbal_yaw_from']}")
    if zoom:
        words.append(f"framing {zoom}")
    if 255 not in vehicle.get("systems", []):
        words.append("no ground station heartbeat")
    return "   ".join(words)


def card_words(card: dict) -> str:
    if not card:
        return ""
    return (f"gpu {card['busy_percent']:.0f}%   "
            f"vram {card['memory_used_mb'] / 1024:.1f}"
            f"/{card['memory_total_mb'] / 1024:.1f} GB   "
            f"encoders {card['encoder_sessions']}")


def vehicle_place(reported: dict, vehicle: dict) -> str:
    words = []
    if reported.get("latitude") is not None:
        words.append(f"{reported['latitude']:.7f}, {reported['longitude']:.7f}")
    if reported.get("altitude_amsl") is not None:
        words.append(f"{reported['altitude_amsl']:.1f} m amsl")
    if vehicle.get("domain"):
        words.append(f"ros domain {vehicle['domain']}")
    if vehicle.get("companion"):
        words.append(f"foxglove {vehicle.get('foxglove', '')}")
    words.append(f"mavlink tcp {reported.get('port', '')}")
    return "   ".join(words)


def gloss(action: Action) -> str:
    """The px4sim word this action runs, for the row of keys along the foot."""
    words = [word for word in action.command if not word.startswith("{")]
    if not words:
        return ""
    if len(words) > 1 and words[0] == "uas":
        return words[1]
    if len(words) > 2 and words[1].startswith("-"):
        return f"{words[0]} {words[2]}"
    return words[0]


class Rows:
    """What each pane shows. One report fills all of them."""

    def __init__(self, report: dict | None):
        report = report or {}
        self.config = report.get("config") or {}
        self.fleet = {int(vehicle.get("n", 0)): vehicle
                      for vehicle in self.config.get("fleet") or []}
        self.services = report.get("services") or []
        self.vehicles = report.get("vehicles") or []
        self.streams = report.get("streams") or []
        self.bridges = report.get("bridges") or []
        self.gpu = report.get("gpu") or {}
        self.errors = [report.get("services_error", ""), report.get("streams_error", "")]

    def of(self, pane: str) -> list[dict]:
        return {SERVICE: self.services, VEHICLE: self.vehicles,
                STREAM: self.streams}[pane]

    def placeholders(self, pane: str, row: dict) -> dict[str, str]:
        if pane == SERVICE:
            return {"service": row.get("service", "")}
        if pane == STREAM:
            return {"stream": row.get("name", "")}
        vehicle = self.fleet.get(int(row.get("number", 0)), {})
        return {"n": str(row.get("number", "")),
                "gimbal": str(vehicle.get("gimbal", "")),
                "router": str(vehicle.get("router", "")),
                "companion": str(vehicle.get("companion", ""))}

    def name(self, pane: str, row: dict) -> str:
        return {SERVICE: row.get("service", ""), STREAM: row.get("name", ""),
                VEHICLE: f"uas{row.get('number', '')}"}[pane]


class Console:
    """The screen: the panes, the keys, and the command that is running."""

    def __init__(self, screen, feed: Feed, runner: Runner):
        self.screen = screen
        self.feed = feed
        self.runner = runner
        self.pane = VEHICLE
        self.selected = {pane: 0 for pane in PANES}
        self.message = ""
        self.rows = Rows(None)
        self.age = 0.0
        self.paint = {}
        self.refresh_when_done = False
        self.scrolled_back = 0

    # ---------------------------------------------------------------- drawing

    def colors(self) -> None:
        pairs = {"good": curses.COLOR_GREEN, "watch": curses.COLOR_YELLOW,
                 "bad": curses.COLOR_RED, "cool": curses.COLOR_CYAN}
        self.paint = {"plain": curses.A_NORMAL, "faint": curses.A_DIM,
                      "title": curses.A_BOLD, "bar": curses.A_REVERSE}
        if not curses.has_colors():
            self.paint.update({name: curses.A_NORMAL for name in pairs})
            self.paint["bad"] = curses.A_BOLD
            return
        curses.start_color()
        curses.use_default_colors()
        for index, (name, color) in enumerate(pairs.items(), start=1):
            curses.init_pair(index, color, -1)
            self.paint[name] = curses.color_pair(index)

    def put(self, row: int, column: int, text: str, style: str = "plain",
            room: int | None = None) -> None:
        height, width = self.screen.getmaxyx()
        space = width - column - 1
        if room is not None:
            space = min(space, room)
        if row < 0 or row >= height or column < 0 or space <= 0:
            return
        try:
            self.screen.addnstr(row, column, text, space,
                                self.paint.get(style, curses.A_NORMAL))
        except curses.error:
            pass

    def rule(self, row: int) -> None:
        _height, width = self.screen.getmaxyx()
        self.put(row, 0, "─" * (width - 1), "faint")

    def draw(self) -> None:
        self.screen.erase()
        height, width = self.screen.getmaxyx()
        self.draw_header(width)
        output_rows = max(4, int(height * OUTPUT_ROWS_SHARE))
        body_top, body_end = 3, max(5, height - output_rows - 2)
        self.draw_body(body_top, body_end, width)
        self.rule(body_end)
        self.draw_output(body_end + 1, height - 1, width)
        self.draw_footer(height - 1, width)
        self.screen.noutrefresh()

    def draw_header(self, width: int) -> None:
        config = self.rows.config
        clock = time.strftime("%H:%M:%S")
        self.put(0, 0, " " * (width - 1), "bar")
        self.put(0, 1, "px4sim", "bar")
        origin = ""
        if config.get("home_lat") not in (None, ""):
            origin = f"{config.get('home_lat')}, {config.get('home_lon')}"
        told = "   ".join(word for word in (
            str(config.get("scene", "")), str(config.get("scenario", "")), origin,
            fleet_words(len(self.rows.fleet))) if word)
        self.put(0, 9, told, "bar", width - len(clock) - 12)
        self.put(0, max(9, width - len(clock) - 2), clock, "bar")

        state, style = "waiting for the first report", "faint"
        if self.rows.config:
            state = f"reported {self.age:.0f}s ago"
            style = "faint" if self.age < STALE_REPORT_S else "bad"
        faults = [fault for fault in self.rows.errors if fault]
        line = self.message or (faults[0] if faults
                                else f"profiles {config.get('profiles', '-')}")
        card = card_words(self.rows.gpu)
        self.put(1, 1, f"{state}    {line}", "watch" if self.message else style,
                 width - len(card) - 4)
        if card:
            self.put(1, max(1, width - len(card) - 2), card, "faint")
        self.rule(2)

    def draw_body(self, top: int, end: int, width: int) -> None:
        left = max(24, min(38, width // 3))
        room = width - left - 1
        self.draw_pane(SERVICE, "SERVICES", top, end, 1, left - 4)
        for row in range(top, end):
            self.put(row, left - 2, "│", "faint")

        given = self.share_rows(end - top)
        row = top
        for name, title, drawn in (
                (VEHICLE, "VEHICLES", self.draw_pane),
                ("detail", "", self.draw_detail),
                (STREAM, "STREAMS", self.draw_pane),
                ("bridges", "FOXGLOVE", self.draw_bridges)):
            if name not in given:
                continue
            below = row + given[name]
            if name in PANES:
                drawn(name, title, row, below, left, room)
            else:
                drawn(row, below, left, room)
            row = below + 1

    def share_rows(self, space: int) -> dict[str, int]:
        """What each part of the right hand column gets, and what it does without.

        A short terminal keeps the vehicles and the streams. What is left over
        goes to the detail of the selected vehicle and to the Foxglove ports.
        """
        wanted = {VEHICLE: max(2, len(self.rows.vehicles) + 1),
                  STREAM: max(2, len(self.rows.streams) + 1),
                  "detail": 3, "bridges": 2}
        given = {}
        for name in (VEHICLE, STREAM, "detail", "bridges"):
            take = min(wanted[name], space - 1)
            if take >= 2:
                given[name] = take
                space -= take + 1
        return given

    def draw_pane(self, pane: str, title: str, top: int, end: int,
                  column: int, room: int) -> None:
        if top + 1 >= end:
            return
        focused = self.pane == pane
        self.put(top, column, title, "title" if focused else "faint", room)
        rows = self.rows.of(pane)
        if not rows:
            self.put(top + 1, column + 2, "none", "faint", room - 2)
            return
        chosen = self.selected[pane] = min(self.selected[pane], len(rows) - 1)
        room_for = end - top - 1
        first = max(0, min(chosen - room_for + 1, len(rows) - room_for))
        for index, row in enumerate(rows[first:first + room_for]):
            self.draw_row(pane, row, top + 1 + index, column,
                          first + index == chosen and focused, room)
        shown = min(room_for, len(rows) - first)
        if shown < len(rows):
            self.put(top, column + len(title) + 1,
                     f"{first + 1} to {first + shown} of {len(rows)}", "faint",
                     room - len(title) - 1)

    def draw_row(self, pane: str, row: dict, line: int, column: int,
                 chosen: bool, room: int) -> None:
        self.put(line, column, "▸" if chosen else " ", "cool" if chosen else "plain")
        name = self.rows.name(pane, row)
        # One column for every name, so the readings line up. A narrow pane
        # gives the name less room, because the reading is the point of the row.
        name_column = min(NAME_COLUMN, max(9, room - 12))
        self.put(line, column + 2, name, "title" if chosen else "plain", name_column - 1)
        at = column + 2 + name_column
        for words, kind in self.row_words(pane, row):
            left_over = room - (at - column)
            if left_over <= 1:
                return
            self.put(line, at, words, kind, left_over)
            at += len(words) + 2

    def row_words(self, pane: str, row: dict) -> list[tuple[str, str]]:
        if pane == SERVICE:
            words, kind = service_words(row)
            said = [(words, kind)]
            if not row.get("wanted", True) and row.get("state") == "running":
                said.append(("no profile selects it", "faint"))
            return said
        if pane == STREAM:
            return stream_words(row)
        return vehicle_words(row)

    def draw_detail(self, top: int, end: int, column: int, room: int) -> None:
        row = self.selected_row(VEHICLE)
        if row is None or top + 1 >= end:
            return
        number = int(row.get("number", 0))
        vehicle = self.rows.fleet.get(number, {})
        self.put(top, column, f"UAS{number}", "faint", room)
        self.put(top + 1, column + 2,
                 vehicle_detail(row, str(vehicle.get("model", "")),
                                str(vehicle.get("zoom", ""))), "plain", room - 2)
        if top + 2 < end:
            self.put(top + 2, column + 2, vehicle_place(row, vehicle), "faint",
                     room - 2)

    def draw_bridges(self, top: int, end: int, column: int, room: int) -> None:
        if top + 1 >= end or not self.rows.bridges:
            return
        self.put(top, column, "FOXGLOVE", "faint", room)
        at = column + 2
        for bridge in self.rows.bridges:
            words = f"{bridge['name']} {bridge['port']}"
            self.put(top + 1, at, words, "good" if bridge["listening"] else "faint",
                     room - (at - column))
            at += len(words) + 3

    def draw_output(self, top: int, end: int, width: int) -> None:
        title = self.runner.command or "nothing has been run yet"
        self.put(top, 1, title[:max(10, width - 24)], "title")
        state = self.runner.state()
        self.put(top, max(1, width - len(state) - 2), state,
                 "watch" if self.runner.busy else "faint")
        rows = end - top - 1
        if rows <= 0:
            return
        self.scrolled_back = min(self.scrolled_back, max(0, self.runner.held() - rows))
        lines = self.runner.tail(rows, self.scrolled_back)
        if not self.runner.command and self.feed.notices:
            lines = list(self.feed.notices)[-rows:]
        for index, line in enumerate(lines):
            self.put(top + 1 + index, 1, line)
        if self.scrolled_back:
            self.put(top, max(1, width - len(state) - 22),
                     f"{self.scrolled_back} lines below", "faint")

    def draw_footer(self, row: int, width: int) -> None:
        keys = ["tab pane", "enter actions", ". stack", ": command",
                "pgup output", "? help", "q quit"]
        keys += [f"{action.key} {gloss(action)}" for action in ACTIONS
                 if action.key and action.scope == self.pane]
        self.put(row, 0, " " + " · ".join(keys), "faint")

    # ---------------------------------------------------------------- asking

    def overlay(self, title: str, lines: list[str], chosen: int = -1) -> None:
        height, width = self.screen.getmaxyx()
        if not lines:
            return
        room = max(1, min(len(lines), height - 6))
        inside = max(8, min(width - 6, max(len(title) + 4,
                                           max(len(line) for line in lines) + 4)))
        top = max(1, (height - room) // 2 - 2)
        left = max(1, (width - inside - 2) // 2)
        first = max(0, min(chosen - room + 1, len(lines) - room))
        edge = "─" * inside
        self.put(top, left, f"┌{edge}┐", "faint")
        self.put(top, left + 2, f" {title} ", "title")
        for index in range(room):
            source = first + index
            picked = source == chosen
            words = ("▸ " if picked else "  ") + lines[source]
            self.put(top + 1 + index, left, "│", "faint")
            self.put(top + 1 + index, left + 1, words[:inside].ljust(inside),
                     "title" if picked else "plain")
            self.put(top + 1 + index, left + 1 + inside, "│", "faint")
        self.put(top + room + 1, left, f"└{edge}┘", "faint")
        if len(lines) > room:
            self.put(top + room + 1, left + 2,
                     f" {first + 1} to {first + room} of {len(lines)} ", "faint")
        self.screen.noutrefresh()

    def choose(self, title: str, labels: list[str]) -> int:
        if not labels:
            return -1
        chosen = 0
        try:
            while True:
                self.draw()
                self.overlay(title, labels, chosen)
                curses.doupdate()
                key = self.screen.getch()
                if key in (curses.KEY_UP, ord("k")):
                    chosen = (chosen - 1) % len(labels)
                elif key in (curses.KEY_DOWN, ord("j")):
                    chosen = (chosen + 1) % len(labels)
                elif key in (curses.KEY_ENTER, 10, 13):
                    return chosen
                elif key in (27, ord("q")):
                    return -1
        except KeyboardInterrupt:
            return -1

    def ask_text(self, prompt: str, default: str = "") -> str | None:
        answer = default
        cursor(1)
        try:
            while True:
                self.draw()
                height, _width = self.screen.getmaxyx()
                self.put(height - 1, 0, " " * 200, "bar")
                self.put(height - 1, 1, f"{prompt}: {answer}", "bar")
                self.screen.noutrefresh()
                curses.doupdate()
                key = self.screen.getch()
                if key in (curses.KEY_ENTER, 10, 13):
                    return answer.strip()
                if key == 27:
                    return None
                if key in (curses.KEY_BACKSPACE, 127, 8):
                    answer = answer[:-1]
                elif 32 <= key < 127:
                    answer += chr(key)
        except KeyboardInterrupt:
            return None
        finally:
            cursor(0)

    def confirm(self, question: str) -> bool:
        return self.choose(question, ["no", "yes"]) == 1

    # ---------------------------------------------------------------- acting

    def actions_for(self, pane: str) -> list[Action]:
        return [action for action in ACTIONS if action.scope == pane]

    def selected_row(self, pane: str) -> dict | None:
        rows = self.rows.of(pane)
        if not rows:
            return None
        return rows[min(self.selected[pane], len(rows) - 1)]

    def placeholders(self) -> dict[str, str]:
        row = self.selected_row(self.pane)
        if row is None:
            return {}
        return self.rows.placeholders(self.pane, row)

    def run(self, action: Action) -> None:
        filling = self.placeholders() if action.scope != STACK else {}
        if action.scope != STACK and not filling:
            self.message = f"no {action.scope} is selected"
            return
        value = ""
        if action.ask is not None:
            value = self.answer(action.ask)
            if value is None:
                return
        if action.confirm and not self.confirm(self.fill(action.confirm, filling, value)):
            return
        arguments: list[str] = []
        for token in action.command:
            filled = self.fill(token, filling, value)
            if token.startswith("{") and token != "{value}" and not filled:
                self.message = (f"{self.rows.name(self.pane, self.selected_row(self.pane))}"
                                f" has no {token.strip('{}')}")
                return
            arguments += filled.split() if (token == "{value}" and action.ask
                                            and action.ask.split) else [filled]
        arguments = [word for word in arguments if word]
        if action.foreground:
            self.hand_over(arguments)
            if action.refresh:
                self.feed.restart()
        elif self.runner.start(arguments):
            self.scrolled_back = 0
            self.refresh_when_done = action.refresh
            self.message = ""
        else:
            self.message = "one command runs at a time. Press esc to stop it."

    def answer(self, ask: Ask) -> str | None:
        choices = list(ask.choices)
        if ask.choices_from:
            choices = [word for word in
                       str(self.rows.config.get(ask.choices_from, "")).split(",") if word]
        if choices:
            picked = self.choose(ask.prompt, choices)
            return None if picked < 0 else choices[picked]
        return self.ask_text(ask.prompt, self.fill_config(ask.default))

    def fill_config(self, text: str) -> str:
        """A default that names a fact, such as {scene}, starts at what it holds.

        A fact the newest report does not carry leaves nothing behind. Before
        the first report that is every fact, and a name in braces is not a
        scene.
        """
        for name, held in self.rows.config.items():
            if isinstance(held, (str, int, float)):
                text = text.replace("{" + name + "}", str(held))
        return re.sub(r"\{[a-z_]+\}", "", text)

    def fill(self, text: str, filling: dict[str, str], value: str) -> str:
        for name, replacement in {**filling, "value": value}.items():
            text = text.replace("{" + name + "}", replacement)
        return text

    def hand_over(self, arguments: list[str]) -> None:
        """Give the terminal to a command that needs it, then take it back."""
        curses.def_prog_mode()
        curses.endwin()
        try:
            subprocess.call([str(FRONT_DOOR), *arguments], cwd=FRONT_DOOR.parent)
        except KeyboardInterrupt:
            pass
        except OSError as failure:
            print(f"cannot run {FRONT_DOOR}: {failure}")
        try:
            input("\npx4sim: push Enter to go back to the console. ")
        except (EOFError, KeyboardInterrupt):
            pass
        curses.reset_prog_mode()
        self.screen.clear()
        self.screen.refresh()

    def menu(self) -> None:
        actions = self.actions_for(self.pane)
        row = self.selected_row(self.pane)
        if row is None:
            self.message = f"there is no {self.pane} to act on"
            return
        name = self.rows.name(self.pane, row)
        labels = [f"{action.key or ' '}  {action.label}" for action in actions]
        picked = self.choose(f"{name}: what to do", labels)
        if picked >= 0:
            self.run(actions[picked])

    def stack_menu(self) -> None:
        actions = self.actions_for(STACK)
        picked = self.choose("the stack: what to do",
                             [f"{action.key or ' '}  {action.label}" for action in actions])
        if picked >= 0:
            self.run(actions[picked])

    def help(self) -> None:
        lines = []
        for scope in (STACK, SERVICE, VEHICLE, STREAM):
            lines.append(f"-- {scope} --")
            for action in self.actions_for(scope):
                command = " ".join(action.command)
                lines.append(f"{action.key or ' ':<2} {action.label:<44} px4sim {command}")
        self.choose("every action, and what it runs", lines)

    def typed(self) -> None:
        typed = self.ask_text("px4sim", "")
        if typed and not self.runner.start(typed.split()):
            self.message = "one command runs at a time. Press esc to stop it."

    def hotkey(self, key: str) -> bool:
        for scope in (self.pane, STACK):
            for action in self.actions_for(scope):
                if action.key == key:
                    self.run(action)
                    return True
        return False

    # ---------------------------------------------------------------- looping

    def interrupted(self) -> None:
        if self.runner.busy:
            self.runner.cancel()
        else:
            self.message = "press q to leave the console."

    def move(self, step: int) -> None:
        rows = self.rows.of(self.pane)
        if rows:
            self.selected[self.pane] = (self.selected[self.pane] + step) % len(rows)

    def key(self, key: int) -> bool:
        if key in (curses.KEY_UP,):
            self.move(-1)
        elif key in (curses.KEY_DOWN,):
            self.move(1)
        elif key == ord("\t"):
            self.pane = PANES[(PANES.index(self.pane) + 1) % len(PANES)]
        elif key == curses.KEY_PPAGE:
            self.scrolled_back += 5
        elif key == curses.KEY_NPAGE:
            self.scrolled_back = max(0, self.scrolled_back - 5)
        elif key == curses.KEY_BTAB:
            self.pane = PANES[(PANES.index(self.pane) - 1) % len(PANES)]
        elif key in (curses.KEY_ENTER, 10, 13):
            self.menu()
        elif key == ord("."):
            self.stack_menu()
        elif key == ord(":"):
            self.typed()
        elif key == ord("?"):
            self.help()
        elif key == 27:
            self.runner.cancel()
        elif key == ord("q"):
            if not self.runner.busy or self.confirm("A command is running. Leave?"):
                return False
        elif 32 <= key < 127:
            self.message = ""
            if not self.hotkey(chr(key)):
                self.message = f"'{chr(key)}' does nothing here. Press ? for the list."
        return True

    def once(self) -> bool:
        """Take the newest report, draw it, and act on one key."""
        if self.refresh_when_done and not self.runner.busy:
            self.refresh_when_done = False
            self.feed.restart()
        report, age = self.feed.latest()
        if report is not None:
            self.rows = Rows(report)
            self.age = age
        self.draw()
        curses.doupdate()
        key = self.screen.getch()
        if key == curses.KEY_RESIZE:
            self.screen.clear()
        elif key != -1:
            return self.key(key)
        return True

    def loop(self) -> None:
        self.colors()
        cursor(0)
        # Without this, curses waits a second to tell escape from an arrow key.
        curses.set_escdelay(25)
        self.screen.timeout(INPUT_PERIOD_MS)
        # Ctrl-C can land anywhere, and it has never meant "end this console".
        while True:
            try:
                if not self.once():
                    return
            except KeyboardInterrupt:
                self.interrupted()


def main() -> int:
    if not FRONT_DOOR.exists():
        print(f"{FRONT_DOOR} is not here. Run this as ./px4sim ui.", file=sys.stderr)
        return 2
    if not sys.stdout.isatty():
        print("The console draws on a terminal. To read the same picture from a"
              " script, run ./px4sim state.", file=sys.stderr)
        return 2
    feed = Feed()
    runner = Runner()
    try:
        curses.wrapper(lambda screen: Console(screen, feed, runner).loop())
    except curses.error as failure:
        print(f"This terminal cannot draw the console: {failure}", file=sys.stderr)
        print(f"TERM is '{os.environ.get('TERM', '')}'. ./px4sim state prints the"
              f" same picture as JSON.", file=sys.stderr)
        return 2
    finally:
        feed.close()
        runner.cancel()
    return 0


if __name__ == "__main__":
    sys.exit(main())
