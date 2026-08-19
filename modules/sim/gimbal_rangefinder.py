#!/usr/bin/env python3
"""Send the gimbal rangefinder to PX4 as MAVLink DISTANCE_SENSOR id 1.

Why this exists
---------------
MAVROS names a rangefinder topic after the MAVLink sensor id, and the flight
code reads id 1:

  id 0  drone_lidar_200m   body fixed, down
  id 1  gimbal_lidar_50m   gimbal boresight
  id 2  drone_lidar_6m     body fixed, down

PX4 numbers the id from the uORB instance: DISTANCE_SENSOR.hpp sends `msg.id = i`
for instance i. PX4's Gazebo bridge maps exactly one lidar, from the fixed topic
`.../link/lidar_sensor_link/sensor/lidar/scan`, and publishes it on instance 0.
So the body rangefinder is id 0 for free, and nothing in PX4 produces a second
instance.

`MavlinkReceiver::handle_message_distance_sensor` publishes what it receives on
a `uORB::PublicationMulti`, which takes the next free instance. So one
DISTANCE_SENSOR sent into PX4 becomes instance 1, and PX4 streams it back out
as id 1. That is the whole trick.

The order matters and it fails silently if it goes wrong: whoever advertises
first owns instance 0. So this waits until PX4 reports a DISTANCE_SENSOR with
id 0 before it sends its own. Until then it only heartbeats, which is also what
teaches PX4's UDP server link where to reply.

The PX4 link this talks to is started by px4-rcS in minimal mode with one added
stream, so the read-back costs almost nothing.

No MAVLink library. Two messages out and one message in is less code than the
dependency.
"""

from __future__ import annotations

import argparse
import math
import re
import selectors
import socket
import struct
import subprocess
import sys
import time

# ---------------------------------------------------------------- tunables
DEFAULT_SENSOR_ID = 1
DEFAULT_MAX_RATE_HZ = 10.0
HEARTBEAT_PERIOD_S = 1.0
GZ_TOPIC_POLL_S = 2.0
WAITING_FOR_PX4_WARN_S = 30.0
# Only used until the first scan arrives with the real limits in it.
FALLBACK_RANGE_M = (0.2, 50.0)

MSG_ID_HEARTBEAT = 0
MSG_ID_DISTANCE_SENSOR = 132
CRC_EXTRA_HEARTBEAT = 50
CRC_EXTRA_DISTANCE_SENSOR = 85

MAV_TYPE_ONBOARD_CONTROLLER = 18
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4
MAVLINK_VERSION = 3
MAV_COMP_ID_ONBOARD_COMPUTER = 191

MAV_DISTANCE_SENSOR_LASER = 0
MAV_SENSOR_ROTATION_NONE = 0  # what px4_config.yaml gives gimbal_lidar_50m

# gz prints a LaserScan as text. These three lines are all this needs, and
# taking the limits from the message keeps model.sdf the only place they live.
RANGES_LINE = re.compile(r"^\s*ranges:\s*(\S+)")
RANGE_MIN_LINE = re.compile(r"^\s*range_min:\s*(\S+)")
RANGE_MAX_LINE = re.compile(r"^\s*range_max:\s*(\S+)")


def crc_accumulate(byte: int, crc: int) -> int:
    """One step of the X25 checksum that MAVLink uses."""
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def checksum(data: bytes, crc_extra: int) -> int:
    crc = 0xFFFF
    for byte in data:
        crc = crc_accumulate(byte, crc)
    return crc_accumulate(crc_extra, crc)


def frame(msg_id: int, payload: bytes, crc_extra: int, seq: int,
          sysid: int, compid: int) -> bytes:
    header = struct.pack(
        "<BBBBBBBBB",
        len(payload),
        0,  # incompat_flags
        0,  # compat_flags
        seq & 0xFF,
        sysid,
        compid,
        msg_id & 0xFF,
        (msg_id >> 8) & 0xFF,
        (msg_id >> 16) & 0xFF,
    )
    crc = checksum(header + payload, crc_extra)
    return b"\xfd" + header + payload + struct.pack("<H", crc)


def heartbeat(seq: int, sysid: int) -> bytes:
    payload = struct.pack(
        "<IBBBBB",
        0,                            # custom_mode
        MAV_TYPE_ONBOARD_CONTROLLER,  # type
        MAV_AUTOPILOT_INVALID,        # autopilot
        0,                            # base_mode
        MAV_STATE_ACTIVE,             # system_status
        MAVLINK_VERSION,
    )
    return frame(MSG_ID_HEARTBEAT, payload, CRC_EXTRA_HEARTBEAT, seq,
                 sysid, MAV_COMP_ID_ONBOARD_COMPUTER)


def distance_sensor(seq: int, sysid: int, sensor_id: int, distance_m: float,
                    min_m: float, max_m: float, valid: bool) -> bytes:
    payload = struct.pack(
        "<IHHHBBBBffffffB",
        int(time.monotonic() * 1000) & 0xFFFFFFFF,  # time_boot_ms
        int(min_m * 100),
        int(max_m * 100),
        int(distance_m * 100),
        MAV_DISTANCE_SENSOR_LASER,
        sensor_id,
        MAV_SENSOR_ROTATION_NONE,
        255,                       # covariance, UINT8_MAX is "unknown"
        0.0, 0.0,                  # horizontal_fov, vertical_fov
        0.0, 0.0, 0.0, 0.0,        # quaternion, unused with a named rotation
        100 if valid else 1,       # signal_quality, 1 means no return
    )
    return frame(MSG_ID_DISTANCE_SENSOR, payload, CRC_EXTRA_DISTANCE_SENSOR, seq,
                 sysid, MAV_COMP_ID_ONBOARD_COMPUTER)


def sensor_ids_in(datagram: bytes):
    """Every DISTANCE_SENSOR id in one datagram. PX4 packs several messages
    into one, so this walks the whole buffer."""
    i, n = 0, len(datagram)
    while i < n:
        if datagram[i] != 0xFD:
            i += 1
            continue
        if i + 12 > n:
            return
        length = datagram[i + 1]
        signed = datagram[i + 2] & 0x01
        msg_id = datagram[i + 7] | datagram[i + 8] << 8 | datagram[i + 9] << 16
        if msg_id == MSG_ID_DISTANCE_SENSOR and length >= 12:
            yield datagram[i + 10 + 11]
        i += 12 + length + (13 if signed else 0)


def wait_for_topic(topic: str, log) -> None:
    """Block until the lidar advertises, complaining while it does not.

    A model name that does not match the spawned entity waits here forever. One
    line at the start would leave that looking like a stack that is merely slow,
    so say it again periodically and name the topic being waited for.
    """
    log(f"waiting for {topic}")
    complain_at = time.monotonic() + WAITING_FOR_PX4_WARN_S
    while True:
        listed = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True)
        if topic in listed.stdout.split():
            log("gazebo topic is up")
            return
        now = time.monotonic()
        if now >= complain_at:
            log(f"still waiting for {topic}. Check that the model name matches "
                f"the entity Gazebo spawned, which is <model>_<px4 instance>.")
            complain_at = now + WAITING_FOR_PX4_WARN_S
        time.sleep(GZ_TOPIC_POLL_S)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", required=True)
    ap.add_argument("--model", required=True, help="Gazebo model instance, e.g. chimera_v3_0")
    ap.add_argument("--link", default="gimbal_lidar_link")
    ap.add_argument("--sensor", default="gimbal_lidar")
    ap.add_argument("--sysid", type=int, required=True)
    ap.add_argument("--px4-host", default="127.0.0.1")
    ap.add_argument("--px4-port", type=int, required=True)
    ap.add_argument("--sensor-id", type=int, default=DEFAULT_SENSOR_ID)
    ap.add_argument("--max-rate", type=float, default=DEFAULT_MAX_RATE_HZ)
    args = ap.parse_args()

    tag = f"[gimbal-rangefinder uas{args.sysid}]"
    def log(message):
        print(f"{tag} {message}", flush=True)

    topic = (f"/world/{args.world}/model/{args.model}"
             f"/link/{args.link}/sensor/{args.sensor}/scan")
    wait_for_topic(topic, log)

    link = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    link.connect((args.px4_host, args.px4_port))
    link.setblocking(False)
    selector = selectors.DefaultSelector()
    selector.register(link, selectors.EVENT_READ)

    seq = 0
    px4_owns_instance_zero = False
    next_heartbeat = 0.0
    complain_at = time.monotonic() + WAITING_FOR_PX4_WARN_S
    min_period = 1.0 / args.max_rate if args.max_rate > 0 else 0.0
    next_send = 0.0
    min_m, max_m = FALLBACK_RANGE_M

    # PX4 answers a datagram sent to a link it has not opened yet with an ICMP
    # port unreachable, which a connected UDP socket raises on the next call.
    # That happens while PX4 boots, so it is normal and it must not stop this.
    def send(message):
        nonlocal seq
        try:
            link.send(message)
        except OSError:
            return
        finally:
            seq = (seq + 1) & 0xFF

    def drain():
        nonlocal px4_owns_instance_zero
        while selector.select(0):
            try:
                datagram = link.recv(4096)
            except OSError:
                return
            for sensor_id in sensor_ids_in(datagram):
                if sensor_id == 0 and not px4_owns_instance_zero:
                    px4_owns_instance_zero = True
                    log("PX4 reports its body rangefinder as id 0, sending id "
                        f"{args.sensor_id} now")

    echo = subprocess.Popen(["gz", "topic", "-e", "-t", topic],
                            stdout=subprocess.PIPE, text=True, bufsize=1)
    log(f"sending id {args.sensor_id} to {args.px4_host}:{args.px4_port} as system {args.sysid}")

    try:
        for line in echo.stdout:
            now = time.monotonic()

            if now >= next_heartbeat:
                send(heartbeat(seq, args.sysid))
                next_heartbeat = now + HEARTBEAT_PERIOD_S

            drain()

            if not px4_owns_instance_zero:
                if now >= complain_at:
                    log("still waiting for a DISTANCE_SENSOR id 0 from PX4. Until it "
                        "arrives, sending would take uORB instance 0 and the gimbal "
                        "range would reach MAVROS as drone_lidar_200m.")
                    complain_at = now + WAITING_FOR_PX4_WARN_S
                continue

            limit = RANGE_MIN_LINE.match(line)
            if limit:
                min_m = float(limit.group(1))
                continue
            limit = RANGE_MAX_LINE.match(line)
            if limit:
                max_m = float(limit.group(1))
                continue

            found = RANGES_LINE.match(line)
            if not found or now < next_send:
                continue
            next_send = now + min_period

            # A gpu_lidar returns +inf past its range limit. Sending that
            # would reach the mission node as an infinite range rather than as
            # no reading, because the consumer only takes min(range, 200).
            measured = float(found.group(1))
            valid = math.isfinite(measured) and measured <= max_m
            send(distance_sensor(seq, args.sysid, args.sensor_id,
                                 measured if valid else max_m,
                                 min_m, max_m, valid))
    except KeyboardInterrupt:
        pass
    finally:
        echo.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
