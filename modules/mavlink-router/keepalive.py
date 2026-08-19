#!/usr/bin/env python3
"""Inject a ground station heartbeat into mavlink-router.

Why this exists
---------------
A vehicle with no ground station attached fires the PX4 data link loss failsafe.
On the aircraft a ground station is always there. On a desk it often is not, and
the failsafe then interrupts whatever you were testing.

This sends a MAVLink 2 HEARTBEAT once a second to a private endpoint on the
router, as system 255, component 190. The router forwards it to PX4, and PX4
counts a ground station as present.

The transport is UDP by default. TCP works as well, and the private endpoint
keeps the heartbeat off the ports that real clients use.

Modes, set with KEEPALIVE in the environment:
  1, always    send forever. PX4 always sees a ground station, so the data link
               loss failsafe never fires. This is the default and it is what you
               want while you develop.
  bootstrap    stop as soon as vehicle traffic appears. Use this to test the
               data link loss failsafe, because PX4 then notices when the real
               ground station goes away.
  0, off       do not run.

No external dependency. The one message it needs is 12 bytes of header and 9
bytes of payload, so pymavlink would be a heavy way to get it.
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import time

# HEARTBEAT, MAVLink message 0.
MSG_ID_HEARTBEAT = 0
CRC_EXTRA_HEARTBEAT = 50

MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4
MAVLINK_VERSION = 3

# A ground station identity that no vehicle uses, and that every router lets
# through: the contract filters are AllowSrcSysIn = <uas>,255.
SYSTEM_ID = 255
COMPONENT_ID = 190  # MAV_COMP_ID_MISSIONPLANNER


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


def heartbeat_frame(seq: int) -> bytes:
    """Build one MAVLink 2 HEARTBEAT frame."""
    # Field order in the payload is by descending field size, then by the order
    # the fields appear in the message definition.
    payload = struct.pack(
        "<IBBBBB",
        0,                      # custom_mode
        MAV_TYPE_GCS,           # type
        MAV_AUTOPILOT_INVALID,  # autopilot
        0,                      # base_mode
        MAV_STATE_ACTIVE,       # system_status
        MAVLINK_VERSION,        # mavlink_version
    )
    header = struct.pack(
        "<BBBBBBBB",
        len(payload),
        0,            # incompat_flags
        0,            # compat_flags
        seq & 0xFF,
        SYSTEM_ID,
        COMPONENT_ID,
        MSG_ID_HEARTBEAT & 0xFF,
        (MSG_ID_HEARTBEAT >> 8) & 0xFF,
    )
    header += bytes([(MSG_ID_HEARTBEAT >> 16) & 0xFF])
    crc = checksum(header + payload, CRC_EXTRA_HEARTBEAT)
    return b"\xfd" + header + payload + struct.pack("<H", crc)


def open_socket(host: str, port: int, transport: str, period: float) -> socket.socket:
    if transport == "tcp":
        sock = socket.create_connection((host, port), timeout=5)
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Connect the datagram socket, so the router replies to this port and
        # recv() only sees traffic meant for it.
        sock.connect((host, port))
    sock.settimeout(period)
    return sock


def run(host: str, port: int, mode: str, period: float, transport: str) -> int:
    seq = 0
    saw_vehicle = False
    log = lambda m: print(f"[keepalive] {m}", flush=True)  # noqa: E731

    log(f"mode={mode} target={transport}://{host}:{port}")

    while True:
        try:
            with open_socket(host, port, transport, period) as sock:
                log("connected to the router")
                while True:
                    sock.send(heartbeat_frame(seq))
                    seq += 1

                    # Anything coming back that is not our own heartbeat means
                    # the vehicle link is alive.
                    try:
                        data = sock.recv(4096)
                        if data == b"" and transport == "tcp":
                            raise ConnectionResetError("router closed the connection")
                        if data and not saw_vehicle:
                            saw_vehicle = True
                            log("vehicle traffic seen, the link is up")
                            if mode == "bootstrap":
                                log("bootstrap mode, stopping")
                                return 0
                    except socket.timeout:
                        pass
                    except ConnectionRefusedError:
                        # UDP: nothing is bound yet. Keep trying.
                        pass

                    time.sleep(period)
        except OSError as exc:
            log(f"{exc}. Retrying in 2 s.")
            time.sleep(2)


# What KEEPALIVE accepts, and the mode each spelling means. None is off.
KEEPALIVE_MODES = {
    "0": None, "off": None, "false": None, "no": None,
    "1": "always", "true": "always", "yes": "always", "always": "always",
    "bootstrap": "bootstrap",
}


def main() -> int:
    setting = os.environ.get("KEEPALIVE", "1").lower()
    if setting not in KEEPALIVE_MODES:
        print(f"[keepalive] KEEPALIVE={setting!r} is not one of "
              f"{sorted(KEEPALIVE_MODES)}. Using always.", flush=True)
    mode = KEEPALIVE_MODES.get(setting, "always")
    if mode is None:
        print("[keepalive] disabled", flush=True)
        return 0

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14599)
    ap.add_argument("--transport", choices=("udp", "tcp"), default="udp")
    ap.add_argument("--period", type=float, default=1.0, help="seconds between heartbeats")
    args = ap.parse_args()

    try:
        return run(args.host, args.port, mode, args.period, args.transport)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
