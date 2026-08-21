#!/usr/bin/env python3
"""MAVLink frames from a byte stream, and the fields this stack reads.

A header is ten bytes and this file reads six messages, so a MAVLink library
would be a heavy way to get them. scripts/state.py watches a vehicle with this,
and verify/component/mavlink_census.py counts messages with it.

This returns a frame only when the stream continues with the start of another
frame. A reader that trusts the first start byte it sees can lock onto a byte
inside a payload and report one wrong value before it recovers. The newest
frame therefore waits for the one after it, which is a few milliseconds on a
link that carries a hundred messages a second.
"""

from __future__ import annotations

import math
import struct
from typing import NamedTuple

START_V2 = 0xFD
START_V1 = 0xFE
STARTS = (START_V2, START_V1)
HEADER_V2 = 10
HEADER_V1 = 6
CHECKSUM_BYTES = 2
SIGNATURE_BYTES = 13
SIGNED_FLAG = 0x01

HEARTBEAT = 0
SYS_STATUS = 1
GPS_RAW_INT = 24
GLOBAL_POSITION_INT = 33
EXTENDED_SYS_STATE = 245
GIMBAL_DEVICE_ATTITUDE_STATUS = 285

ARMED_FLAG = 0x80
YAW_IN_VEHICLE_FRAME = 16
YAW_IN_EARTH_FRAME = 32
BATTERY_VOLTS_UNKNOWN = 65535
BATTERY_PERCENT_UNKNOWN = -1

# The mode names MAVROS gives the same vehicle, so the front door, Foxglove and
# this all call one mode by one name. The numbers are the ones PX4 sends, from
# src/modules/commander/px4_custom_mode.h of the PX4 this stack flies.
PX4_MAIN_MODES = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 5: "ACRO",
                  6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE", 9: "SIMPLE",
                  10: "TERMINATION", 11: "ALTITUDE_CRUISE"}
PX4_AUTO_MODE = 4
PX4_AUTO_MODES = {1: "AUTO.READY", 2: "AUTO.TAKEOFF", 3: "AUTO.LOITER",
                  4: "AUTO.MISSION", 5: "AUTO.RTL", 6: "AUTO.LAND",
                  7: "AUTO.RESERVED", 8: "AUTO.FOLLOW_TARGET",
                  9: "AUTO.PRECLAND", 10: "AUTO.VTOL_TAKEOFF"}
GPS_FIXES = {0: "no gps", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS",
             5: "RTK float", 6: "RTK fixed", 7: "static", 8: "PPP"}
LANDED_STATES = {0: "unknown", 1: "on the ground", 2: "in the air",
                 3: "taking off", 4: "landing"}
HEADING_UNKNOWN = 65535


class Frame(NamedTuple):
    system: int
    component: int
    message: int
    payload: bytes


def frame_length(buffer: bytes, index: int) -> int | None:
    """The whole length of the frame at this index, or None to read more."""
    if buffer[index] == START_V2:
        if index + 3 > len(buffer):
            return None
        signed = buffer[index + 2] & SIGNED_FLAG
        return (HEADER_V2 + buffer[index + 1] + CHECKSUM_BYTES
                + (SIGNATURE_BYTES if signed else 0))
    if index + 2 > len(buffer):
        return None
    return HEADER_V1 + buffer[index + 1] + CHECKSUM_BYTES


def read_frame(buffer: bytes, index: int) -> Frame:
    if buffer[index] == START_V2:
        payload = HEADER_V2 + buffer[index + 1]
        return Frame(system=buffer[index + 5], component=buffer[index + 6],
                     message=(buffer[index + 7] | buffer[index + 8] << 8
                              | buffer[index + 9] << 16),
                     payload=buffer[index + HEADER_V2:index + payload])
    payload = HEADER_V1 + buffer[index + 1]
    return Frame(system=buffer[index + 3], component=buffer[index + 4],
                 message=buffer[index + 5],
                 payload=buffer[index + HEADER_V1:index + payload])


def frames(buffer: bytes) -> tuple[list[Frame], bytes]:
    """Every confirmed frame in the buffer, and the bytes that follow them."""
    found: list[Frame] = []
    index = 0
    size = len(buffer)
    while index < size:
        if buffer[index] not in STARTS:
            index += 1
            continue
        length = frame_length(buffer, index)
        if length is None or index + length >= size:
            break
        if buffer[index + length] not in STARTS:
            index += 1
            continue
        found.append(read_frame(buffer, index))
        index += length
    return found, buffer[index:]


def unpack(layout: str, payload: bytes) -> tuple:
    """MAVLink 2 cuts the trailing zeros of a payload, so put them back."""
    size = struct.calcsize(layout)
    return struct.unpack(layout, payload.ljust(size, b"\x00")[:size])


def mode_name(custom_mode: int) -> str:
    main = (custom_mode >> 16) & 0xFF
    if main == PX4_AUTO_MODE:
        sub = (custom_mode >> 24) & 0xFF
        if sub in PX4_AUTO_MODES:
            return PX4_AUTO_MODES[sub]
        return f"AUTO.EXTERNAL{sub - 10}" if 11 <= sub <= 18 else f"AUTO.{sub}"
    return PX4_MAIN_MODES.get(main, f"mode {main}")


def pitch_yaw_deg(w: float, x: float, y: float, z: float) -> tuple[float, float]:
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(pitch), math.degrees(yaw)


def heartbeat(payload: bytes) -> dict:
    custom_mode, _type, _autopilot, base_mode, _status, _version = unpack("<IBBBBB", payload)
    return {"mode": mode_name(custom_mode), "armed": bool(base_mode & ARMED_FLAG)}


def sys_status(payload: bytes) -> dict:
    values = unpack("<IIIHHhHHHHHHb", payload)
    millivolts, percent = values[4], values[12]
    return {"battery_volts": None if millivolts == BATTERY_VOLTS_UNKNOWN
            else millivolts / 1000.0,
            "battery_percent": None if percent == BATTERY_PERCENT_UNKNOWN else percent}


def gps_raw_int(payload: bytes) -> dict:
    values = unpack("<QiiiHHHHBB", payload)
    return {"gps_fix": GPS_FIXES.get(values[8], str(values[8])),
            "satellites": values[9]}


def global_position_int(payload: bytes) -> dict:
    values = unpack("<IiiiihhhH", payload)
    heading = values[8]
    return {"latitude": values[1] / 1e7, "longitude": values[2] / 1e7,
            "altitude_amsl": values[3] / 1000.0,
            "altitude_home": values[4] / 1000.0,
            "heading": None if heading == HEADING_UNKNOWN else heading / 100.0}


def extended_sys_state(payload: bytes) -> dict:
    _vtol, landed = unpack("<BB", payload)
    return {"landed": LANDED_STATES.get(landed, str(landed))}


def gimbal_attitude(payload: bytes) -> dict:
    """Where the gimbal says it points, and what its yaw is measured from.

    A gimbal reports yaw from the nose or from north, and the flags say which.
    Stock PX4 sends it from north, and patches/px4-gzgimbal-frame.patch makes
    this simulator send it from the nose, so the frame has to travel with the
    angle. See modules/sim/px4-rcS.
    """
    values = unpack("<I4f3fIHBB", payload)
    pitch, yaw = pitch_yaw_deg(*values[1:5])
    flags = values[9]
    if flags & YAW_IN_VEHICLE_FRAME:
        frame = "the nose"
    elif flags & YAW_IN_EARTH_FRAME:
        frame = "north"
    else:
        frame = ""
    return {"gimbal_pitch": pitch, "gimbal_yaw": yaw, "gimbal_yaw_from": frame,
            "gimbal_failures": values[8]}


READERS = {
    HEARTBEAT: heartbeat,
    SYS_STATUS: sys_status,
    GPS_RAW_INT: gps_raw_int,
    GLOBAL_POSITION_INT: global_position_int,
    EXTENDED_SYS_STATE: extended_sys_state,
    GIMBAL_DEVICE_ATTITUDE_STATUS: gimbal_attitude,
}


def fields(frame: Frame) -> dict:
    """What one frame says about the vehicle. A message nothing reads says nothing."""
    reader = READERS.get(frame.message)
    if reader is None:
        return {}
    try:
        return reader(frame.payload)
    except struct.error:
        return {}
