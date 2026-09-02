#!/usr/bin/env python3
"""Fly a mission on one simulated vehicle, and move the mission out from under the
node while it is working.

The mission node opens a "waypoint operation" on IMAGE_START_CAPTURE: ask
img_processing for a capture, survey the boxes it returns with the gimbal
pointing action, then advance the mission past the hold. That spans two long
waits, and the mission can move during either one -- an operator advances past
the hold, or the flight mode leaves AUTO.MISSION. The guards under test are what
stops a stale continuation from slewing the gimbal at a waypoint the aircraft
has already left.

Everything here goes through the interfaces an operator has: MAVROS for the
mission and the flight mode, and the 5g_drone topics for the operator's advance
button. Nothing reaches into the node, so a pass here is a pass of the flight
code.

Runs inside the vehicle's companion container, on its ROS domain.

    plan <path>            write a course.py-shaped plan for this scene
    scenario <a|b|c|d|e>   fly one scenario and report what the node did
    upload-service         push the configured plan through the node's own
                           upload_plan service, rather than from here
"""

import argparse
import json
import math
import re
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import CommandCode, MountControl, State, Waypoint, WaypointList, WaypointReached
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, WaypointPush, WaypointSetCurrent
from rcl_interfaces.msg import Log
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=50)
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
# mavros latches the waypoint list, so a late subscriber still gets the mission
TRANSIENT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST, depth=10)

# Below this the vehicle is on the ground. A landed vehicle stays armed in a mode
# it will not climb out of, so a mission started into that state never leaves the pad.
GROUNDED_M = 1.5
ALTITUDE_MODE_RELATIVE = 1
MISSION_MODE = "AUTO.MISSION"
HOLD_MODE = "AUTO.LOITER"
LAND_MODE = "AUTO.LAND"

# Plan geometry. course.py stands off from a casualty along a bearing and faces
# back down it; these are its defaults, and the shape of the plan it emits is
# what the mission node's index arithmetic is written against.
PLAN_ALTITUDE_M = 12.0
PLAN_STANDOFF_M = 10.0
# A hold has to outlast the capture and the survey that run inside it. The node
# advances as soon as the survey succeeds, so a generous hold costs no flight
# time -- but a short one lets PX4 advance first, which trips the op's own
# window guard during entirely normal progression. course.py defaults to 5 s.
PLAN_HOLD_S = 60.0
PLAN_PITCH_DEG = -50.0
FINAL_PITCH_DEG = -90.0
OBSERVATION_BEARINGS_DEG = (0.0, 180.0)
EARTH_RADIUS_M = 6371000.0

# The scene's casualties, as ENU metres from the scene origin. Read from the
# scenario file by the caller and passed in, so this file holds no scene data.


def offset_latlon(lat_deg, lon_deg, bearing_deg, distance_m):
    """A point `distance_m` from (lat, lon) along `bearing_deg`."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    bearing = math.radians(bearing_deg)
    angular = distance_m / EARTH_RADIUS_M
    out_lat = math.asin(math.sin(lat) * math.cos(angular)
                        + math.cos(lat) * math.sin(angular) * math.cos(bearing))
    out_lon = lon + math.atan2(
        math.sin(bearing) * math.sin(angular) * math.cos(lat),
        math.cos(angular) - math.sin(lat) * math.sin(out_lat))
    return math.degrees(out_lat), math.degrees(out_lon)


def enu_to_latlon(lat_deg, lon_deg, east_m, north_m):
    """Flat-earth ENU offset from an origin, which is exact enough over a scene."""
    d_lat = north_m / 111320.0
    d_lon = east_m / (111320.0 * math.cos(math.radians(lat_deg)))
    return lat_deg + d_lat, lon_deg + d_lon


# --- the plan ---------------------------------------------------------------------------------
# Item order is course.py's: one mount control up front, then
# (nav waypoint, image capture, holding waypoint) for each observation point,
# then a mount control to stow -- which is also what gives the last holding
# waypoint somewhere to advance to. A NAV_TAKEOFF leads, because a mission
# started from the ground has to climb out before it flies anywhere.


def mount_control_item(pitch):
    return {
        "autoContinue": True,
        "command": int(CommandCode.DO_MOUNT_CONTROL),
        "doJumpId": 0,
        "frame": int(Waypoint.FRAME_MISSION),
        "params": [pitch, 0, 0, 0, 0, 0, MountControl.MAV_MOUNT_MODE_MAVLINK_TARGETING],
        "type": "SimpleItem",
    }


def waypoint_item(lat, lon, yaw, hold, altitude):
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": altitude,
        "AltitudeMode": ALTITUDE_MODE_RELATIVE,
        "autoContinue": True,
        "command": int(CommandCode.NAV_WAYPOINT),
        "doJumpId": 0,
        "frame": int(Waypoint.FRAME_GLOBAL_REL_ALT),
        "params": [hold, 0, 0, yaw, lat, lon, altitude],
        "type": "SimpleItem",
    }


def image_capture_item():
    # param3 is the image count the node reads; param4 picks the routine, but
    # the VLM branch is commented out in the node, so every capture takes the
    # standard path that opens an operation.
    return {
        "autoContinue": True,
        "command": int(CommandCode.IMAGE_START_CAPTURE),
        "doJumpId": 0,
        "frame": int(Waypoint.FRAME_MISSION),
        "params": [0, 0, 1, 0.0, None, None, None],
        "type": "SimpleItem",
    }


def takeoff_item(lat, lon, altitude):
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": altitude,
        "AltitudeMode": ALTITUDE_MODE_RELATIVE,
        "autoContinue": True,
        "command": int(CommandCode.NAV_TAKEOFF),
        "doJumpId": 0,
        "frame": int(Waypoint.FRAME_GLOBAL_REL_ALT),
        "params": [0, 0, 0, float("nan"), lat, lon, altitude],
        "type": "SimpleItem",
    }


def build_plan(home_lat, home_lon, casualties, altitude=PLAN_ALTITUDE_M,
               standoff=PLAN_STANDOFF_M, hold=PLAN_HOLD_S):
    """A course.py-shaped plan over the scene's casualties."""
    items = [takeoff_item(home_lat, home_lon, altitude),
             mount_control_item(PLAN_PITCH_DEG)]

    for east, north in casualties:
        lat, lon = enu_to_latlon(home_lat, home_lon, east, north)
        for bearing in OBSERVATION_BEARINGS_DEG:
            point_lat, point_lon = offset_latlon(lat, lon, bearing, standoff)
            yaw = (bearing + 180.0) % 360.0
            items.append(waypoint_item(point_lat, point_lon, yaw, 0.0, altitude))
            items.append(image_capture_item())
            items.append(waypoint_item(point_lat, point_lon, yaw, hold, altitude))

    items.append(mount_control_item(FINAL_PITCH_DEG))

    for index, item in enumerate(items):
        item["doJumpId"] = index + 1

    return {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": 15,
            "firmwareType": 12,
            "globalPlanAltitudeMode": 0,
            "hoverSpeed": 5,
            "items": items,
            "plannedHomePosition": [home_lat, home_lon, 0.0],
            "vehicleType": 2,
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }


def plan_to_waypoints(plan):
    """The same mapping the node's own loader makes: params[0:4] are param1..4,
    params[4:7] are lat/lon/alt, an unset param is NaN and an unset coordinate 0."""
    waypoints = []
    for item in plan["mission"]["items"]:
        params = item.get("params", [])

        def param(index, unset):
            if index >= len(params) or params[index] is None:
                return unset
            return float(params[index])

        wp = Waypoint()
        wp.frame = int(item.get("frame", Waypoint.FRAME_MISSION))
        wp.command = int(item["command"])
        wp.is_current = False
        wp.autocontinue = bool(item.get("autoContinue", True))
        wp.param1 = param(0, math.nan)
        wp.param2 = param(1, math.nan)
        wp.param3 = param(2, math.nan)
        wp.param4 = param(3, math.nan)
        wp.x_lat = param(4, 0.0)
        wp.y_long = param(5, 0.0)
        wp.z_alt = param(6, 0.0)
        waypoints.append(wp)
    return waypoints


# --- the harness ------------------------------------------------------------------------------

COMMAND_NAMES = {
    16: "NAV_WAYPOINT", 22: "NAV_TAKEOFF", 20: "NAV_RETURN_TO_LAUNCH",
    205: "DO_MOUNT_CONTROL", 2000: "IMAGE_START_CAPTURE", 2001: "IMAGE_STOP_CAPTURE",
    2500: "VIDEO_START_CAPTURE", 2501: "VIDEO_STOP_CAPTURE",
}


class Harness(Node):
    def __init__(self, number):
        super().__init__("mission_harness")
        self.namespace_ = f"/uas{number}"

        self.state = None
        self.local = None
        self.gimbal_pitch = None
        self.wps = None
        self.reached = []
        self.logs = []
        self.lock = threading.Lock()

        self.create_subscription(Log, "/rosout", self._on_log, 200)
        self.create_subscription(State, f"{self.namespace_}/state", self._on_state, RELIABLE_QOS)
        self.create_subscription(PoseStamped, f"{self.namespace_}/local_position/pose",
                                 self._on_local, SENSOR_QOS)
        self.create_subscription(WaypointList, f"{self.namespace_}/mission/waypoints",
                                 self._on_wps, TRANSIENT_QOS)
        self.create_subscription(WaypointReached, f"{self.namespace_}/mission/reached",
                                 self._on_reached, RELIABLE_QOS)

        self.push = self.create_client(WaypointPush, f"{self.namespace_}/mission/push")
        self.set_current = self.create_client(WaypointSetCurrent,
                                              f"{self.namespace_}/mission/set_current")
        self.arming = self.create_client(CommandBool, f"{self.namespace_}/cmd/arming")
        self.set_mode = self.create_client(SetMode, f"{self.namespace_}/set_mode")
        self.upload_plan = self.create_client(Trigger, f"{self.namespace_}/upload_plan")

        self.advance_pub = self.create_publisher(Bool, f"{self.namespace_}/advance_mission_cmd",
                                                 RELIABLE_QOS)
        # Claiming gimbal control is a publish, like the ground station does it.
        self.reassert_pub = self.create_publisher(Float32,
                                                  f"{self.namespace_}/reassert_gimbal_cmd",
                                                  RELIABLE_QOS)

    # --- inputs
    def _on_log(self, msg):
        with self.lock:
            self.logs.append((time.time(), msg.level, msg.name, msg.msg))

    def _on_state(self, msg):
        self.state = msg

    def _on_local(self, msg):
        self.local = msg

    @property
    def height(self):
        return self.local.pose.position.z if self.local else float("nan")

    def _on_wps(self, msg):
        self.wps = msg

    def _on_reached(self, msg):
        self.reached.append((time.time(), int(msg.wp_seq)))

    # --- log helpers
    def since(self):
        with self.lock:
            return len(self.logs)

    def lines(self, mark=0, pattern=None, node=None):
        with self.lock:
            entries = list(self.logs[mark:])
        out = []
        for stamp, level, name, text in entries:
            if node is not None and node not in name:
                continue
            if pattern is not None and not re.search(pattern, text):
                continue
            out.append((stamp, level, name, text))
        return out

    def index_of(self, pattern, mark=0):
        """Absolute log index of the first line matching `pattern` at or after `mark`."""
        with self.lock:
            entries = list(self.logs[mark:])
        for offset, (_stamp, _level, _name, text) in enumerate(entries):
            if re.search(pattern, text):
                return mark + offset
        return None

    def wait_log(self, pattern, timeout, mark=0, node=None):
        """Block until a rosout line matches, and return it. None on timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            hits = self.lines(mark, pattern, node)
            if hits:
                return hits[0]
            time.sleep(0.1)
        return None

    def wait_until(self, predicate, timeout, what):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if predicate():
                    return True
            except (TypeError, AttributeError, IndexError):
                pass
            time.sleep(0.1)
        print(f"  timed out after {timeout:.0f}s waiting for {what}", file=sys.stderr)
        return False

    # --- outputs
    def call(self, client, request, label, timeout=15.0):
        if not client.wait_for_service(timeout_sec=timeout):
            print(f"  {label}: service {client.srv_name} never appeared", file=sys.stderr)
            return None
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            print(f"  {label}: no reply in {timeout:.0f}s", file=sys.stderr)
            return None
        return future.result()

    def push_plan(self, plan):
        waypoints = plan_to_waypoints(plan)
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = waypoints
        answer = self.call(self.push, request, "mission push", timeout=30.0)
        ok = bool(answer and answer.success)
        print(f"  push: {'ok' if ok else 'FAILED'} "
              f"({getattr(answer, 'wp_transfered', 0)}/{len(waypoints)} items)")
        return ok

    def upload_via_service(self):
        """Push the plan the way the node offers it: its own upload_plan service.

        This is the only upload path that resets the node's item bookkeeping, so
        it is what the scenarios use. A plan pushed straight to the FCU -- which is
        what a ground station does -- leaves that bookkeeping where the last
        mission left it.
        """
        # Retried: `px4sim place` respawns the companion, and the node advertises this
        # service before mavros is serving mission/push. The first upload after a respawn
        # is then accepted and never answered.
        for attempt in range(3):
            answer = self.call(self.upload_plan, Trigger.Request(), "upload_plan", 60.0)
            ok = bool(answer and answer.success)
            print(f"  upload_plan: {'ok' if ok else 'FAILED'} "
                  f"({getattr(answer, 'message', 'no reply')})")
            if ok:
                return True
            if attempt < 2:
                print("  waiting for mavros to finish coming up, then retrying")
                time.sleep(15.0)
        return False

    def load_mission(self, plan, how):
        if how == "service":
            return self.upload_via_service()
        return self.push_plan(plan)

    def jump_to(self, seq):
        answer = self.call(self.set_current, WaypointSetCurrent.Request(wp_seq=int(seq)),
                           f"set_current {seq}")
        ok = bool(answer and answer.success)
        print(f"  set_current -> {seq}: {'accepted' if ok else 'REFUSED'}")
        return ok

    def mode(self, name):
        answer = self.call(self.set_mode, SetMode.Request(base_mode=0, custom_mode=name),
                           f"set_mode {name}")
        ok = bool(answer and answer.mode_sent)
        print(f"  set_mode {name}: {'sent' if ok else 'REFUSED'}")
        return ok

    def arm(self):
        answer = self.call(self.arming, CommandBool.Request(value=True), "arming")
        return bool(answer and answer.success)

    def start_mission(self):
        """Arm and put the vehicle in AUTO.MISSION, from wherever it is."""
        if not self.wait_until(lambda: self.state is not None, 30.0, "a MAVROS state"):
            return False
        if not self.wait_until(lambda: self.wps is not None and len(self.wps.waypoints) > 0,
                               20.0, "a waypoint list"):
            return False

        self.wait_until(lambda: self.local is not None, 15.0, "a local position")
        for attempt in range(4):
            # A vehicle left over from an earlier scenario is still armed and holding a
            # mode it will not climb out of, so the mission is accepted and it never
            # leaves the pad. Disarmed on the ground is the one state every mission
            # starts from. PX4 refuses a disarm until its land detector has fired
            # ("Disarming denied: not landed") -- which it has not while the vehicle
            # hovers a few centimetres up -- so land it rather than asking twice.
            if self.state.armed:
                print(f"  armed at {self.height:.1f} m; landing before the mission starts")
                self.mode(LAND_MODE)
                if not self.wait_until(lambda: not self.state.armed, 60.0,
                                       "the landing to disarm the vehicle"):
                    self.call(self.arming, CommandBool.Request(value=False), "disarming")
                    self.wait_until(lambda: not self.state.armed, 15.0, "a disarmed vehicle")
            # Rewind only once the vehicle is down. A mission still running through the
            # landing carries current_seq on with it, so rewinding first and arming
            # afterwards starts the flight from the middle of the plan -- and the
            # waypoint the scenario is waiting for has already gone by.
            self.jump_to(0)
            if not self.state.armed and not self.arm():
                time.sleep(2.0)
                continue
            self.wait_until(lambda: self.state.armed, 10.0, "an armed vehicle")
            self.mode(MISSION_MODE)
            if self.wait_until(lambda: self.state.mode == MISSION_MODE, 10.0, MISSION_MODE):
                print(f"  flying: armed={self.state.armed} mode={self.state.mode}")
                if self.gimbal_pitch is not None:
                    self.point_gimbal(self.gimbal_pitch)
                return True
            print(f"  attempt {attempt + 1}: mode is {self.state.mode}, retrying")
        return False

    def point_gimbal(self, pitch):
        """Claim gimbal control and point it, which PX4 does not do here.

        A mission's DO_MOUNT_CONTROL is remembered by the mission node but never
        reaches the device in this stack, so the camera flies level and the
        detector returns nothing -- which would leave every capture with no boxes
        and the survey half of the operation untested. A reassert carrying a pitch
        claims control and points in one message, and the gimbal node clears the
        held pitch immediately after using it, so this does not fight the survey
        that reasserts later.
        """
        self.reassert_pub.publish(Float32(data=float(pitch)))
        print(f"  gimbal: claimed control, pitch {pitch:.0f}")

    def advance_button(self):
        """The operator's advance button, the same topic the ground station uses."""
        self.advance_pub.publish(Bool(data=True))
        print("  advance_mission_cmd: published")

    def mission_summary(self):
        if self.wps is None:
            return "no mission"
        out = []
        for index, wp in enumerate(self.wps.waypoints):
            name = COMMAND_NAMES.get(int(wp.command), str(int(wp.command)))
            hold = f" hold={wp.param1:.0f}" if int(wp.command) == 16 else ""
            out.append(f"    {index:3d}  {name}{hold}")
        return "\n".join(out)


def print_lines(harness, mark, pattern=None, label="rosout"):
    hits = harness.lines(mark, pattern)
    if not hits:
        print(f"  ({label}: nothing matched)")
        return hits
    print(f"  {label}:")
    for _stamp, level, name, text in hits:
        tag = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}.get(level, level)
        short = name.split(".")[-1]
        for line in text.splitlines():
            print(f"    [{tag:5s}] {short}: {line}")
    return hits


# --- scenarios --------------------------------------------------------------------------------

# What a cancelled operation has to say for itself. The node logs these in order,
# and each one is a different guard: the window watcher noticing, the capture wait
# giving up, and the continuation refusing to act on what it got back.
CANCEL_MARKS = (
    (r"op \d+ \(item \d+\) window expired \(.*\); cancelling", "window watcher cancelled the op"),
    # One mark, five spellings, because where the cancel lands decides which of them the
    # node writes -- and the scenario cannot steer that. The capture may still be out
    # (the wait abandons the reply, or the abort beats the detector home and the capture
    # returns a failure); it may have just landed (the continuation discards it); or it
    # may have got as far as the survey, which then bails at the reassert gate or at the
    # goal gate. All five are the operation being stopped with nothing reaching the
    # gimbal, and demanding one particular spelling fails runs where the interlock worked
    # perfectly -- and worse, the missing mark then blocks for the whole --cancel-timeout,
    # so everything the mission legitimately does meanwhile lands in the forbidden window.
    (r"op \d+: abandoning image capture reply"
     r"|image capture request failed: aborted"
     r"|op \d+ \(item \d+\) cancelled \(.*\); discarding capture, no survey, no advance"
     r"|op \d+ cancelled during survey reassert; not dispatching"
     r"|op \d+ cancelled \(.*\); not dispatching survey goal"
     r"|survey action result failed: Goal cancel",
     "the operation was stopped"),
)
# What must NOT appear after a cancel: the gimbal being handed a survey, or the
# mission being pushed forward by an operation that no longer owns it.
# `survey: yolo request` is logged on ENTERING the survey routine, two gates before
# anything is sent, so a cancel landing during the reassert wait produces that line and
# then `not dispatching` -- correct behaviour that reads as a violation. The line that
# actually means a goal is going to the gimbal is `survey localized:`, written
# immediately before send_goal_async with no gate left between them.
FORBIDDEN_AFTER_CANCEL = (
    (r"survey localized: \d+ points", "a survey goal was dispatched"),
    (r"survey succeeded -> advancing mission", "the mission was advanced"),
    (r"mission current set -> ", "the mission was advanced"),
)

# A survey goal leaving the node, for spotting one dispatched in the gap between the
# operator moving the mission and mavros telling the node about it. Same reasoning:
# the dispatch, not the intention to dispatch.
SURVEY_DISPATCHED = r"survey localized: \d+ points"

# ...and the ways that survey is allowed to end: the goal cancelled in flight, or
# never dispatched at all because a gate caught the op first.
SURVEY_CANCELLED = (r"survey action result failed: Goal cancel"
                    r"|not dispatching survey goal"
                    r"|cancelled during survey reassert")

# How long the node gets, after it learns the mission moved, to put down a survey
# it dispatched before it knew. The goal is cancelled mid-settle, and _ROI_SETTLE_SEC
# in survey.py is 4 s, so this is that plus room for the round trip.
CANCEL_GRACE_S = 10.0


def lines_between(harness, start, end, pattern):
    """Lines matching `pattern` whose position in the log falls in [start, end).

    By position, not by text. The harness's older checks locate a line by searching the
    log for its own text, which maps every repeat of an identical line onto the first
    one. That errs safe -- a violation inside a window is itself the first match there,
    so none can be hidden -- but it is not exact, and this judgement now carries three
    scenarios and lines like `survey localized: 1 points` that repeat verbatim.
    """
    found = []
    for offset, entry in enumerate(harness.lines(start)):
        if start + offset >= end:
            break
        if re.search(pattern, entry[3]):
            found.append(entry)
    return found


def judge_cancel_window(harness, notes, command_mark, noticed, label):
    """Judge what the node did around a cancel, and answer whether it was allowed.

    Three scenarios interrupt an operation -- B moves the mission, C changes the flight
    mode, H sends the mission backwards -- and all three run into the same fact: the node
    learns what the operator did only when mavros tells it, about a second later. So there
    are two windows, not one.

    Before `noticed`, the node is acting on the last thing it was told. A survey goal
    dispatched here is not a fault; leaving it running once the node knows would be, so it
    is bounded rather than forbidden. After `noticed`, nothing may be dispatched at all,
    up to the point the interrupt's own re-run opens the next operation -- past which a
    survey and an advance are the mission working, which these scenarios separately assert.

    Judging from `command_mark` instead of `noticed` asks the node to act on what it has
    not been told, and fails runs where it behaved perfectly. That is what it was doing.
    """
    passed = True

    next_op = harness.index_of(r"op \d+ \(item \d+\) started", noticed)
    window_end = next_op if next_op is not None else harness.since()
    offending = 0
    for pattern, description in FORBIDDEN_AFTER_CANCEL:
        offenders = lines_between(harness, noticed, window_end, pattern)
        if offenders:
            notes.append(f"FORBIDDEN: {description} after the node noticed, before the "
                         f"next operation ({len(offenders)} lines)")
            passed = False
            offending += len(offenders)
    if offending == 0:
        notes.append(f"nothing dispatched and no advance once the node noticed {label}")

    # Now the window the one above deliberately excludes, so that excluding it does not
    # amount to ignoring it.
    blind = lines_between(harness, command_mark, noticed, SURVEY_DISPATCHED)
    if blind:
        told = harness.lines(command_mark, r"window expired \(.*\); cancelling"
                                           r"|mission moved back \d+ -> \d+")
        # `told` cannot be empty when `blind` is not -- an empty one collapses `noticed`
        # onto `command_mark`, which leaves nothing before it -- but the timings below are
        # decoration on a verdict, and decoration must not be able to raise.
        noticed_at = told[0][0] if told else blind[0][0]
        notes.append(f"a survey goal went out {noticed_at - blind[0][0]:.2f}s before the "
                     f"node noticed {label}")
        put_down = harness.wait_log(SURVEY_CANCELLED, CANCEL_GRACE_S, noticed)
        if put_down:
            notes.append(f"bounded: it was put down {put_down[0] - noticed_at:.2f}s after "
                         f"-- {put_down[3]}")
        else:
            notes.append(f"FORBIDDEN: a survey goal dispatched before the node noticed was "
                         f"still running {CANCEL_GRACE_S:.0f}s later")
            passed = False
    else:
        notes.append("no survey goal went out before the node noticed; nothing to bound")

    return passed


def capture_items(harness):
    """Every IMAGE_START_CAPTURE index in the loaded mission."""
    return [i for i, wp in enumerate(harness.wps.waypoints)
            if int(wp.command) == int(CommandCode.IMAGE_START_CAPTURE)]


def parse_op_start(text):
    """(op_id, item_idx, hold_idx) out of an op start line. Every op is guarded now."""
    started = re.search(r"op (\d+) \(item (\d+)\) started: cancel if current_seq leaves (\d+)", text)
    if started:
        return int(started.group(1)), int(started.group(2)), int(started.group(3))
    return None


def wait_for_op(harness, timeout, mark=0, guarded=None, min_item=0):
    """Wait for an operation to open, and say which one it is."""
    deadline = time.time() + timeout
    seen = mark
    while time.time() < deadline:
        for stamp, _level, _name, text in harness.lines(seen, r"op \d+ \(item \d+\) started"):
            parsed = parse_op_start(text)
            if parsed is None:
                continue
            if guarded is True and parsed[2] is None:
                continue
            if parsed[1] < min_item:
                continue
            return parsed
        seen = harness.since()
        time.sleep(0.2)
    return None


def report(name, passed, notes):
    print()
    print(f"  {'PASS' if passed else 'FAIL'}  scenario {name}")
    for note in notes:
        print(f"        {note}")
    return 0 if passed else 1


def scenario_a(harness, plan, args):
    """Normal mission, end to end. The baseline every other scenario is read against."""
    print("\n== A: normal mission, end to end ==")
    mark = harness.since()
    if not harness.load_mission(plan, args.push):
        return report("A", False, ["the mission would not push"])
    if not harness.start_mission():
        return report("A", False, ["the vehicle would not start the mission"])

    captures = capture_items(harness)
    print(f"  mission has {len(harness.wps.waypoints)} items, "
          f"{len(captures)} captures at {captures}")

    # let it fly. A capture is ~25 s and a hold 5 s, so give every cycle its time.
    budget = args.budget or (len(captures) * 70 + 120)
    print(f"  flying for up to {budget}s")
    deadline = time.time() + budget
    last_seq = -1
    while time.time() < deadline:
        if harness.wps is not None and int(harness.wps.current_seq) != last_seq:
            last_seq = int(harness.wps.current_seq)
            print(f"    current_seq -> {last_seq}")
        if harness.wps is not None and last_seq >= len(harness.wps.waypoints) - 1:
            time.sleep(10.0)
            break
        time.sleep(1.0)

    print()
    fired = harness.lines(mark, r"image capture \(op \d+\)")
    surveys = harness.lines(mark, r"survey: yolo request")
    advances = harness.lines(mark, r"mission current set -> ")
    cancels = harness.lines(mark, r"window expired")
    jumps = harness.lines(mark, r"verdict=JUMP")
    backward = harness.lines(mark, r"verdict=BACKWARD")

    print_lines(harness, mark, r"progress:", "progression")
    print_lines(harness, mark, r"op \d+ ", "operations")

    notes = [
        f"captures requested: {len(fired)} of {len(captures)} planned",
        f"surveys requested:  {len(surveys)}",
        f"surveys completed:  {len(harness.lines(mark, r'survey action complete'))}",
        f"mission advances:   {len(advances)}",
        f"cancellations:      {len(cancels)} (expected 0)",
        f"false JUMP:         {len(jumps)} (expected 0)",
        f"false BACKWARD:     {len(backward)} (expected 0)",
    ]
    passed = (len(fired) == len(captures) and len(surveys) > 0
              and len(cancels) == 0 and len(jumps) == 0)
    return report("A", passed, notes)


def _interrupt_scenario(harness, plan, args, name, headline, interrupt, recover=None):
    """B and C are the same test with a different way of moving the mission."""
    print(f"\n== {name}: {headline} ==")
    mark = harness.since()
    if not harness.load_mission(plan, args.push):
        return report(name, False, ["the mission would not push"])
    if not harness.start_mission():
        return report(name, False, ["the vehicle would not start the mission"])

    # Move the mission while the capture is genuinely outstanding. A capture that finds
    # nothing comes back in about a second, and the operation then advances on its own --
    # leaving nothing to cancel and making the guards look absent when they were simply
    # never needed. So trigger off the capture request line rather than a fixed delay, and
    # if the operation finishes first anyway, take the next one instead of failing.
    op = None
    seek_mark = mark
    for attempt in range(3):
        op = wait_for_op(harness, args.op_timeout, seek_mark, guarded=True)
        if op is None:
            return report(name, False, ["no guarded operation opened before the timeout"])
        op_id, item_idx, hold = op
        print(f"  op {op_id} on item {item_idx}, held by {hold}")
        if harness.wait_log(rf"image capture \(op {op_id}\)", 30.0, seek_mark) is None:
            return report(name, False, [f"op {op_id} never requested its capture"])
        print(f"  capture is in flight; interrupting after {args.into_capture:.1f}s")
        time.sleep(args.into_capture)

        interrupt_mark = harness.since()
        interrupt(harness, hold)
        settled = harness.wait_log(rf"op {op_id} .*window expired", 12.0, interrupt_mark)
        if settled is not None:
            break
        finished = harness.lines(interrupt_mark, r"no targets to survey|survey succeeded")
        if not finished:
            break
        print(f"  op {op_id} completed before the interrupt landed; trying the next one")
        # Leaving AUTO.MISSION ends the mission, so an attempt that lost the race also
        # stopped the flight. Put it back before looking for another operation.
        if recover is not None:
            recover(harness)
        seek_mark = harness.since()

    print("  watching for the cancel chain")
    notes = []
    passed = True
    for pattern, description in CANCEL_MARKS:
        hit = harness.wait_log(pattern, args.cancel_timeout, interrupt_mark)
        if hit:
            notes.append(f"saw: {description}")
        else:
            notes.append(f"MISSING: {description}")
            passed = False

    # The abort has to reach the image node, or it sits on frames for a waypoint the
    # aircraft has left and busy-rejects the next one. "no capture in flight" is a pass,
    # not a miss: it means the capture had already come home before the cancel, so there
    # was nothing left to abort and the survey gates are what stopped the operation.
    abort = harness.wait_log(r"capture abort -> |no capture in flight", 10.0, interrupt_mark)
    notes.append(f"abort reply: {abort[3] if abort else 'MISSING'}")
    if abort is None:
        passed = False

    noticed = harness.index_of(
        r"op \d+ \(item \d+\) window expired \(.*\); cancelling", interrupt_mark)
    if noticed is None:
        noticed = interrupt_mark
    if not judge_cancel_window(harness, notes, interrupt_mark, noticed,
                               "the mission had moved"):
        passed = False

    # and the waypoint after it has to be servable, not busy-rejected
    print("  waiting for the next operation to prove the image node is free")
    following = wait_for_op(harness, args.next_op_timeout, interrupt_mark)
    if following:
        notes.append(f"next op {following[0]} opened on item {following[1]}")
        rejected = harness.wait_log(r"busy|reject", 20.0, interrupt_mark)
        if rejected and "abort" not in rejected[3].lower():
            notes.append(f"but a capture was refused: {rejected[3]}")
            passed = False
        else:
            notes.append("the next capture was not busy-rejected")
    else:
        notes.append("no following operation opened (mission may have ended)")

    print()
    print_lines(harness, interrupt_mark, r"op \d+|abort|survey|mission current", "after the interrupt")
    return report(name, passed, notes)


def scenario_b(harness, plan, args):
    def interrupt(h, bounding):  # noqa: the hold index is what bounds the op now
        # past the hold this op was armed against: what an operator does when
        # they advance the mission while a capture is still running
        h.jump_to(bounding + 1)
    return _interrupt_scenario(harness, plan, args, "B",
                               "skip a waypoint mid-capture", interrupt)


def scenario_c(harness, plan, args):
    def interrupt(h, _hold):
        h.mode(HOLD_MODE)

    def recover(h):
        h.mode(MISSION_MODE)
        h.wait_until(lambda: h.state is not None and h.state.mode == MISSION_MODE,
                     15.0, "the mission to resume")

    return _interrupt_scenario(harness, plan, args, "C",
                               "leave AUTO.MISSION mid-capture", interrupt, recover=recover)


def scenario_d(harness, plan, args):
    """The last capture in the plan is guarded like any other.

    It used to be exempt so the mode change at the end of a mission could not cancel its
    survey. That exemption is gone: it left an op nothing could cancel for the whole of
    every lap of a looping mission, and cost the final survey is the price."""
    print("\n== D: the last capture is guarded like any other ==")
    mark = harness.since()
    if not harness.load_mission(plan, args.push):
        return report("D", False, ["the mission would not push"])
    if not harness.start_mission():
        return report("D", False, ["the vehicle would not start the mission"])

    captures = capture_items(harness)
    tail = captures[-1]
    print(f"  flying the mission through to the last capture, item {tail}")

    op = wait_for_op(harness, args.op_timeout, mark, min_item=tail)
    if op is None:
        every = harness.lines(mark, r"op \d+ \(item \d+\) started")
        return report("D", False,
                      [f"no operation opened on the last capture (item {tail})",
                       f"op starts seen: {[line[3] for line in every]}"])
    op_id, item_idx, hold = op
    tail_mark = harness.since()
    print(f"  op {op_id} on item {item_idx}, held by {hold} -- not exempt")

    # Push the mission past that hold. This is the case the tail exemption used to make
    # unreachable: the last capture in a plan had no bounding waypoint, so nothing could
    # cancel it. Now it is guarded like any other.
    harness.jump_to(min(hold + 1, len(harness.wps.waypoints) - 1))

    cancelled = harness.wait_log(rf"op {op_id} .*window expired", args.cancel_timeout, tail_mark)
    print()
    print_lines(harness, tail_mark, r"op \d+|survey|image capture|moved back", "the last operation")

    notes = [f"op {op_id} opened guarded by hold {hold}, with no tail exemption",
             f"cancelled when the mission passed it: {'yes' if cancelled else 'NO'}"]
    if cancelled:
        notes.append(f"cancelled: {cancelled[3]}")
    passed = hold is not None and bool(cancelled)
    return report("D", passed, notes)


def scenario_e(harness, plan, args):
    """The classifier, against jumps no flight bag contains. Observe-only: nothing
    here changes what the node processes, so this reports rather than asserts --
    except for a false JUMP on normal progression, which is the blocker."""
    print("\n== E: what the progression classifier makes of real jumps ==")
    mark = harness.since()
    if not harness.load_mission(plan, args.push):
        return report("E", False, ["the mission would not push"])
    if not harness.start_mission():
        return report("E", False, ["the vehicle would not start the mission"])

    total = len(harness.wps.waypoints)
    captures = capture_items(harness)
    print(f"  mission has {total} items, captures at {captures}")

    # 1. normal progression, to see what the classifier says when nothing is wrong
    print(f"\n  -- normal progression for {args.normal_watch:.0f}s --")
    normal_mark = harness.since()
    time.sleep(args.normal_watch)
    normal = print_lines(harness, normal_mark, r"progress:", "normal progression")
    false_jump = [line for line in normal if "verdict=JUMP" in line[3]]
    false_unpaired = [line for line in normal if "pairing=UNPAIRED" in line[3]]

    # 2. a distant jump, the case no bag contains
    far = min(captures[-1], total - 2)
    print(f"\n  -- a distant jump to item {far} --")
    far_mark = harness.since()
    harness.jump_to(far)
    time.sleep(args.jump_watch)
    far_lines = print_lines(harness, far_mark, r"progress:", "after the distant jump")

    # 3. a small jump, one plan cycle: the only jump current_seq alone can see
    current = int(harness.wps.current_seq)
    small = min(current + 2, total - 1)
    print(f"\n  -- a small jump, {current} -> {small} --")
    small_mark = harness.since()
    harness.jump_to(small)
    time.sleep(args.jump_watch)
    small_lines = print_lines(harness, small_mark, r"progress:", "after the small jump")

    # 4. backward, which is what looping a mission looks like from here
    back = max(captures[0] - 1, 1)
    print(f"\n  -- a backward jump to item {back} --")
    back_mark = harness.since()
    harness.jump_to(back)
    time.sleep(args.jump_watch)
    back_lines = print_lines(harness, back_mark, r"progress:", "after the backward jump")

    # did PX4 report an arrival anywhere near a commanded jump?
    reached_near_jumps = [
        (stamp, seq) for stamp, seq in harness.reached
        if stamp > harness.lines(far_mark)[0][0] if harness.lines(far_mark)
    ] if harness.lines(far_mark) else []

    notes = [
        f"normal progression: {len(normal)} events, "
        f"{len(false_jump)} false JUMP, {len(false_unpaired)} UNPAIRED",
        f"distant jump to {far}: "
        f"{'JUMP' if any('verdict=JUMP' in l[3] for l in far_lines) else 'not classified JUMP'}, "
        f"{'UNPAIRED' if any('UNPAIRED' in l[3] for l in far_lines) else 'no unpaired flag'}",
        f"small jump to {small}: "
        f"{'JUMP' if any('verdict=JUMP' in l[3] for l in small_lines) else 'not classified JUMP'}, "
        f"{'UNPAIRED' if any('UNPAIRED' in l[3] for l in small_lines) else 'no unpaired flag'}",
        f"backward jump to {back}: "
        f"{'BACKWARD' if any('verdict=BACKWARD' in l[3] for l in back_lines) else 'not classified BACKWARD'}",
        f"mission/reached messages during the run: {len(harness.reached)}",
    ]
    if false_jump:
        notes.append("BLOCKER: a JUMP was called on normal progression")
    # only the false positive fails this scenario; the rest is reporting
    return report("E", not false_jump, notes)


def scenario_f(harness, plan, args):
    """Loop the mission: fly part of it, then send the mission back to an item it
    has already passed, and see whether that item is run again.

    The node keeps `last_mission_item_idx` as the next unhandled item and never
    lets it go backwards, so this asks the question that matters for a looping
    plan -- not whether the classifier calls the transition BACKWARD, but whether
    a second lap does any work.
    """
    print("\n== F: does a mission that loops run its items again? ==")
    mark = harness.since()
    if not harness.load_mission(plan, args.push):
        return report("F", False, ["the mission would not push"])
    if not harness.start_mission():
        return report("F", False, ["the vehicle would not start the mission"])

    captures = capture_items(harness)
    first, second = captures[0], captures[1]

    # fly the first two capture cycles, so there is a lap to repeat
    print(f"  flying the first two capture cycles (items {first} and {second})")
    if not harness.wait_until(
            lambda: any(f"mission item {second}:" in line[3]
                        for line in harness.lines(mark, r"mission item \d+:")),
            args.op_timeout, f"item {second} to be processed"):
        return report("F", False, ["the mission never reached the second capture"])
    time.sleep(args.jump_watch)

    lap_one = harness.lines(mark, r"mission item \d+:")
    ops_one = harness.lines(mark, r"op \d+ \(item \d+\) started")
    print(f"  first lap processed {len(lap_one)} items and opened {len(ops_one)} operations")

    # now loop: send the mission back to the first capture's waypoint
    loop_target = first - 1
    print(f"\n  -- looping: sending the mission back to item {loop_target} --")
    lap_mark = harness.since()
    harness.jump_to(loop_target)
    print(f"  watching for {args.tail_watch:.0f}s to see whether the lap repeats")
    time.sleep(args.tail_watch)

    lap_two = harness.lines(lap_mark, r"mission item \d+:")
    ops_two = harness.lines(lap_mark, r"op \d+ \(item \d+\) started")
    captures_two = harness.lines(lap_mark, r"image capture \(op \d+\)")
    backward = harness.lines(lap_mark, r"verdict=BACKWARD")

    print()
    print_lines(harness, lap_mark, r"progress:|mission item \d+:|op \d+ \(item", "the second lap")

    notes = [
        f"second lap processed {len(lap_two)} mission items (first lap: {len(lap_one)})",
        f"second lap opened {len(ops_two)} operations (first lap: {len(ops_one)})",
        f"second lap requested {len(captures_two)} captures",
        f"BACKWARD verdicts: {len(backward)}",
    ]
    if not lap_two:
        notes.append("the loop ran NO mission items: last_mission_item_idx never "
                     "goes backwards, so every item on the second lap is suppressed")
    # This scenario reports; it does not assert, because the node does not claim
    # to support looping yet. It fails only if it cannot be told either way.
    return report("F", bool(lap_one), notes)


def scenario_g(harness, plan, args):
    """Skip several items at once, and see what the node does with the ones jumped over.

    The catch-up loop still walks every index, so no item is silently dropped and the
    cursor stays true. But a capture whose hold the mission has already passed is
    declined before it asks anything of img_processing -- the operation would have been
    cancelled on the next mission update anyway, after spending a detection pass on a
    vantage nobody is standing at."""
    print("\n== G: what happens to the items a jump skips over? ==")
    mark = harness.since()
    if not harness.load_mission(plan, args.push):
        return report("G", False, ["the mission would not push"])
    if not harness.start_mission():
        return report("G", False, ["the vehicle would not start the mission"])

    captures = capture_items(harness)
    # let the first cycle happen, so the jump starts from a known place
    print(f"  waiting for the first capture (item {captures[0]})")
    if not harness.wait_until(
            lambda: harness.lines(mark, rf"mission item {captures[0]}:"),
            args.op_timeout, "the first capture"):
        return report("G", False, ["the mission never reached the first capture"])
    time.sleep(args.jump_watch)

    far = captures[-1]
    jump_mark = harness.since()
    current = int(harness.wps.current_seq)
    print(f"\n  -- skipping {current} -> {far}, over "
          f"{len([c for c in captures if current < c < far])} captures --")
    harness.jump_to(far)
    print(f"  watching for {args.tail_watch:.0f}s")
    time.sleep(args.tail_watch)

    processed = harness.lines(jump_mark, r"mission item \d+:")
    jumps = harness.lines(jump_mark, r"verdict=JUMP")
    skipped = [c for c in captures if current < c < far]
    declined = harness.lines(jump_mark, r"not capturing, the mission is at")
    # an operation opened on a skipped capture means a request went out for a vantage
    # the aircraft had already left
    opened_on_skipped = [line for line in harness.lines(jump_mark, r"op \d+ \(item (\d+)\) started")
                         if any(f"(item {c})" in line[3] for c in skipped)]

    print()
    print_lines(harness, jump_mark,
                r"progress:|mission item \d+:|op \d+ \(item|not capturing", "after the skip")

    notes = [
        f"captures the jump skipped: {skipped}",
        f"items still processed after the skip: {len(processed)} (the cursor keeps moving)",
        f"captures declined as already behind the mission: {len(declined)}",
        f"operations opened on a skipped capture: {len(opened_on_skipped)} (expected 0)",
        f"JUMP verdicts: {len(jumps)}",
    ]
    if opened_on_skipped:
        notes.append("a capture was requested for a vantage the aircraft had left")
    passed = bool(processed) and not opened_on_skipped and len(declined) >= 1
    return report("G", passed, notes)


def scenario_h(harness, plan, args):
    """Send the mission back while an operation is in flight.

    This is the case the mission node could not previously express: a rewind puts
    current_seq *below* the op's hold, where no forward test can see it, so the op used
    to sit there and survey a vantage the aircraft was leaving. It is also what a loop
    wrap looks like from here -- MAVLink reports neither as anything but a lower index.
    """
    print("\n== H: the mission goes back while an operation is open ==")
    mark = harness.since()
    if not harness.load_mission(plan, args.push):
        return report("H", False, ["the mission would not push"])
    if not harness.start_mission():
        return report("H", False, ["the vehicle would not start the mission"])

    captures = capture_items(harness)
    # Wait for a later cycle, so that going back to the first vantage is real travel. A
    # rewind onto the waypoint the aircraft is already holding at is re-reached instantly,
    # and current_seq dips and recovers between two mission/waypoints updates -- the node
    # never sees it, and neither would an operator watching the same topic.
    settled = captures[min(2, len(captures) - 1)]
    print(f"  waiting for an operation at item {settled} or later")
    op = wait_for_op(harness, args.op_timeout, mark, min_item=settled)
    if op is None:
        return report("H", False, ["no operation opened before the timeout"])
    op_id, item_idx, hold = op
    print(f"  op {op_id} on item {item_idx}, held by {hold}")

    if harness.wait_log(rf"image capture \(op {op_id}\)", 30.0, mark) is None:
        return report("H", False, [f"op {op_id} never requested its capture"])
    print(f"  capture is in flight; sending the mission back after {args.into_capture:.1f}s")
    time.sleep(args.into_capture)

    # back to the first vantage: earlier than this op's item, which is what a DO_JUMP
    # wrapping the survey loop would produce
    target = max(captures[0] - 1, 0)
    rewind_mark = harness.since()
    rewind_time = time.time()
    harness.jump_to(target)

    notes = []
    passed = True

    rewound = harness.wait_log(r"mission moved back \d+ -> \d+", 20.0, rewind_mark)
    if rewound:
        notes.append(f"saw: {rewound[3]}")
        # Always reported, because it is the number that decides whether the bounded
        # check below has anything to look at: the node is blind for this long after the
        # operator moves the mission, and whether a survey happens to be dispatched
        # inside that window is a race the scenario cannot steer. Saying the latency out
        # loud stops "nothing to bound" from reading as "nothing was checked".
        notes.append(f"the node was blind for {rewound[0] - rewind_time:.2f}s after "
                     f"the cursor moved (mavros reporting latency)")
    else:
        notes.append("MISSING: the node did not report the mission moving back")
        passed = False

    cancelled = harness.wait_log(rf"op {op_id} .*went back to item", 20.0, rewind_mark)
    if cancelled:
        notes.append(f"saw: {cancelled[3]}")
    else:
        notes.append(f"MISSING: op {op_id} was not cancelled by the rewind")
        passed = False

    # The window opens where the NODE learns the mission moved, not where this harness sent
    # the command. Between the two sits mavros: a moved cursor is reported about a second
    # after an operator moves it, and a survey dispatched in that second is dispatched by a
    # node with no reason not to. Forbidding it asks the node to act on what it has not been
    # told, which it cannot do and which is not what the guards are for. That second is not
    # ignored, though -- it is bounded below.
    notified = harness.index_of(r"mission moved back \d+ -> \d+", rewind_mark)
    if notified is None:
        # already reported missing above; nothing sensible to measure the window from
        notified = rewind_mark
    if not judge_cancel_window(harness, notes, rewind_mark, notified,
                               "the mission had moved back"):
        passed = False

    # and the items it went back to have to run again
    print(f"  watching for the repeated items ({args.tail_watch:.0f}s)")
    time.sleep(args.tail_watch)
    replayed = harness.lines(rewind_mark, r"mission item \d+:")
    reopened = harness.lines(rewind_mark, r"op \d+ \(item \d+\) started")
    notes.append(f"items processed after the rewind: {len(replayed)}")
    notes.append(f"operations opened after the rewind: {len(reopened)}")
    if not replayed:
        notes.append("the rewind ran no items: the cursor did not follow the mission back")
        passed = False

    print()
    print_lines(harness, rewind_mark,
                r"progress:|mission item \d+:|op \d+|moved back|ignoring reached", "after the rewind")
    return report("H", passed, notes)


def scenario_i(harness, plan, args):
    """The ground station route: a plan pushed straight to the FCU, then the operator
    setting the current item, with the vehicle on the ground.

    A mavros push leaves current_seq where it was and never touches the node, so the
    only thing that can move the cursor is the operator's own jump -- which happens
    while disarmed. This is the case the rewind gate used to reject, leaving the cursor
    on the previous flight's high-water mark and the whole next mission unprocessed.
    """
    print("\n== I: a ground-station push, then the operator sets the current item ==")
    mark = harness.since()

    # First flight, through the node's own upload, to leave the cursor well advanced.
    if not harness.upload_via_service():
        return report("I", False, ["the first mission would not upload"])
    if not harness.start_mission():
        return report("I", False, ["the vehicle would not start the first mission"])

    captures = capture_items(harness)
    print(f"  flying the first mission until item {captures[1]} is behind us")
    if not harness.wait_until(
            lambda: harness.lines(mark, rf"mission item {captures[1]}:"),
            args.op_timeout, "the first mission to advance"):
        return report("I", False, ["the first mission never advanced"])
    first_flight = harness.lines(mark, r"mission item \d+:")
    print(f"  first flight processed {len(first_flight)} items")

    # Now the ground-station route: push straight to the FCU, touching nothing else.
    print("\n  -- pushing the same plan to mavros, as a ground station does --")
    push_mark = harness.since()
    if not harness.push_plan(plan):
        return report("I", False, ["the mavros push failed"])
    reset = harness.lines(push_mark, r"uploaded \d+/\d+ mission items")
    if reset:
        return report("I", False,
                      ["the mavros push went through the node's upload_plan; "
                       "this scenario needs the plain push"])

    # The operator sets the current item with the vehicle out of AUTO.MISSION, which is
    # what the relaxed gate is about. Done by leaving mission mode rather than landing:
    # a land and re-arm drags in a separate simulator problem and proves nothing here.
    print("  -- operator leaves mission mode, sets the current item, resumes --")
    second_mark = harness.since()
    harness.mode(HOLD_MODE)
    harness.wait_until(lambda: harness.state is not None and harness.state.mode == HOLD_MODE,
                       15.0, "the vehicle to leave mission mode")
    harness.jump_to(0)
    moved = harness.lines(second_mark, r"mission moved back \d+ -> \d+")
    harness.mode(MISSION_MODE)
    harness.wait_until(lambda: harness.state is not None and harness.state.mode == MISSION_MODE,
                       15.0, "the mission to resume")
    print(f"  watching the second flight ({args.tail_watch:.0f}s)")
    time.sleep(args.tail_watch)

    second_flight = harness.lines(second_mark, r"mission item \d+:")
    second_ops = harness.lines(second_mark, r"op \d+ \(item \d+\) started")
    second_captures = harness.lines(second_mark, r"image capture \(op \d+\)")

    print()
    print_lines(harness, second_mark,
                r"moved back|mission item \d+:|op \d+ \(item|progress:", "the second flight")

    notes = [
        f"cursor followed the operator's jump: {'yes' if moved else 'NO'}",
        f"second flight processed {len(second_flight)} items (first: {len(first_flight)})",
        f"second flight opened {len(second_ops)} operations, "
        f"requested {len(second_captures)} captures",
    ]
    if moved:
        notes.append(f"saw: {moved[0][3]}")
    passed = bool(moved) and bool(second_flight) and bool(second_captures)
    if not passed:
        notes.append("a ground-station push followed by an operator jump left the mission "
                     "unprocessed")
    return report("I", passed, notes)


def scenario_j(harness, plan, args):
    """Ignore zones in flight: what the filter removes, and what it must not stall.

    The filter is late by design. It takes nothing out of a TargetBoxArray -- only out
    of the two acts that leave the aircraft, pointing the gimbal at a target and
    reporting one -- so it is invisible to a unit test of the pipeline and only shows up
    in a flight. And the interesting case is not what it drops but what happens when it
    drops everything: a survey with no points left never dispatches, so it never produces
    the action result that normally advances the mission. If nothing else advanced it,
    the vehicle would sit out the whole of a hold at every waypoint, and a mission that
    finds only ignorable targets would never finish.

    `--zones all` is that case, over a zone covering the whole course. `--zones partial`
    covers some targets and leaves others, so the filter has to be selective rather than
    simply off or simply fatal.

    The zone file is a node parameter read at start-up, so it is set outside this harness
    -- see ONBOARD_PARAMS_FILE and logs/onboard/zone_params_*.yaml.
    """
    mode = args.zones or "partial"
    print(f"\n== J: ignore zones in flight ({mode}) ==")
    mark = harness.since()
    if not harness.load_mission(plan, args.push):
        return report("J", False, ["the mission would not push"])
    if not harness.start_mission():
        return report("J", False, ["the vehicle would not start the mission"])

    captures = capture_items(harness)
    last_item = len(harness.wps.waypoints) - 1
    print(f"  mission has {len(harness.wps.waypoints)} items, "
          f"{len(captures)} captures at {captures}")

    budget = args.budget or (len(captures) * 70 + 120)
    print(f"  flying for up to {budget}s")
    deadline = time.time() + budget
    last_seq = -1
    finished = False
    while time.time() < deadline:
        if harness.wps is not None and int(harness.wps.current_seq) != last_seq:
            last_seq = int(harness.wps.current_seq)
            print(f"    current_seq -> {last_seq}")
        if harness.wps is not None and last_seq >= last_item:
            finished = True
            time.sleep(10.0)
            break
        time.sleep(1.0)

    print()
    fired = harness.lines(mark, r"image capture \(op \d+\)")
    advances = harness.lines(mark, r"mission current set -> ")
    dispatched = harness.lines(mark, r"survey localized: \d+ points")
    skipped_points = harness.lines(mark, r"not pointing at a target .*inside ignore zone")
    all_ignored = harness.lines(mark, r"all \d+ targets are inside ignore zones")
    skip_advance = harness.lines(mark, r"nothing to survey -> advancing mission")
    dropped_obs = harness.lines(mark, r"not reporting detection .*inside ignore zone")
    frame_ignored = harness.lines(mark, r"all \d+ detections are inside ignore zones")
    no_targets = harness.lines(mark, r"no targets to survey")

    notes = [
        f"captures requested:            {len(fired)} of {len(captures)} planned",
        f"survey points dropped by zone: {len(skipped_points)}",
        f"surveys emptied by zone:       {len(all_ignored)}",
        f"survey goals still dispatched: {len(dispatched)}",
        f"observations dropped by zone:  {len(dropped_obs)}",
        f"frames fully ignored:          {len(frame_ignored)}",
        f"mission advances:              {len(advances)}",
        f"reached the last item:         {finished}",
    ]
    passed = True

    # The zones have to be doing something, or the run says nothing at all.
    if not (skipped_points or all_ignored or dropped_obs or frame_ignored):
        notes.append("NO ZONE ACTIVITY: is search.ignore_zones_file set on the vehicle?")
        return report("J", False, notes)

    # Whatever the zones swallow, the mission still has to end. This is the assertion
    # the whole scenario exists for: the advance that normally rides on a successful
    # survey has to come from somewhere else when there is no survey to succeed.
    if not finished:
        notes.append("STALLED: the mission did not reach its last item")
        passed = False

    if mode == "all":
        if dispatched:
            notes.append(f"FORBIDDEN: {len(dispatched)} survey goals went out with every "
                         f"target inside a zone")
            passed = False
        if not all_ignored:
            notes.append("MISSING: no capture reported all its targets inside a zone "
                         "(did the detector find anything at all?)")
            passed = False
        # every emptied survey must hand the mission on rather than leave it holding
        if len(skip_advance) < len(all_ignored):
            notes.append(f"MISSING: {len(all_ignored)} surveys were emptied but only "
                         f"{len(skip_advance)} advanced the mission")
            passed = False
        if not frame_ignored:
            notes.append("MISSING: img_processing never suppressed a whole frame")
            passed = False
    else:
        if not skipped_points:
            notes.append("MISSING: no survey point was dropped for being inside a zone")
            passed = False
        if not dispatched:
            notes.append("MISSING: every survey was emptied; the zone is not selective "
                         "(expected some targets outside it)")
            passed = False

    # A capture that simply found nothing is not the filter working, and the two look
    # alike in the advance count. Say how many there were so a reader can tell them apart.
    notes.append(f"(captures that found nothing at all: {len(no_targets)})")

    print_lines(harness, mark, r"ignore zone|inside ignore zones|nothing to survey|"
                               r"survey localized|mission current set", "the filter at work")
    return report("J", passed, notes)


SCENARIOS = {"a": scenario_a, "b": scenario_b, "c": scenario_c, "d": scenario_d, "e": scenario_e,
             "f": scenario_f, "g": scenario_g, "h": scenario_h,
             "i": scenario_i, "j": scenario_j}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["plan", "scenario", "upload-service", "show"])
    parser.add_argument("argument", nargs="?", default="")
    parser.add_argument("--uas", type=int, default=11)
    parser.add_argument("--plan", default="/logs/mission_test.plan")
    parser.add_argument("--home", default="")
    parser.add_argument("--casualties", default="",
                        help="ENU metres from the origin: 'e,n;e,n;...'")
    parser.add_argument("--budget", type=int, default=0)
    parser.add_argument("--hold", type=float, default=PLAN_HOLD_S)
    parser.add_argument("--into-capture", type=float, default=0.3)
    parser.add_argument("--op-timeout", type=float, default=240.0)
    parser.add_argument("--cancel-timeout", type=float, default=45.0)
    parser.add_argument("--next-op-timeout", type=float, default=180.0)
    parser.add_argument("--tail-watch", type=float, default=90.0)
    parser.add_argument("--normal-watch", type=float, default=150.0)
    parser.add_argument("--jump-watch", type=float, default=25.0)
    parser.add_argument("--push", choices=["service", "mavros"], default="service",
                        help="how the mission reaches the FCU: the node's own "
                             "upload_plan service, or straight to mavros as a "
                             "ground station does it")
    parser.add_argument("--zones", choices=["partial", "all"], default="partial",
                        help="scenario J: what the configured ignore zones are expected "
                             "to cover -- some of the targets, or all of them")
    parser.add_argument("--gimbal-pitch", type=float, default=PLAN_PITCH_DEG,
                        help="pitch to claim the gimbal at once the mission is flying; "
                             "PX4 does not drive the mount from DO_MOUNT_CONTROL here")
    args = parser.parse_args()

    if args.command == "plan":
        latitude, longitude = (float(v) for v in args.home.split(","))
        casualties = [tuple(float(v) for v in pair.split(","))
                      for pair in args.casualties.split(";") if pair]
        plan = build_plan(latitude, longitude, casualties, hold=args.hold)
        path = args.argument or args.plan
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(plan, handle, indent=4)
        items = plan["mission"]["items"]
        print(f"wrote {len(items)} items to {path}")
        for index, item in enumerate(items):
            name = COMMAND_NAMES.get(item["command"], str(item["command"]))
            hold = ""
            if item["command"] == 16:
                hold = f" hold={item['params'][0]:.0f}"
            print(f"  {index:3d}  {name}{hold}")
        return 0

    with open(args.plan, "r", encoding="utf-8") as handle:
        plan = json.load(handle)

    rclpy.init()
    harness = Harness(args.uas)
    harness.gimbal_pitch = args.gimbal_pitch
    executor = MultiThreadedExecutor()
    executor.add_node(harness)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        if args.command == "show":
            harness.wait_until(lambda: harness.wps is not None, 20.0, "a waypoint list")
            print(harness.mission_summary())
            return 0
        if args.command == "upload-service":
            answer = harness.call(harness.upload_plan, Trigger.Request(), "upload_plan", 60.0)
            if answer is None:
                print("  upload_plan: no reply")
                return 1
            print(f"  upload_plan: success={answer.success} message={answer.message}")
            return 0 if answer.success else 1

        name = (args.argument or "a").lower()
        if name not in SCENARIOS:
            print(f"unknown scenario {name}", file=sys.stderr)
            return 2
        return SCENARIOS[name](harness, plan, args)
    finally:
        # Join the spin thread before tearing anything down. shutdown() only signals it,
        # so without the join the process can reach exit while a worker is still inside
        # the rmw wait, and the C++ side aborts with "terminate called without an active
        # exception" -- after the scenario has already printed PASS. That turns a passing
        # run into exit 134, which any batch or CI gate reads as a failure. Seen on
        # scenario C and on H; intermittent, because it depends on where the workers are
        # when the last assertion finishes.
        executor.shutdown()
        thread.join(timeout=10.0)
        if thread.is_alive():
            print("  warning: the executor thread did not stop within 10s",
                  file=sys.stderr)
        harness.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
