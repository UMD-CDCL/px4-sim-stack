#!/usr/bin/env python3
"""Fly one simulated vehicle and read what its nodes are doing.

Everything here goes through the interfaces the aircraft uses: MAVROS services
for flight, and the 5g_drone topics for the gimbal. Nothing reaches into the
simulator, so a check that passes here is a check of the flight code.

Runs inside that vehicle's companion container, on its ROS domain:
    ./px4sim uas 11 takeoff 40
    ./px4sim uas 11 gimbal -90
    ./px4sim uas 11 status
"""

import argparse
import json
import math
import os
import sys
import time

import yaml

import numpy as np
import pymap3d as pm
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from cdcl_umd_msgs.msg import TargetBoxArray
from cdcl_umd_msgs.srv import TBALocalization
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition, State
from mavros_msgs.srv import CommandBool, CommandInt, CommandTOL, SetMode
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import NavSatFix
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float32

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

SERVICE_WAIT_S = 10.0
SETTLE_POLL_S = 0.2
# PX4 drops out of OFFBOARD when setpoints stop arriving, and refuses to enter
# it until a few have. 20 Hz is the rate the flight code streams at.
ARRIVED_M = 2.0
# Movement that counts as getting closer rather than as noise.
PROGRESS_M = 0.5
# MAV_CMD_DO_REPOSITION, with the frame whose altitude is metres over home.
# This is the "fly here" a ground station sends, and it needs no setpoint
# stream, which is what the vehicle's MAVROS is built to carry.
MAV_CMD_DO_REPOSITION = 192
# PX4 drops the frame out of COMMAND_INT and reads the altitude as metres
# above mean sea level whatever it says, so send that and name the frame to
# match rather than the other way round.
MAV_FRAME_GLOBAL_INT = 5
DEFAULT_GROUND_SPEED = -1.0
CHANGE_MODE = 1.0
DEGREES_TO_INT = 1e7
# gimbal.py reads this on the reassert topic to mean "take control back, and
# keep the pitch you already hold".
KEEP_PITCH = -361.0


class Uas(Node):
    """One vehicle, addressed the way the ground station addresses it."""

    def __init__(self, number: int) -> None:
        super().__init__("verify_uas")
        self.namespace = f"/uas{number}"
        self.state = None
        self.fix = None
        self.local = None
        self.gimbal = None
        self.home = None
        self.height = None

        self.create_subscription(State, f"{self.namespace}/state",
                                 self._on_state, RELIABLE_QOS)
        self.create_subscription(NavSatFix, f"{self.namespace}/global_position/global",
                                 self._on_fix, SENSOR_QOS)
        self.create_subscription(PoseStamped, f"{self.namespace}/local_position/pose",
                                 self._on_local, SENSOR_QOS)
        self.create_subscription(Altitude, f"{self.namespace}/altitude",
                                 self._on_altitude, SENSOR_QOS)
        self.create_subscription(HomePosition, f"{self.namespace}/home_position/home",
                                 self._on_home, LATCHED_QOS)
        self.create_subscription(
            GimbalDeviceAttitudeStatus,
            f"{self.namespace}/gimbal_control/device/attitude_status",
            self._on_gimbal, SENSOR_QOS)
        self.gimbal_pitch = self.create_publisher(
            Float32, f"{self.namespace}/gimbal_raw_command", RELIABLE_QOS)
        self.gimbal_reassert = self.create_publisher(
            Float32, f"{self.namespace}/reassert_gimbal_cmd", RELIABLE_QOS)
        self.reposition = self.create_client(CommandInt, f"{self.namespace}/cmd/command_int")
        # The detector's own services. They sit outside the vehicle namespace,
        # which is where ds_node advertises them.
        self.toggle_detection = self.create_client(SetBool, "/ds/mode/toggle_detection")
        # What the detector can be asked for, by the name an operator uses.
        self.captures = {
            name: self.create_client(Trigger, path) for name, path in (
                ("detect", "/ds/batch/run_detect"),
                ("mosaic", "/ds/capture/mosaic"),
                ("fiducial", "/ds/capture/fiducial"),
                ("vlm", "/ds/capture/vlm"),
                ("snapshot", "/ds/snapshot"))}
        self.localize = self.create_client(TBALocalization, f"{self.namespace}/tba_loczn")
        # What an operator's Foxglove panel publishes: a pixel in the preview
        # image, plus the two services that say whether clicks are live.
        self.click = self.create_publisher(
            PointStamped, f"{self.namespace}/camera/click", RELIABLE_QOS)
        self.click_point = self.create_client(
            Trigger, f"{self.namespace}/gimbal/click_mode/point")
        self.click_off = self.create_client(
            Trigger, f"{self.namespace}/gimbal/click_mode/off")

        self.arming = self.create_client(CommandBool, f"{self.namespace}/cmd/arming")
        self.takeoff_srv = self.create_client(CommandTOL, f"{self.namespace}/cmd/takeoff")
        self.land_srv = self.create_client(CommandTOL, f"{self.namespace}/cmd/land")
        self.set_mode = self.create_client(SetMode, f"{self.namespace}/set_mode")

    def _on_state(self, msg): self.state = msg
    def _on_fix(self, msg): self.fix = msg
    def _on_local(self, msg): self.local = msg
    def _on_gimbal(self, msg): self.gimbal = msg
    def _on_home(self, msg): self.home = msg
    def _on_altitude(self, msg): self.height = msg

    def amsl_for(self, height_over_home: float):
        """An altitude above mean sea level, from a height over home.

        Both numbers come off one message, so nothing here has to know the
        geoid: global_position/global reports an ellipsoid height, and the two
        differ by about thirty metres in Maryland.
        """
        if self.height is None:
            return None
        return self.height.amsl - self.height.relative + height_over_home

    def boresight_depression_deg(self):
        """How far below the horizon the camera looks, from the gimbal's own
        report.

        The camera looks along the x axis of the frame the report declares, and
        MAVLink states a gimbal attitude in a forward-right-DOWN frame, so a
        positive z component is already a look downwards.
        """
        if self.gimbal is None:
            return None
        q = self.gimbal.q
        forward = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([1.0, 0.0, 0.0])
        return float(np.degrees(np.arcsin(np.clip(forward[2], -1.0, 1.0))))

    def pump(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, ready, deadline_s: float, what: str, remaining=None) -> bool:
        """Poll for a condition and say which way it went. Every wait here says
        what it is waiting for, so a stack that never gets there names the step
        instead of stopping silently.

        `remaining` returns how far there is to go. While that keeps falling the
        deadline is pushed back, because the answer is coming: a busy simulator
        runs at a fraction of real time, and a wall clock deadline then gives up
        on a vehicle that is flying perfectly well, only slowly.
        """
        end = time.monotonic() + deadline_s
        closest = None
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=SETTLE_POLL_S)
            if ready():
                return True
            if remaining is not None:
                left = remaining()
                if closest is None or left < closest - PROGRESS_M:
                    closest = left
                    end = time.monotonic() + deadline_s
        print(f"gave up waiting for {what} after {deadline_s:.0f}s without progress",
              file=sys.stderr)
        return False

    def call(self, client, request, what: str):
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            print(f"no {what} service at {client.srv_name}", file=sys.stderr)
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_WAIT_S)
        return future.result()

    @property
    def altitude(self) -> float:
        return self.local.pose.position.z if self.local else float("nan")


def command_status(uas: Uas, _args) -> int:
    uas.wait_until(lambda: uas.state and uas.fix and uas.local, 15.0, "telemetry")
    if not uas.state:
        print("no telemetry: MAVROS is not publishing state")
        return 1
    print(f"mode        {uas.state.mode}")
    print(f"armed       {uas.state.armed}")
    print(f"connected   {uas.state.connected}")
    if uas.fix:
        print(f"position    {uas.fix.latitude:.7f}, {uas.fix.longitude:.7f}, "
              f"{uas.fix.altitude:.1f} m")
    print(f"height      {uas.altitude:.1f} m over home")
    if uas.local:
        q = uas.local.pose.orientation
        east, north = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([1.0, 0.0, 0.0])[:2]
        print(f"heading     {math.degrees(math.atan2(east, north)) % 360:.0f} degrees from north")
    depression = uas.boresight_depression_deg()
    if depression is not None:
        print(f"gimbal      {depression:.1f} degrees below the horizon, "
              f"flags {uas.gimbal.flags}")
    return 0


def command_arm(uas: Uas, _args) -> int:
    result = uas.call(uas.arming, CommandBool.Request(value=True), "arming")
    print("armed" if result and result.success else "arming refused")
    return 0 if result and result.success else 1


def command_takeoff(uas: Uas, args) -> int:
    uas.wait_until(lambda: uas.state is not None, 15.0, "MAVROS state")
    if uas.state and not uas.state.armed:
        uas.call(uas.arming, CommandBool.Request(value=True), "arming")
    # NAV_TAKEOFF takes an altitude above mean sea level and an operator means
    # a height over home.
    if not uas.wait_until(lambda: uas.height is not None, 15.0, "an altitude report"):
        return 1
    request = CommandTOL.Request(min_pitch=0.0, yaw=float("nan"),
                                 latitude=float("nan"), longitude=float("nan"),
                                 altitude=float(uas.amsl_for(args.height)))
    result = uas.call(uas.takeoff_srv, request, "takeoff")
    if not (result and result.success):
        print("takeoff refused")
        return 1
    reached = uas.wait_until(lambda: uas.altitude >= args.height * 0.9,
                             args.deadline, f"{args.height} m over home",
                             remaining=lambda: max(0.0, args.height - uas.altitude))
    print(f"height {uas.altitude:.1f} m")
    return 0 if reached else 1


def command_land(uas: Uas, args) -> int:
    result = uas.call(uas.land_srv, CommandTOL.Request(), "land")
    if not (result and result.success):
        print("land refused")
        return 1
    uas.wait_until(lambda: uas.altitude < 1.0, args.deadline, "the ground")
    print(f"height {uas.altitude:.1f} m")
    return 0


def command_gimbal(uas: Uas, args) -> int:
    """Point the gimbal by publishing on the topic the operator's controls use.

    The manager hands control to whoever asked last, so take it back before
    commanding: a vehicle that has been flown from a ground station or a mission
    otherwise accepts the angle and never moves.
    """
    uas.wait_until(lambda: uas.state is not None, 15.0, "MAVROS state")
    uas.gimbal_reassert.publish(Float32(data=KEEP_PITCH))
    uas.pump(1.0)
    for _ in range(args.repeat):
        uas.gimbal_pitch.publish(Float32(data=float(args.pitch)))
        uas.pump(0.25)
    print(f"commanded pitch {args.pitch} degrees")
    uas.pump(args.settle)
    reported = uas.boresight_depression_deg()
    if reported is None:
        print("the gimbal reports no attitude", file=sys.stderr)
        return 1
    print(f"reported depression {reported:.2f} degrees below the horizon")
    print(f"flags {uas.gimbal.flags}")
    return 0


def command_goto(uas: Uas, args) -> int:
    """Fly to a point measured from home, in metres east, north and up.

    Sent as MAV_CMD_DO_REPOSITION, which is the "fly here" a ground station
    sends. OFFBOARD would need a setpoint stream, and this vehicle's MAVROS
    carries no setpoint plugins, so a stream would reach nobody.
    """
    if not uas.wait_until(
            lambda: uas.home is not None and uas.local is not None and uas.height is not None,
            20.0, "a home position and an altitude report"):
        return 1
    latitude, longitude, _ = pm.enu2geodetic(
        args.east, args.north, 0.0,
        uas.home.geo.latitude, uas.home.geo.longitude, uas.home.geo.altitude, deg=True)

    request = CommandInt.Request(
        frame=MAV_FRAME_GLOBAL_INT,
        command=MAV_CMD_DO_REPOSITION,
        param1=DEFAULT_GROUND_SPEED,
        param2=CHANGE_MODE,
        # NaN keeps the heading it has. The gimbal's yaw is locked to the
        # airframe, so pointing the camera at something means pointing the
        # vehicle at it first.
        param4=math.radians(args.heading) if args.heading is not None else float("nan"),
        x=int(round(latitude * DEGREES_TO_INT)),
        y=int(round(longitude * DEGREES_TO_INT)),
        z=float(uas.amsl_for(args.up)))
    answer = uas.call(uas.reposition, request, "reposition")
    if not (answer and answer.success):
        print("reposition refused", file=sys.stderr)
        return 1

    def arrived():
        here = uas.local.pose.position
        return math.dist((here.x, here.y, here.z),
                         (args.east, args.north, args.up)) <= ARRIVED_M

    def distance_to_go():
        here = uas.local.pose.position
        return math.dist((here.x, here.y, here.z), (args.east, args.north, args.up))

    reached = uas.wait_until(arrived, args.deadline,
                             f"{args.east}, {args.north}, {args.up}",
                             remaining=distance_to_go)
    here = uas.local.pose.position
    print(f"at {here.x:.1f} east, {here.y:.1f} north, {here.z:.1f} up from home")
    return 0 if reached else 1


def command_detect(uas: Uas, args) -> int:
    """Start or stop continuous detection.

    The detector runs its pipeline from the first frame but detects nothing
    until this is on: a mission turns it on, and a bare stack leaves it off.
    """
    answer = uas.call(uas.toggle_detection, SetBool.Request(data=args.on == "on"),
                      "toggle_detection")
    if not answer:
        return 1
    print(answer.message or ("on" if args.on == "on" else "off"))
    return 0 if answer.success else 1


def command_capture(uas: Uas, args) -> int:
    """Ask the detector for one capture of the kind named."""
    answer = uas.call(uas.captures[args.what], Trigger.Request(), args.what)
    if not answer:
        return 1
    print(answer.message)
    return 0 if answer.success else 1


def command_detections(uas: Uas, args) -> int:
    """Report one detection message, and where its boxes localize.

    The detector publishes boxes and tf_loc localizes on request, which is what
    a mission does, so this asks the same way rather than waiting for a mission.
    """
    latest = {}
    uas.create_subscription(TargetBoxArray, f"{uas.namespace}/target_detections",
                            lambda msg: latest.setdefault("msg", msg), RELIABLE_QOS)
    if not uas.wait_until(lambda: "msg" in latest, args.deadline, "a detection"):
        return 1
    detection = latest["msg"]
    if not args.tsv:
        print(f"{len(detection.uav_target_boxes)} boxes in seq {detection.seq}, "
              f"image {len(detection.source_img.data)} bytes")
        for box in detection.uav_target_boxes:
            centre = box.target_bbox.center.position
            print(f"  {box.detection_class or '?'} {box.detection_confidence:.2f} "
                  f"at {centre.x:.0f}, {centre.y:.0f} "
                  f"({box.target_bbox.size_x:.0f} x {box.target_bbox.size_y:.0f} px)")
    if not detection.uav_target_boxes:
        return 1

    answer = uas.call(uas.localize, TBALocalization.Request(un_localized=detection),
                      "tba_loczn")
    if not answer:
        return 1
    truth = ground_truth(args.truth, args.scene)
    for box in answer.localized_boxes.uav_target_boxes:
        fix = box.target_location_altimeter_plane
        topo = box.target_location_topo
        surface = "terrain" if topo.status.status >= 0 else "flat plane"
        name, error = nearest(truth, fix.latitude, fix.longitude) if truth else ("", 0.0)
        if args.tsv:
            # One line per box, for a caller comparing two vehicles' answers.
            print(f"{name}\t{fix.latitude:.7f}\t{fix.longitude:.7f}"
                  f"\t{fix.altitude:.2f}\t{error:.2f}\t{surface}")
            continue
        line = (f"  localized {fix.latitude:.7f}, {fix.longitude:.7f}, "
                f"{fix.altitude:.1f} m on the {surface}")
        if truth:
            line += f"  -> {error:.1f} m from {name}"
        print(line)
    return 0 if answer.localized_boxes.uav_target_boxes else 1


def ground_truth(path: str, scene_surface: str):
    """Where the scenario really put its targets, as WGS84 positions.

    The simulator records the pose it achieved for every entity, in metres from
    the world origin, and the scene surface says where that origin is.
    """
    if not (path and os.path.exists(path) and os.path.exists(scene_surface)):
        return []
    origin = json.load(open(scene_surface))["origin_lla"]
    placed = yaml.safe_load(open(path)) or {}
    truth = []
    for entity in placed.get("entities", []):
        east, north, up = entity["pose"]
        latitude, longitude, _ = pm.enu2geodetic(east, north, up, *origin, deg=True)
        truth.append((entity["name"], latitude, longitude))
    return truth


def nearest(truth, latitude: float, longitude: float):
    """The closest recorded target, and how far away it is on the ground."""
    def metres(entry):
        east, north, _ = pm.geodetic2enu(latitude, longitude, 0.0,
                                         entry[1], entry[2], 0.0, deg=True)
        return math.hypot(east, north)
    closest = min(truth, key=metres)
    return closest[0], metres(closest)


def command_published(uas: Uas, args) -> int:
    """Dump the localizations this graph publishes, as one line per box.

    Run on the vehicle and on the ground station and the two lines for one
    sequence number must match character for character: the ground shows what
    the vehicle worked out, not its own approximation of it.
    """
    seen = {}
    truth = ground_truth(args.truth, args.scene)

    def collect(msg):
        for index, box in enumerate(msg.uav_target_boxes):
            fix = box.target_location_altimeter_plane
            if args.named:
                # Keyed by the target rather than the message, so a caller
                # comparing two vehicles has a name to join on.
                name, error = nearest(truth, fix.latitude, fix.longitude)
                seen[name] = (f"{name}\t{fix.latitude:.7f}\t{fix.longitude:.7f}"
                              f"\t{fix.altitude:.2f}\t{error:.2f}")
                continue
            seen[(msg.seq, index)] = (
                f"{msg.seq}\t{index}\t{fix.latitude:.7f}\t{fix.longitude:.7f}"
                f"\t{fix.altitude:.2f}\t{box.detection_class or '?'}")

    uas.create_subscription(TargetBoxArray, f"{uas.namespace}/{args.topic}",
                            collect, RELIABLE_QOS)
    uas.pump(args.seconds)
    for key in sorted(seen):
        print(seen[key])
    return 0 if seen else 1


def command_score(uas: Uas, args) -> int:
    """What this graph makes of the detections against the known targets.

    The vehicle scores what it localized and the ground station scores what
    reached it, so the same numbers on both sides mean the ground is not
    approximating the vehicle's answer.
    """
    values = {}
    uas.pump(3.0)  # discovery, or the type lookup below finds nothing
    known = dict(uas.get_topic_names_and_types())
    wanted = ("detection_precision", "detection_recall", "position_error")
    for topic in wanted:
        full = f"{uas.namespace}/{topic}"
        types = known.get(full)
        if not types:
            continue
        uas.create_subscription(
            get_message(types[0]), full,
            (lambda name: lambda msg: values.setdefault(name, msg.data))(topic),
            RELIABLE_QOS)
    uas.wait_until(lambda: len(values) >= len(wanted), args.deadline, "a score")
    if not values:
        print("nothing scored", file=sys.stderr)
        return 1
    for topic in wanted:
        if topic in values:
            print(f"{topic}\t{values[topic]:.4f}")
    return 0


def command_click(uas: Uas, args) -> int:
    """Click a pixel of the preview image, the way an operator does.

    Sets the click mode first, because a station whose clicks are off is the
    safe default and a click then changes nothing.
    """
    if args.mode == "off":
        answer = uas.call(uas.click_off, Trigger.Request(), "click_mode/off")
        print(answer.message if answer else "no click mode service")
        return 0 if answer and answer.success else 1

    if not args.keep_mode:
        answer = uas.call(uas.click_point, Trigger.Request(), "click_mode/point")
        if not (answer and answer.success):
            print("click mode refused", file=sys.stderr)
            return 1
    if not uas.wait_until(lambda: uas.gimbal is not None, 15.0,
                          "the gimbal's attitude"):
        return 1
    before = uas.boresight_depression_deg()
    point = PointStamped()
    point.header.frame_id = "preview"
    point.header.stamp = uas.get_clock().now().to_msg()
    point.point.x, point.point.y = float(args.u), float(args.v)
    for _ in range(args.repeat):
        uas.click.publish(point)
        uas.pump(0.25)
    uas.pump(args.settle)
    after = uas.boresight_depression_deg()
    print(f"clicked {args.u}, {args.v}: depression {before:.1f} -> {after:.1f} degrees")
    return 0


def command_fiducial(uas: Uas, args) -> int:
    """Localize one box as the fiducial marker, and read back the correction.

    A real fiducial capture goes to a human, who marks the marker in the image.
    This stands in for that: it takes a frame the detector has already produced,
    marks it as the fiducial, and asks for the same localization the human's
    answer would ask for. What comes back is the survey that moves the whole
    fleet's frame.
    """
    latest = {}
    uas.create_subscription(TargetBoxArray, f"{uas.namespace}/target_detections",
                            lambda msg: latest.setdefault("msg", msg), RELIABLE_QOS)
    correction = {}
    uas.create_subscription(TransformStamped, f"{uas.namespace}/fiducial_update",
                            lambda msg: correction.setdefault("msg", msg), RELIABLE_QOS)
    if not uas.wait_until(lambda: "msg" in latest, args.deadline, "a frame to mark"):
        return 1

    marked = latest["msg"]
    marked.fiducial_marker = True
    marked.uav_target_boxes = list(marked.uav_target_boxes)[:1]
    if not marked.uav_target_boxes:
        print("the frame carried no box to mark", file=sys.stderr)
        return 1
    uas.call(uas.localize, TBALocalization.Request(un_localized=marked), "tba_loczn")

    if not uas.wait_until(lambda: "msg" in correction, 10.0, "a fiducial correction"):
        return 1
    moved = correction["msg"].transform.translation
    print(f"{correction['msg'].header.frame_id} -> {correction['msg'].child_frame_id}"
          f"\t{moved.x:.2f}\t{moved.y:.2f}\t{moved.z:.2f}")
    return 0


COMMANDS = {
    "status": command_status,
    "arm": command_arm,
    "takeoff": command_takeoff,
    "land": command_land,
    "gimbal": command_gimbal,
    "goto": command_goto,
    "detect": command_detect,
    "capture": command_capture,
    "detections": command_detections,
    "published": command_published,
    "score": command_score,
    "click": command_click,
    "fiducial": command_fiducial,
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("number", type=int, help="the UAS number, 11 upwards")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status")
    sub.add_parser("arm")
    takeoff = sub.add_parser("takeoff")
    takeoff.add_argument("height", type=float, nargs="?", default=40.0,
                         help="metres over home, not above sea level")
    takeoff.add_argument("--deadline", type=float, default=90.0)
    land = sub.add_parser("land")
    land.add_argument("--deadline", type=float, default=120.0)
    gimbal = sub.add_parser("gimbal")
    gimbal.add_argument("pitch", type=float,
                        help="degrees, negative looks down, -90 is straight down")
    gimbal.add_argument("--repeat", type=int, default=4,
                        help="the node holds the newest command, so a few make "
                             "the first one land whatever the discovery timing")
    gimbal.add_argument("--settle", type=float, default=4.0)
    goto = sub.add_parser("goto")
    goto.add_argument("east", type=float)
    goto.add_argument("north", type=float)
    goto.add_argument("up", type=float, help="metres over home")
    goto.add_argument("--deadline", type=float, default=120.0)
    goto.add_argument("--heading", type=float, default=None,
                      help="degrees clockwise from north to face on arrival")
    detect = sub.add_parser("detect")
    detect.add_argument("on", choices=["on", "off"])
    capture = sub.add_parser("capture")
    capture.add_argument("what", nargs="?", default="detect",
                         choices=["detect", "mosaic", "fiducial", "vlm", "snapshot"])
    detections = sub.add_parser("detections")
    detections.add_argument("--deadline", type=float, default=20.0)
    detections.add_argument("--tsv", action="store_true",
                            help="one line per box: target, position, error")
    detections.add_argument("--truth", default=os.environ.get("RESOLVED_TRUTH_FILE", ""),
                            help="what the scenario actually placed")
    click = sub.add_parser("click")
    click.add_argument("mode", choices=["point", "off"])
    click.add_argument("u", type=float, nargs="?", default=320.0)
    click.add_argument("v", type=float, nargs="?", default=90.0)
    click.add_argument("--repeat", type=int, default=3)
    click.add_argument("--keep-mode", action="store_true",
                       help="click without setting the mode, to see whether a "
                            "station whose clicks are off really ignores them")
    click.add_argument("--settle", type=float, default=5.0)
    fiducial = sub.add_parser("fiducial")
    fiducial.add_argument("--deadline", type=float, default=20.0)
    score = sub.add_parser("score")
    score.add_argument("--deadline", type=float, default=20.0)
    published = sub.add_parser("published")
    published.add_argument("--topic", default="target_locations")
    published.add_argument("--named", action="store_true",
                           help="one line per target, named by the recorded "
                                "target it landed nearest")
    published.add_argument("--truth", default=os.environ.get("RESOLVED_TRUTH_FILE", ""))
    published.add_argument("--scene",
                           default=f"/scenes/worlds/{os.environ.get('SCENE', '')}_surface.json")
    published.add_argument("--seconds", type=float, default=12.0,
                           help="how long to collect. Run both sides at once "
                                "and join on the sequence number: the two are "
                                "watching one live stream, not one message")
    detections.add_argument("--scene", default=f"/scenes/worlds/{os.environ.get('SCENE', '')}_surface.json",
                            help="the surface that says where the world origin is")
    args = parser.parse_args()

    rclpy.init()
    uas = Uas(args.number)
    try:
        return COMMANDS[args.command](uas, args)
    finally:
        uas.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
