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
import array
import json
import math
import os
import statistics
import struct
import sys
import time
from collections import deque, namedtuple

import yaml

import numpy as np
import pymap3d as pm
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import (PointStamped, PoseStamped, TransformStamped,
                               TwistStamped)
from cdcl_umd_msgs.msg import KnownCasualtyLocations, TargetBox, TargetBoxArray
from cdcl_umd_msgs.srv import TBALocalization
from mavros_msgs.msg import (Altitude, GimbalDeviceAttitudeStatus,
                            GimbalManagerSetPitchyaw, HomePosition, State)
from mavros_msgs.srv import CommandBool, CommandInt, CommandTOL, SetMode
from std_srvs.srv import Trigger
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import CameraInfo, NavSatFix
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float32, Float64
from foxglove_msgs.msg import SceneUpdate
from cdcl_umd_msgs.msg import CameraFOV
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)
# How far the drawn ground may be from square with the scene, and how near an
# edge of the image its own edge has to fall.
SCENE_SPAN_TOLERANCE_M = 5.0
# How far the drawn heading of the airframe may be from the one it flies, and
# how far the footprint may lie from where the camera points. The footprint is
# a quadrilateral on uneven ground, so its middle is not exactly down the
# camera's own axis.
HEADING_TOLERANCE_DEG = 10.0
FOOTPRINT_TOLERANCE_DEG = 20.0
FOOTPRINT_MIN_DEPRESSION_DEG = 15.0
METRES_PER_DEGREE = 111320.0
TEXTURE_EDGE = 0.05
SCENE_HEIGHT_TOLERANCE_M = 5.0
# Colours closer together than this came from a default, not from an image.
FLAT_COLOUR = 0.02

LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

SERVICE_WAIT_S = 10.0
SETTLE_POLL_S = 0.2
# PX4 drops out of OFFBOARD when setpoints stop arriving, and refuses to enter
# it until a few have. 20 Hz is the rate the flight code streams at.
ARRIVED_M = 2.0
# Below this the vehicle is on the ground, whatever it believes.
GROUNDED_M = 1.5
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

# How long a recorded frame waits for the scorer to judge it. The scorer judges
# a frame as it arrives, so this only covers the hop between the two nodes.
VERDICT_WAIT_S = 1.0
ROW_FLUSH_S = 0.2
# How many judged frames to hold. The scorer republishes its judgement at 2 Hz
# until a new frame arrives, so a reader that keeps only the newest stamp still
# finds the frame it is looking for; this holds a few seconds of them anyway,
# because frames are localized concurrently and can arrive out of order.
JUDGED_DEPTH = 256
# Over what interval the camera pose is differentiated to get the slew rate.
# Long enough to average the steps in the pose stream, short enough that the
# answer is the rate at this frame rather than the mean of a whole sweep.
SLEW_WINDOW_S = 0.2
# How many samples of a telemetry topic to hold, for a reader that wants the
# one nearest a frame's own stamp rather than whichever arrived last.
SAMPLE_DEPTH = 200
# What a scorer's verdict cites, best first. A `hit:` is a box inside a
# casualty's gate, a `crossed:` is a box whose ray passed through one and
# landed elsewhere, and `nearest:` is the closest casualty of all.
CITATIONS = ("hit:", "crossed:", "nearest:")


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
        # Both axes at once. The elevation-only topic above holds the azimuth
        # the gimbal already has, which is what USPI sends and what leaves a
        # yawed mount where the last click put it.
        self.gimbal_angle = self.create_publisher(
            GimbalManagerSetPitchyaw, f"{self.namespace}/gimbal_angle_cmd", RELIABLE_QOS)
        self.gimbal_reassert = self.create_publisher(
            Float32, f"{self.namespace}/reassert_gimbal_cmd", RELIABLE_QOS)
        self.reposition = self.create_client(CommandInt, f"{self.namespace}/cmd/command_int")
        # Continuous detection, as one parameter on img_processing. That node turns
        # the detector on itself and publishes what comes back, so the parameter is
        # the whole switch. The detector's own /ds/mode/toggle_detection is half of
        # it: it detects, and img_processing drops every detection.
        self.continuous_detection = self.create_client(
            SetParameters, f"{self.namespace}/img_processing/set_parameters")
        # What the detector can be asked for, by the name an operator uses. These
        # sit outside the vehicle namespace, which is where ds_node advertises them.
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
        self.click_roi = self.create_client(
            Trigger, f"{self.namespace}/gimbal/click_mode/roi")

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

    def _boresight(self):
        """Where the camera looks, in the frame the gimbal's report declares."""
        if self.gimbal is None:
            return None
        q = self.gimbal.q
        return Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([1.0, 0.0, 0.0])

    def boresight_depression_deg(self):
        """How far below the horizon the camera looks, from the gimbal's own
        report.

        The camera looks along the x axis of the frame the report declares, and
        MAVLink states a gimbal attitude in a forward-right-DOWN frame, so a
        positive z component is already a look downwards.
        """
        forward = self._boresight()
        if forward is None:
            return None
        return float(np.degrees(np.arcsin(np.clip(forward[2], -1.0, 1.0))))

    def boresight_azimuth_deg(self):
        """How far right of the nose the camera looks, from the same report.

        The mount yaws, so the other half of where it points is this one.
        Positive is right, in the same forward-right-down frame. That frame is
        the vehicle frame while this station commands the gimbal, so the angle
        is off the nose; a region of interest set by somebody else carries
        YAW_LOCK and makes it an angle from north instead.
        """
        forward = self._boresight()
        if forward is None:
            return None
        return float(np.degrees(np.arctan2(forward[1], forward[0])))

    def pump(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def topic(self, name: str) -> str:
        """A name in this vehicle's namespace, unless it is already absolute."""
        return name if name.startswith("/") else f"{self.namespace}/{name}"

    def latest(self, message_type, topic: str, qos, deadline_s: float):
        """The newest message on a topic, or None. Every reader here wants the
        same thing: subscribe, wait, take what came."""
        arrived = {}
        self.create_subscription(message_type, topic,
                                 lambda msg: arrived.__setitem__("msg", msg), qos)
        self.wait_until(lambda: "msg" in arrived, deadline_s, topic)
        return arrived.get("msg")

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


def boresight_of(rotation) -> np.ndarray:
    """Where a frame in the tree looks, as a unit vector in east, north, up.

    Every frame the flight code points with looks along its own x axis: the
    gimbal, the camera under it and the nose of the airframe. tf_loc casts its
    rays from the camera frame the same way.
    """
    return Rotation.from_quat([rotation.x, rotation.y, rotation.z,
                               rotation.w]).apply([1.0, 0.0, 0.0])


def compass_of(rotation) -> float:
    """Which way a frame in the tree points, clockwise from north."""
    east, north, _ = boresight_of(rotation)
    return math.degrees(math.atan2(east, north)) % 360.0


def depression_of(rotation) -> float:
    """How far below the horizon a frame in the tree looks."""
    up = boresight_of(rotation)[2]
    return -math.degrees(math.asin(float(np.clip(up, -1.0, 1.0))))


def separation(one: float, other: float) -> float:
    """How far apart two bearings are, the short way round."""
    return abs((one - other + 180.0) % 360.0 - 180.0)


def stamp_key(stamp):
    """One frame's stamp, as something a dictionary can be keyed on."""
    return (stamp.sec, stamp.nanosec)


def seconds_of(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


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
    """Take off to a height over home, from whatever state the vehicle is in.

    A vehicle that has flown and landed is armed, on the ground and holding a
    mode it will not climb out of, and a takeoff sent into that is accepted and
    does nothing. So a vehicle already on the ground is disarmed first: that is
    the one state every takeoff works from, and it costs a second against a
    stage that otherwise measures a vehicle that never left the ground.
    """
    if not uas.wait_until(lambda: uas.state is not None and uas.height is not None,
                          20.0, "MAVROS state and an altitude report"):
        return 1

    for attempt in range(args.attempts):
        if uas.altitude < GROUNDED_M and uas.state.armed:
            uas.call(uas.arming, CommandBool.Request(value=False), "disarming")
            uas.wait_until(lambda: not uas.state.armed, 15.0, "a disarmed vehicle")
        if not uas.state.armed:
            answer = uas.call(uas.arming, CommandBool.Request(value=True), "arming")
            if not (answer and answer.success):
                print("arming refused", file=sys.stderr)
                return 1
            uas.wait_until(lambda: uas.state.armed, 15.0, "an armed vehicle")

        # NAV_TAKEOFF takes an altitude above mean sea level and an operator
        # means a height over home.
        request = CommandTOL.Request(min_pitch=0.0, yaw=float("nan"),
                                     latitude=float("nan"), longitude=float("nan"),
                                     altitude=float(uas.amsl_for(args.height)))
        answer = uas.call(uas.takeoff_srv, request, "takeoff")
        if not (answer and answer.success):
            print("takeoff refused", file=sys.stderr)
            return 1
        if uas.wait_until(lambda: uas.altitude >= args.height * 0.9,
                          args.deadline, f"{args.height} m over home",
                          remaining=lambda: max(0.0, args.height - uas.altitude)):
            print(f"height {uas.altitude:.1f} m")
            return 0
        print(f"still {uas.altitude:.1f} m after attempt {attempt + 1}", file=sys.stderr)

    print(f"height {uas.altitude:.1f} m")
    return 1


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
        if args.yaw is None:
            uas.gimbal_pitch.publish(Float32(data=float(args.pitch)))
        else:
            uas.gimbal_angle.publish(GimbalManagerSetPitchyaw(
                pitch=float(args.pitch), yaw=float(args.yaw)))
        uas.pump(0.25)
    print(f"commanded pitch {args.pitch} degrees"
          + ("" if args.yaw is None else f", yaw {args.yaw} degrees off the nose"))
    uas.pump(args.settle)
    reported = uas.boresight_depression_deg()
    if reported is None:
        print("the gimbal reports no attitude", file=sys.stderr)
        return 1
    print(f"reported depression {reported:.2f} degrees below the horizon")
    print(f"reported azimuth {uas.boresight_azimuth_deg():.2f} degrees right of the nose")
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
        # NaN keeps the heading it has. The gimbal yaws -180 to +180 degrees
        # off the nose, so a heading here is how a caller frames the shot, not
        # how the camera reaches the target.
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
    until this is on: a mission turns it on, and a bare stack leaves it off. A
    mission turns it on by setting this parameter, so this asks the same way,
    and what an operator sees here is what a mission produces.
    """
    on = args.on == "on"
    request = SetParameters.Request(parameters=[Parameter(
        name="continuous",
        value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=on))])
    answer = uas.call(uas.continuous_detection, request, "img_processing parameter")
    if not answer or not answer.results:
        print("img_processing said nothing about the continuous parameter", file=sys.stderr)
        return 1
    result = answer.results[0]
    print(result.reason or ("on" if on else "off"))
    return 0 if result.successful else 1


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
    # The newest frame, not the first to arrive. A reliable subscription hands
    # over whatever is queued, and localizing a frame from before the vehicle
    # settled measures a pose the camera has already left.
    latest = {}
    casualties = Casualties(uas)
    uas.create_subscription(TargetBoxArray, f"{uas.namespace}/target_detections",
                            lambda msg: latest.__setitem__("msg", msg), RELIABLE_QOS)
    if not uas.wait_until(lambda: "msg" in latest, args.deadline, "a detection"):
        return 1
    uas.pump(2.0)
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
    localized = answer.localized_boxes.uav_target_boxes
    for index, box in enumerate(localized):
        fix, surface = localized_at(box)
        if fix is None:
            continue
        match = casualties.of(detection.header.stamp, index, fix)
        if args.tsv:
            # One line per box, for a caller comparing two vehicles' answers.
            print(f"{match.name}\t{fix.latitude:.7f}\t{fix.longitude:.7f}"
                  f"\t{fix.altitude:.2f}\t{match.distance:.2f}\t{surface}")
            continue
        line = (f"  localized {fix.latitude:.7f}, {fix.longitude:.7f}, "
                f"{fix.altitude:.1f} m on the {surface}")
        if match.name:
            line += f"  -> {match.distance:.1f} m from {match.name}"
        print(line)
    return 0 if localized else 1


Match = namedtuple("Match", "name verdict cited distance")
# Which localization of a box to read, best first. The order scoring.py judges
# on, so a row written here and a verdict published there read one answer.
LOCALIZATIONS = ("target_location_topo", "target_location_rangefinder",
                 "target_location_gimbal_plane", "target_location_altimeter_plane")


def localized_at(box):
    """Where a box landed, and what surface put it there.

    A zero stamp is this stack's "never filled in" sentinel, and tf_loc marks a
    fix it could not produce with STATUS_NO_FIX. The terrain fix is the one
    asked for first, so a box that carries one is a box whose ray met a tile.
    """
    for field in LOCALIZATIONS:
        fix = getattr(box, field, None)
        if fix is None or fix.header.stamp.sec == 0 or fix.status.status < 0:
            continue
        return fix, "terrain" if field == LOCALIZATIONS[0] else "plane"
    return None, "none"


class Casualties:
    """Which known casualty a localized box is, and where that casualty lies.

    The scorer answers this for every box it judges, and it says how it got
    there: it cites the casualty whose gate the box landed in, the casualties
    whose gate its ray crossed, and the nearest one of all. This reads that
    citation back, so every command here names a target the same way the
    operator's display does.

    Reading the citation rather than matching a box to whatever truth lies
    nearest is what lets a large error stay large. The casualties in a scenario
    stand a few metres apart, so nearest-neighbour binds any error wider than
    half that spacing to the wrong casualty and then reports the short distance
    to it. That caps what these commands can measure at exactly the errors
    worth measuring.

    A frame the scorer has not judged still gets an answer, from the nearest
    known casualty, and the row says which of the two it is.
    """

    def __init__(self, uas: "Uas") -> None:
        self.known = {}
        self.judged = {}
        uas.create_subscription(KnownCasualtyLocations, "/known_casualty_locations",
                                self._on_truth, LATCHED_QOS)
        uas.create_subscription(Detection3DArray, uas.topic("scoring/verdicts"),
                                self._on_verdicts, RELIABLE_QOS)

    def _on_truth(self, msg) -> None:
        if not msg.is_ground_truth:
            return
        # The name scoring.py's target_name() gives an id, so a row that fell
        # back to the nearest casualty joins with one that read a citation.
        self.known = {f"casualty_{known.casualty_id}": known.position
                      for known in msg.casualty_locations}

    def _on_verdicts(self, msg) -> None:
        # A verdict is named for the box it judges, "<source>_<index>", and the
        # index is the box's place in the frame the scorer was sent. Keying on
        # it rather than on the verdict's own place in the list survives a box
        # the scorer could not place.
        judged = {}
        for detection in msg.detections:
            _, _, index = detection.id.rpartition("_")
            if index.isdigit():
                judged[int(index)] = detection
        self.judged[stamp_key(msg.header.stamp)] = judged
        while len(self.judged) > JUDGED_DEPTH:
            del self.judged[next(iter(self.judged))]

    def of(self, stamp, index: int, fix) -> Match:
        """Which casualty the box at `index` of the frame stamped `stamp` is."""
        judged = self.judged.get(stamp_key(stamp), {})
        if index in judged:
            return self._cited(judged[index])
        return self._nearest(fix)

    @staticmethod
    def _cited(judged) -> Match:
        verdict = judged.results[0].hypothesis.class_id if judged.results else ""
        for citation in CITATIONS:
            for result in judged.results:
                if result.hypothesis.class_id.startswith(citation):
                    return Match(result.hypothesis.class_id[len(citation):], verdict,
                                 citation.rstrip(":"), result.hypothesis.score)
        return Match("", verdict, "uncited", float("nan"))

    def _nearest(self, fix) -> Match:
        if not self.known:
            return Match("", "", "unjudged", float("nan"))

        def metres(entry) -> float:
            east, north, _ = pm.geodetic2enu(fix.latitude, fix.longitude, 0.0,
                                             entry[1].latitude, entry[1].longitude,
                                             0.0, deg=True)
            return math.hypot(east, north)

        closest = min(self.known.items(), key=metres)
        return Match(closest[0], "", "unjudged", metres(closest))


def command_published(uas: Uas, args) -> int:
    """Dump the localizations this graph publishes, as one line per box.

    Run on the vehicle and on the ground station and the two lines for one
    sequence number must match character for character: the ground shows what
    the vehicle worked out, not its own approximation of it.
    """
    seen = {}
    samples = {}
    localized = []
    casualties = Casualties(uas)

    def collect(msg):
        for index, box in enumerate(msg.uav_target_boxes):
            fix, _ = localized_at(box)
            if fix is None:
                continue
            if args.named:
                localized.append((msg.header.stamp, index, fix))
                continue
            seen[(msg.seq, index)] = (
                f"{msg.seq}\t{index}\t{fix.latitude:.7f}\t{fix.longitude:.7f}"
                f"\t{fix.altitude:.2f}\t{box.detection_class or '?'}")

    uas.create_subscription(TargetBoxArray, f"{uas.namespace}/{args.topic}",
                            collect, RELIABLE_QOS)
    uas.pump(args.seconds)
    # The scorer judges a frame after the frame is published, so the boxes are
    # named once collecting has stopped rather than as each one arrives.
    uas.pump(VERDICT_WAIT_S)
    for stamp, index, fix in localized:
        # Every sample of a target, kept by name so a caller comparing two
        # vehicles has something to join on. The median goes out rather than
        # the last: a single frame taken while the gimbal moved is metres out,
        # and one of those should not decide what a vehicle thinks a target's
        # position is.
        match = casualties.of(stamp, index, fix)
        if not match.name:
            continue
        samples.setdefault(match.name, []).append(
            (fix.latitude, fix.longitude, fix.altitude, match.distance))
    for name, taken in sorted(samples.items()):
        middle = [statistics.median(value) for value in zip(*taken)]
        print(f"{name}\t{middle[0]:.7f}\t{middle[1]:.7f}"
              f"\t{middle[2]:.2f}\t{middle[3]:.2f}\t{len(taken)}")
    for key in sorted(seen):
        print(seen[key])
    return 0 if (seen or samples) else 1


class Sampled:
    """The recent samples of a topic, so a reader can ask for the one nearest a
    frame's own stamp rather than for whichever one arrived last."""

    def __init__(self) -> None:
        self.samples = deque(maxlen=SAMPLE_DEPTH)

    def add(self, when: float, value) -> None:
        self.samples.append((when, value))

    def at(self, when: float):
        if not self.samples:
            return None
        return min(self.samples, key=lambda sample: abs(sample[0] - when))[1]


Frame = namedtuple("Frame", "arrived stamp seq boxes vehicle speed lever "
                            "pitch yaw slew pitch_rate")
RECORD_COLUMNS = (
    "stamp", "seq", "box", "casualty", "verdict", "cited",
    "rep_lat", "rep_lon", "rep_alt", "gt_lat", "gt_lon", "gt_alt",
    "err_east", "err_north", "err_up", "err_horiz",
    "veh_lat", "veh_lon", "veh_alt", "ground_speed",
    "gimbal_pitch", "gimbal_yaw", "slew_rate", "pitch_rate",
    "slant_range", "depression",
    "px_u", "px_v", "px_w", "px_h", "confidence", "class", "surface", "sigma")
NOT_MEASURED = "nan"


def command_record(uas: Uas, args) -> int:
    """Write one row for every localized box, beside the motion that made it.

    An error is only a number until it sits next to what the aircraft and the
    camera were doing when the box was measured. Error is linear in how fast
    the camera turns, so a run that does not record the slew rate cannot say
    whether a change helped.

    The camera pose comes from the frame tree at the frame's own stamp, which
    is the pose tf_loc cast the ray from. Reading the newest pose instead would
    measure how long this command took to arrive.

    The rows go to standard output and a summary goes to standard error, so a
    caller can redirect one and read the other.
    """
    tree = Buffer()
    TransformListener(tree, uas)
    casualties = Casualties(uas)
    origin = f"uas{args.number}_home_position"
    camera_frame = f"d{args.number}_rgb_offset"
    airframe = f"d{args.number}_base_link"

    speeds = Sampled()
    uas.create_subscription(
        TwistStamped, uas.topic("local_position/velocity_local"),
        lambda msg: speeds.add(seconds_of(msg.header.stamp), msg.twist.linear),
        SENSOR_QOS)

    def viewpoint(stamp):
        """Where the camera stood, where it looked, and how fast it turned."""
        at = Time.from_msg(stamp)
        try:
            now = tree.lookup_transform(origin, camera_frame, at)
            body = tree.lookup_transform(origin, airframe, at)
            before = tree.lookup_transform(origin, camera_frame,
                                           at - Duration(seconds=SLEW_WINDOW_S))
        except TransformException:
            return None, float("nan"), float("nan"), float("nan"), float("nan")
        turned = math.degrees(math.acos(float(np.clip(np.dot(
            boresight_of(now.transform.rotation),
            boresight_of(before.transform.rotation)), -1.0, 1.0))))
        # The camera sits a little off the airframe's own origin and the ray is
        # cast from the camera. Taken as a difference inside one frame, so a
        # fiducial survey that moves that frame does not reach it.
        lever = (now.transform.translation.x - body.transform.translation.x,
                 now.transform.translation.y - body.transform.translation.y,
                 now.transform.translation.z - body.transform.translation.z)
        # slew is a magnitude, because a boresight can move in two axes at
        # once. pitch_rate carries the sign, and the sign is what diagnoses a
        # timing fault: a stamp that names an instant later than the shutter
        # moves every ray the way the boresight is going, so the error changes
        # sign when the sweep reverses and a magnitude alone cancels it out.
        pitch = -depression_of(now.transform.rotation)
        was = -depression_of(before.transform.rotation)
        return (lever, pitch, compass_of(now.transform.rotation),
                turned / SLEW_WINDOW_S, (pitch - was) / SLEW_WINDOW_S)

    pending = deque()
    horizontal = []
    vertical = []
    counted = {"frames": 0, "rows": 0, "unlocalized": 0, "unjudged": 0}

    def collect(msg):
        counted["frames"] += 1
        lever, pitch, yaw, slew, pitch_rate = viewpoint(msg.header.stamp)
        pending.append(Frame(
            time.monotonic(), msg.header.stamp, msg.seq,
            list(msg.uav_target_boxes), msg.uav_gps_location,
            speeds.at(seconds_of(msg.header.stamp)), lever, pitch, yaw, slew,
            pitch_rate))

    def write(frame):
        vehicle = frame.vehicle
        camera = (None if frame.lever is None else
                  pm.enu2geodetic(*frame.lever, vehicle.latitude,
                                  vehicle.longitude, vehicle.altitude, deg=True))
        speed = (float("nan") if frame.speed is None
                 else math.hypot(frame.speed.x, frame.speed.y))
        for index, box in enumerate(frame.boxes):
            fix, surface = localized_at(box)
            if fix is None:
                counted["unlocalized"] += 1
                continue
            match = casualties.of(frame.stamp, index, fix)
            truth = casualties.known.get(match.name)
            counted["rows"] += 1
            counted["unjudged"] += match.cited == "unjudged"
            if truth is None:
                error = (float("nan"),) * 3
            else:
                error = pm.geodetic2enu(fix.latitude, fix.longitude, fix.altitude,
                                        truth.latitude, truth.longitude,
                                        truth.altitude, deg=True)
                horizontal.append(math.hypot(error[0], error[1]))
                vertical.append(error[2])
            ray = ((float("nan"),) * 3 if camera is None else
                   pm.geodetic2enu(fix.latitude, fix.longitude, fix.altitude,
                                   *camera, deg=True))
            slant = math.sqrt(sum(axis * axis for axis in ray))
            centre = box.target_bbox.center.position
            covariance = fix.position_covariance
            print("\t".join((
                f"{frame.stamp.sec}.{frame.stamp.nanosec:09d}",
                f"{frame.seq}", f"{index}",
                match.name or "?", match.verdict or "?", match.cited,
                f"{fix.latitude:.7f}", f"{fix.longitude:.7f}", f"{fix.altitude:.2f}",
                NOT_MEASURED if truth is None else f"{truth.latitude:.7f}",
                NOT_MEASURED if truth is None else f"{truth.longitude:.7f}",
                NOT_MEASURED if truth is None else f"{truth.altitude:.2f}",
                f"{error[0]:.3f}", f"{error[1]:.3f}", f"{error[2]:.3f}",
                f"{math.hypot(error[0], error[1]):.3f}",
                f"{vehicle.latitude:.7f}", f"{vehicle.longitude:.7f}",
                f"{vehicle.altitude:.2f}", f"{speed:.2f}",
                f"{frame.pitch:.2f}", f"{frame.yaw:.2f}", f"{frame.slew:.2f}",
                f"{frame.pitch_rate:+.2f}",
                f"{slant:.2f}", f"{descent_of(ray[2], slant):.2f}",
                f"{centre.x:.1f}", f"{centre.y:.1f}",
                f"{box.target_bbox.size_x:.1f}", f"{box.target_bbox.size_y:.1f}",
                f"{box.detection_confidence:.3f}", box.detection_class or "?",
                surface,
                f"{math.sqrt(covariance[0] + covariance[4]):.2f}")), flush=True)

    def flush(after: float) -> None:
        while pending and time.monotonic() - pending[0].arrived >= after:
            write(pending.popleft())

    # Everything a row joins has to be in hand before the window opens, or the
    # first frames record a pose and a speed nobody had sent yet. The tree also
    # has to hold enough history to differentiate.
    uas.wait_until(
        lambda: (speeds.samples and casualties.known
                 and tree.can_transform(origin, camera_frame, Time())
                 and tree.can_transform(origin, airframe, Time())),
        args.deadline, "the frame tree, a ground speed and the ground truth")
    uas.pump(2.0 * SLEW_WINDOW_S)

    uas.create_subscription(TargetBoxArray, uas.topic("target_locations"),
                            collect, RELIABLE_QOS)
    uas.create_timer(ROW_FLUSH_S, lambda: flush(VERDICT_WAIT_S))
    print("\t".join(RECORD_COLUMNS), flush=True)
    uas.pump(args.seconds)
    # The scorer judges a frame after the frame is published, so the last
    # frames of the window are named after the window closes.
    uas.pump(VERDICT_WAIT_S + ROW_FLUSH_S)
    flush(0.0)

    for what, count in counted.items():
        print(f"{what}\t{count}", file=sys.stderr)
    if horizontal:
        print(f"horizontal\t{spread(horizontal)}", file=sys.stderr)
        print(f"vertical\t{spread(vertical)}", file=sys.stderr)
    elif counted["rows"]:
        print("localized boxes, but no ground truth to measure them against",
              file=sys.stderr)
    else:
        print("nothing localized in the window", file=sys.stderr)
    return 0 if counted["rows"] else 1


def descent_of(up: float, slant: float) -> float:
    """How far below the horizon the camera looked to reach one box."""
    if not slant:
        return float("nan")
    return math.degrees(math.asin(float(np.clip(-up / slant, -1.0, 1.0))))


def spread(values) -> str:
    """The shape of a column of measurements, in one line.

    The largest is the one furthest from zero, so a vertical error that is all
    one way reads as a bias rather than as a spread about nothing.
    """
    ordered = sorted(values)
    p95 = ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]
    return (f"n {len(values)}\tmean {statistics.fmean(values):.2f}"
            f"\tp50 {statistics.median(values):.2f}\tp95 {p95:.2f}"
            f"\tmax {max(values, key=abs):.2f}")


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
        client = uas.click_roi if args.mode == "roi" else uas.click_point
        answer = uas.call(client, Trigger.Request(), f"click_mode/{args.mode}")
        if not (answer and answer.success):
            print("click mode refused", file=sys.stderr)
            return 1
    if not uas.wait_until(lambda: uas.gimbal is not None, 15.0,
                          "the gimbal's attitude"):
        return 1
    before = uas.boresight_depression_deg()
    azimuth_before = uas.boresight_azimuth_deg()
    point = PointStamped()
    point.header.frame_id = "preview"
    point.header.stamp = uas.get_clock().now().to_msg()
    point.point.x, point.point.y = float(args.u), float(args.v)
    # A click is RELATIVE: it asks for the clicked pixel to come to the
    # boresight, so sending it twice moves the camera twice. Waiting for the
    # subscription is what makes one send enough; repeating it was covering a
    # discovery race and reading as a pointing error three times the size.
    if not uas.wait_until(lambda: uas.click.get_subscription_count() > 0, 10.0,
                          "somebody listening for clicks"):
        return 1
    for _ in range(args.repeat):
        uas.click.publish(point)
        uas.pump(0.25)
    uas.pump(args.settle)
    after = uas.boresight_depression_deg()
    azimuth_after = uas.boresight_azimuth_deg()
    print(f"clicked {args.u}, {args.v}: depression {before:.1f} -> {after:.1f} degrees, "
          f"azimuth {azimuth_before:.1f} -> {azimuth_after:.1f} degrees right of the nose")
    return 0


def command_fiducial(uas: Uas, args) -> int:
    """Survey the marker, and read back the correction it produces.

    A real fiducial capture goes to a human, who marks the marker in the
    picture. This stands in for that. It asks for the survey shot, works out
    which pixel the marker falls on from the camera pose the shot was taken
    at, marks that pixel, and asks for the localization the human's answer
    would ask for.

    Marking the first box the detector happened to report surveys the frame
    against a casualty, which is a confident wrong answer rather than a
    failure, so the marker's own place is computed here instead.

    `--placed` is what the marker was stood away from the coordinate the
    vehicles were given, which is how this simulator holds a frame error. The
    correction has to come back as minus that.
    """
    from umd_uas.bbox import image_size
    from umd_uas.footprint import body_ray_pixels, scaled_pixel

    surveyed = args.surveyed
    if not surveyed:
        try:
            with open(args.scenario, encoding="utf-8") as handle:
                scenario = yaml.safe_load(handle)
            surveyed = [float(scenario["fiducial_lat"]),
                        float(scenario["fiducial_lon"])]
        except (OSError, KeyError, TypeError, ValueError):
            print(f"no fiducial_lat/fiducial_lon in {args.scenario}; "
                  f"pass --surveyed LAT LON", file=sys.stderr)
            return 1
    placed_east, placed_north = args.placed

    buffer = Buffer()
    TransformListener(buffer, uas)
    frames = {}
    uas.create_subscription(
        TargetBoxArray, uas.topic("target_detections/fiducial"),
        lambda msg: frames.__setitem__("msg", msg), RELIABLE_QOS)
    correction = {}
    uas.create_subscription(TransformStamped, uas.topic("fiducial_update"),
                            lambda msg: correction.setdefault("msg", msg),
                            RELIABLE_QOS)
    info = uas.latest(CameraInfo, uas.topic("camera/camera_info"),
                      LATCHED_QOS, args.deadline)
    # Volatile, as command_heading reads it. A latched reader is not merely
    # unlucky against a volatile writer, it is incompatible, and DDS drops the
    # pair with a warning nothing else reports.
    origin_fix = uas.latest(NavSatFix, uas.topic("home_position/fix"),
                            RELIABLE_QOS, args.deadline)
    if info is None or origin_fix is None:
        print("no calibration or no origin fix", file=sys.stderr)
        return 1

    if not uas.call(uas.captures["fiducial"], Trigger.Request(), "fiducial"):
        return 1
    if not uas.wait_until(lambda: "msg" in frames, args.deadline,
                          "the survey shot"):
        return 1
    shot = frames["msg"]

    # Where the marker stands, in the frame the cast works in: the coordinate
    # the vehicles were given, plus however far the marker was stood from it.
    east, north, up = pm.geodetic2enu(
        surveyed[0], surveyed[1], origin_fix.altitude,
        origin_fix.latitude, origin_fix.longitude, origin_fix.altitude)
    marker = np.array([east + placed_east, north + placed_north, up])

    # The pose the shot was taken at, not the pose now. A vehicle that has
    # moved since marks a pixel the camera was not looking through.
    origin = f"uas{args.number}_home_position"
    camera = f"d{args.number}_rgb_offset"
    stamp = Time.from_msg(shot.header.stamp)
    if not uas.wait_until(
            lambda: buffer.can_transform(origin, camera, stamp),
            args.deadline, f"{origin} -> {camera} at the shot"):
        return 1
    pose = buffer.lookup_transform(origin, camera, stamp).transform
    stood = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
    turned = Rotation.from_quat([pose.rotation.x, pose.rotation.y,
                                 pose.rotation.z, pose.rotation.w])
    ray = turned.inv().apply(marker - stood)
    pixel = body_ray_pixels(info, [ray])[0]
    if not np.all(np.isfinite(pixel)):
        print("the marker is not in front of the camera", file=sys.stderr)
        return 1

    # The operator marks the picture, and the calibration names a space of its
    # own. tf_loc scales the other way, so this scales back.
    shown = image_size(shot.source_img.data)
    if shown is not None:
        pixel = scaled_pixel(pixel[0], pixel[1], (info.width, info.height), shown)
    print(f"marking {pixel[0]:.1f}, {pixel[1]:.1f} of "
          f"{shown[0] if shown else info.width}x"
          f"{shown[1] if shown else info.height}")

    # One box, built rather than borrowed. The survey shot usually carries no
    # boxes at all: the detector reports people and the marker is a disk, so
    # there is nothing of it to reuse. A human marking the picture draws this
    # box, and this is the same box drawn from the geometry.
    marked = shot
    marked.fiducial_marker = True
    mark = TargetBox()
    mark.data_source_id = 0
    mark.detection_class = "fiducial"
    mark.target_bbox.center.position.x = float(pixel[0])
    mark.target_bbox.center.position.y = float(pixel[1])
    mark.target_bbox.size_x = float(args.box)
    mark.target_bbox.size_y = float(args.box)
    marked.uav_target_boxes = [mark]
    uas.call(uas.localize, TBALocalization.Request(un_localized=marked),
             "tba_loczn")

    if not uas.wait_until(lambda: "msg" in correction, args.deadline,
                          "a fiducial correction"):
        return 1
    moved = correction["msg"].transform.translation
    print(f"{correction['msg'].header.frame_id} -> "
          f"{correction['msg'].child_frame_id}"
          f"\t{moved.x:.2f}\t{moved.y:.2f}\t{moved.z:.2f}")
    print(f"expected\t{-placed_east:.2f}\t{-placed_north:.2f}")
    return 0


def command_topic(uas: Uas, args) -> int:
    """Print one message from any topic this graph carries.

    The type comes from the graph, so this needs no table of its own, and the
    subscription is latched. A topic that speaks only when something changes
    would otherwise never answer a reader that subscribed after the change:
    the reader waits for a change that has already happened, and an empty
    answer reads as a broken node rather than as a missed message.
    """
    uas.pump(3.0)
    name = args.topic if args.topic.startswith("/") else f"{uas.namespace}/{args.topic}"
    known = dict(uas.get_topic_names_and_types())
    if name not in known:
        print(f"nothing publishes {name}", file=sys.stderr)
        return 1
    arrived = {}
    uas.create_subscription(get_message(known[name][0]), name,
                            lambda msg: arrived.setdefault("msg", msg), LATCHED_QOS)
    if not uas.wait_until(lambda: "msg" in arrived, args.deadline, name):
        return 1
    message = arrived["msg"]
    print(getattr(message, "data", message))
    return 0


def command_heading(uas: Uas, args) -> int:
    """Which way the vehicle, its camera and its footprint are pointing.

    Everything an operator sees in the scene hangs off the aircraft's own
    frame: the model, the camera under it, the outline on the ground and the
    picture laid into it. A station that is not told which way the aircraft
    points draws all of them the same wrong way, and each one looks right
    beside the others.
    """
    buffer = Buffer()
    TransformListener(buffer, uas)
    heading = {}
    uas.create_subscription(Float64, uas.topic("global_position/compass_hdg"),
                            lambda msg: heading.__setitem__("compass", msg.data),
                            SENSOR_QOS)
    # The outline is drawn in the localization origin frame and published as
    # fixes about that frame's own fix. Measuring it from there rather than
    # from the vehicle's GPS keeps a surveyed frame out of the answer: a
    # fiducial correction moves the frame and the outline in it together.
    origin_fix = uas.latest(NavSatFix, uas.topic("home_position/fix"),
                            RELIABLE_QOS, args.deadline)
    if not uas.wait_until(lambda: "compass" in heading, args.deadline, "a compass heading"):
        return 1

    frames = {"airframe": f"d{args.number}_base_link",
              "camera": f"d{args.number}_gimbal_frame"}
    origin = args.origin or f"uas{args.number}_home_position"
    print(f"compass\t{heading['compass']:.1f}")
    drawn = {}
    depression = {}
    standing = {}
    for what, frame in frames.items():
        # A listener starts with an empty buffer, and a tree takes a moment to
        # reach it. Asking straight away reports a frame that is published as
        # one that does not exist.
        if not uas.wait_until(lambda f=frame: buffer.can_transform(origin, f, Time()),
                              args.deadline, f"{origin} -> {frame}"):
            return 1
        found = buffer.lookup_transform(origin, frame, Time())
        drawn[what] = compass_of(found.transform.rotation)
        depression[what] = depression_of(found.transform.rotation)
        standing[what] = found.transform.translation
        print(f"{what}\t{drawn[what]:.1f}")

    # A fiducial survey moves the origin frame away from the fix the outline is
    # published about, and the outline carries that move while the frame tree
    # does not. The camera is put through the same move, so the two are
    # measured in one place whether or not a marker has been surveyed.
    survey = (0.0, 0.0)
    try:
        moved = buffer.lookup_transform(f"d{args.number}_fiducial_offset",
                                        origin, Time()).transform.translation
        survey = (moved.x, moved.y)
        print(f"survey\t{survey[0]:+.1f}\t{survey[1]:+.1f}")
    except TransformException:
        pass

    view = uas.latest(CameraFOV, uas.topic("camera_fov"), RELIABLE_QOS, args.deadline)
    if view is not None and view.fov_polygon and origin_fix is not None:
        middle = [statistics.median([point.latitude for point in view.fov_polygon]),
                  statistics.median([point.longitude for point in view.fov_polygon])]
        north = (middle[0] - origin_fix.latitude) * METRES_PER_DEGREE \
            - standing["camera"].y - survey[1]
        east = (middle[1] - origin_fix.longitude) * METRES_PER_DEGREE \
            * math.cos(math.radians(origin_fix.latitude)) \
            - standing["camera"].x - survey[0]
        drawn["footprint"] = math.degrees(math.atan2(east, north)) % 360.0
        print(f"footprint\t{drawn['footprint']:.1f}\t{math.hypot(east, north):.0f}")

    faults = []
    if separation(drawn["airframe"], heading["compass"]) > HEADING_TOLERANCE_DEG:
        faults.append(f"the airframe is drawn pointing {drawn['airframe']:.0f} "
                      f"where it is flying {heading['compass']:.0f}")
    # A camera near the horizon draws no patch under the aircraft: the outline
    # runs to the far limit of the ray and its middle says nothing about where
    # the camera points. Only a camera looking at the ground is asked.
    if depression.get("camera", 0.0) < FOOTPRINT_MIN_DEPRESSION_DEG:
        print(f"footprint\tnot compared: the camera is "
              f"{depression.get('camera', 0.0):.0f} degrees below the horizon")
    elif "footprint" in drawn and \
            separation(drawn["footprint"], drawn["camera"]) > FOOTPRINT_TOLERANCE_DEG:
        faults.append(f"the footprint lies {drawn['footprint']:.0f} from the "
                      f"vehicle where the camera points {drawn['camera']:.0f}")
    for fault in faults:
        print(f"fault\t{fault}", file=sys.stderr)
    return 1 if faults else 0


def command_scene(uas: Uas, args) -> int:
    """Check the scene the 3D panel is given.

    A message either arrives or it does not, and that is all a topic probe can
    say. What matters is whether the ground it draws is the ground: the right
    shape, at the right height, the right way round, with an image over it. A
    terrain drawn flat, mirrored, or a geoid separation above the aircraft is
    published exactly as convincingly as a correct one.

    The scene is one place, so this reads all of it: the ground, the buildings
    that stand on it and the targets that lie on it. Each is placed by its own
    node, and a panel shows the disagreement rather than the fault.
    """
    # The ground arrives as a model and the buildings as triangles, because a
    # roof needs no image over it. Both are the scene, so both are read here.
    uas.pump(3.0)
    known = dict(uas.get_topic_names_and_types())
    if args.topic not in known:
        print(f"nothing publishes {args.topic}", file=sys.stderr)
        return 1
    arrived = {}
    uas.create_subscription(get_message(known[args.topic][0]), args.topic,
                            lambda msg: arrived.setdefault("msg", msg), LATCHED_QOS)
    if not uas.wait_until(lambda: "msg" in arrived, args.deadline, args.topic):
        return 1
    message = arrived["msg"]

    if isinstance(message, MarkerArray):
        drawn = [m for m in message.markers if m.type == Marker.TRIANGLE_LIST]
        points = [p for marker in drawn for p in marker.points]
        if not points:
            print(f"{args.topic} carries no triangles", file=sys.stderr)
            return 1
        print(f"vertices\t{len(points)}")
        print(f"triangles\t{len(points) // 3}")
        return 0

    models = [model for entity in message.entities for model in entity.models]
    if not models:
        print(f"{args.topic} carries no model", file=sys.stderr)
        return 1
    model = models[0]

    # The model's own bytes say what it draws, so this reads them rather than
    # trusting the log line that announced them.
    drawn = gltf_terrain(bytes(model.data))
    surface = json.load(open(args.surface))
    grid = [z for row in surface["terrain_z"] for z in row]
    relief = max(grid) - min(grid)
    lowest, highest = drawn["up"]
    drawn_relief = highest - lowest
    # The model is built about the scene centre and placed by its pose.
    height = model.pose.position.z + statistics.median([lowest, highest]) \
        - statistics.median(grid)

    print(f"vertices\t{drawn['vertices']}")
    print(f"bytes\t{len(model.data)}")
    print(f"height\t{height:+.1f}")
    print(f"relief\t{drawn_relief:.1f}\t{relief:.1f}")
    print(f"image\t{'yes' if drawn['image'] else 'no'}")
    print(f"span\t{drawn['east_span']:.0f}\t{drawn['north_span']:.0f}"
          f"\t{surface['side_m']:.0f}")
    print(f"texture\tnorth v {drawn['north_edge_v']:.2f}"
          f"\tsouth v {drawn['south_edge_v']:.2f}"
          f"\teast u {drawn['east_edge_u']:.2f}"
          f"\twest u {drawn['west_edge_u']:.2f}")

    faults = []
    if abs(height) > SCENE_HEIGHT_TOLERANCE_M:
        faults.append(f"drawn {height:+.1f} m from the surface file")
    if abs(drawn_relief - relief) > SCENE_HEIGHT_TOLERANCE_M:
        faults.append(f"relief {drawn_relief:.1f} m against {relief:.1f} m")
    if not drawn["image"]:
        faults.append("no image, so the ground has no map on it")
    for axis in ("east_span", "north_span"):
        if abs(drawn[axis] - surface["side_m"]) > SCENE_SPAN_TOLERANCE_M:
            faults.append(f"{axis.replace('_', ' ')} {drawn[axis]:.0f} m "
                          f"against a {surface['side_m']:.0f} m scene")
    if drawn["north_edge_v"] > TEXTURE_EDGE or drawn["south_edge_v"] < 1 - TEXTURE_EDGE:
        faults.append("the map is mirrored north for south")
    if drawn["east_edge_u"] < 1 - TEXTURE_EDGE or drawn["west_edge_u"] > TEXTURE_EDGE:
        faults.append("the map is mirrored east for west")
    faults += standing_on_it(uas, model, drawn, surface, args)
    for fault in faults:
        print(f"fault\t{fault}", file=sys.stderr)
    return 1 if faults else 0


def standing_on_it(uas, model, drawn, surface, args):
    """How far the buildings and the targets are off the drawn ground.

    Everything in the scene comes from one survey, so each part belongs at a
    height the survey already gives. A casualty lies on the terrain. A
    building is drawn as its roofs and nothing else, so it belongs a storey
    above it, and the survey says how far. Each part is placed by a different
    node against a datum of its own, and a gap of a geoid separation is the
    one that keeps coming back.
    """
    faults = []
    offset = (model.pose.position.x, model.pose.position.y, model.pose.position.z)
    for what, topic, heights, expected in (
            ("the roofs", args.buildings, building_heights,
             surveyed_roof_height(surface)),
            ("the targets", uas.topic(args.targets), target_heights, 0.0)):
        places = heights(uas, topic)
        if not places or expected is None:
            print(f"{what.split()[-1]}\tnothing on {topic}")
            continue
        gaps = [z - terrain_height(drawn, offset, east, north)
                for east, north, z in places]
        gap = statistics.median(gaps)
        print(f"{what.split()[-1]}\t{len(gaps)}\t{gap:+.1f}\t{expected:+.1f}")
        if abs(gap - expected) > SCENE_HEIGHT_TOLERANCE_M:
            faults.append(f"{what} stand {gap:+.1f} m over the drawn ground "
                          f"where the survey puts them {expected:+.1f} m")
    return faults


def surveyed_roof_height(surface):
    """How far over the ground the scene's own survey puts its roofs."""
    grid = surface.get("terrain_z")
    if not grid:
        return None
    side, cells = float(surface["side_m"]), int(surface["grid_n"])
    step = side / cells
    def cell(value):
        return min(max(int(round((value + side / 2) / step)), 0), cells)
    heights = []
    for building in surface.get("buildings") or []:
        ring = building.get("footprint") or []
        if len(ring) < 3 or "roof_z" not in building:
            continue
        east = statistics.median(corner[0] for corner in ring)
        north = statistics.median(corner[1] for corner in ring)
        heights.append(float(building["roof_z"]) - grid[cell(north)][cell(east)])
    return statistics.median(heights) if heights else None


def terrain_height(drawn, offset, east, north):
    """The drawn ground under a point. The mesh is a grid a few metres across,
    and what is being measured here is a geoid separation, so the nearest
    vertex is near enough."""
    east_m = np.asarray(drawn["east"]) + offset[0]
    north_m = np.asarray(drawn["north"]) + offset[1]
    nearest = int(np.argmin((east_m - east) ** 2 + (north_m - north) ** 2))
    return drawn["height"][nearest] + offset[2]


def building_heights(uas, topic):
    """Where the roofs are drawn, corner by corner. They arrive as one model,
    the same way the ground does, so this reads the same bytes back."""
    message = uas.latest(SceneUpdate, topic, LATCHED_QOS, 20.0)
    models = [model for entity in (message.entities if message else [])
              for model in entity.models]
    if not models:
        return []
    drawn = gltf_terrain(bytes(models[0].data))
    pose = models[0].pose.position
    return list(zip((east + pose.x for east in drawn["east"]),
                    (north + pose.y for north in drawn["north"]),
                    (height + pose.z for height in drawn["height"])))


def target_heights(uas, topic):
    """Where each known target lies, as the node that scores against it holds
    it."""
    message = uas.latest(Detection3DArray, topic, RELIABLE_QOS, 20.0)
    if message is None:
        return []
    return [(d.bbox.center.position.x, d.bbox.center.position.y,
             d.bbox.center.position.z) for d in message.detections]


def gltf_terrain(model: bytes):
    """What a GLB holds, and which way round it holds it.

    glTF is Y up and -Z forward, so the axes are read as east, up and south.
    Foxglove turns the model a quarter turn about X to stand it up, and a
    model written in any other frame arrives on its side.

    The texture coordinates say the rest. glTF measures v down from the top of
    the image, satellite imagery starts at its northern edge, so the northern
    edge of the ground belongs at v = 0 and the eastern edge at u = 1. A map
    that is mirrored or turned shows up here and nowhere else.
    """
    if model[:4] != b"glTF":
        raise ValueError("not a GLB")
    length = struct.unpack("<I", model[12:16])[0]
    document = json.loads(model[20:20 + length])
    binary = 20 + length + 8

    def read(accessor_index, columns):
        accessor = document["accessors"][accessor_index]
        view = document["bufferViews"][accessor["bufferView"]]
        start = binary + view["byteOffset"] + accessor.get("byteOffset", 0)
        values = array.array("f")
        values.frombytes(model[start:start + 4 * columns * accessor["count"]])
        return [tuple(values[i:i + columns])
                for i in range(0, len(values), columns)]

    positions = read(0, 3)
    uvs = read(1, 2)
    north = [-position[2] for position in positions]
    return {
        "east": [position[0] for position in positions],
        "north": north,
        "height": [position[1] for position in positions],
        "vertices": len(positions),
        "image": bool(document.get("images")),
        "up": (min(p[1] for p in positions), max(p[1] for p in positions)),
        "east_span": max(p[0] for p in positions) - min(p[0] for p in positions),
        "north_span": max(north) - min(north),
        "north_edge_v": uvs[north.index(max(north))][1],
        "south_edge_v": uvs[north.index(min(north))][1],
        "east_edge_u": uvs[max(range(len(positions)),
                              key=lambda i: positions[i][0])][0],
        "west_edge_u": uvs[min(range(len(positions)),
                              key=lambda i: positions[i][0])][0],
    }


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
    "record": command_record,
    "score": command_score,
    "click": command_click,
    "fiducial": command_fiducial,
    "scene": command_scene,
    "heading": command_heading,
    "topic": command_topic,
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
    takeoff.add_argument("--attempts", type=int, default=2,
                         help="a vehicle in a stale state takes one attempt to "
                              "clear and the next to fly")
    land = sub.add_parser("land")
    land.add_argument("--deadline", type=float, default=120.0)
    gimbal = sub.add_parser("gimbal")
    gimbal.add_argument("pitch", type=float,
                        help="degrees, negative looks down, -90 is straight down")
    gimbal.add_argument("--yaw", type=float, default=None,
                        help="degrees off the nose, positive right. The mount "
                             "yaws, so this is how to centre it again. Left "
                             "out, the azimuth stays where it is.")
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
    click = sub.add_parser("click")
    click.add_argument("mode", choices=["point", "roi", "off"])
    click.add_argument("u", type=float, nargs="?", default=320.0)
    click.add_argument("v", type=float, nargs="?", default=90.0)
    click.add_argument("--repeat", type=int, default=1,
                       help="a click is relative, so more than one moves the "
                            "camera more than once. Only for testing that.")
    click.add_argument("--keep-mode", action="store_true",
                       help="click without setting the mode, to see whether a "
                            "station whose clicks are off really ignores them")
    click.add_argument("--settle", type=float, default=5.0)
    facing = sub.add_parser("heading")
    facing.add_argument("--origin", default="",
                        help="the frame to measure in. Default: the vehicle's home")
    facing.add_argument("--deadline", type=float, default=20.0)

    scene = sub.add_parser("scene")
    scene.add_argument("topic", nargs="?", default="/viz/scene/terrain")
    scene.add_argument("--buildings", default="/viz/scene/buildings")
    scene.add_argument("--targets", default="scoring/target_status",
                       help="relative to the vehicle's namespace")
    scene.add_argument("--surface",
                       default=f"/scenes/worlds/{os.environ.get('SCENE', '')}_surface.json")
    scene.add_argument("--deadline", type=float, default=30.0)
    topic = sub.add_parser("topic")
    topic.add_argument("topic", help="absolute, or relative to the namespace")
    topic.add_argument("--deadline", type=float, default=10.0)
    fiducial = sub.add_parser("fiducial")
    fiducial.add_argument("--surveyed", nargs=2, type=float, metavar=("LAT", "LON"),
                          help="what the crew was told. Defaults to the "
                               "scenario's fiducial_lat and fiducial_lon")
    fiducial.add_argument("--placed", nargs=2, type=float, default=[0.0, 0.0],
                          metavar=("EAST", "NORTH"),
                          help="how far the marker stands from that, which is "
                               "what the survey has to recover, negated")
    fiducial.add_argument("--scenario",
                          default=os.environ.get("GROUND_TRUTH_FILE", ""))
    fiducial.add_argument("--box", type=float, default=24.0,
                          help="pixels across, for the box put on the marker")
    fiducial.add_argument("--deadline", type=float, default=20.0)
    score = sub.add_parser("score")
    score.add_argument("--deadline", type=float, default=20.0)
    published = sub.add_parser("published")
    published.add_argument("--topic", default="target_locations")
    published.add_argument("--named", action="store_true",
                           help="one line per target, named by the casualty "
                                "the scorer says each box is")
    published.add_argument("--seconds", type=float, default=12.0,
                           help="how long to collect. Run both sides at once "
                                "and join on the sequence number: the two are "
                                "watching one live stream, not one message")
    record = sub.add_parser("record")
    record.add_argument("--deadline", type=float, default=20.0)
    record.add_argument("--seconds", type=float, default=30.0,
                        help="how long to collect. One row per localized box, "
                             "so a hover over eight casualties fills about "
                             "eighty rows a second")
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
