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
import math
import sys
import time

import numpy as np
import pymap3d as pm
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition, State
from mavros_msgs.srv import CommandBool, CommandInt, CommandTOL, SetMode
from sensor_msgs.msg import NavSatFix
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

    def wait_until(self, ready, deadline_s: float, what: str) -> bool:
        """Poll for a condition and say which way it went. Every wait here says
        what it is waiting for, so a stack that never gets there names the step
        instead of stopping silently."""
        end = time.monotonic() + deadline_s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=SETTLE_POLL_S)
            if ready():
                return True
        print(f"gave up waiting for {what} after {deadline_s:.0f}s", file=sys.stderr)
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
                             args.deadline, f"{args.height} m over home")
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
        param4=float("nan"),
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

    reached = uas.wait_until(arrived, args.deadline,
                             f"{args.east}, {args.north}, {args.up}")
    here = uas.local.pose.position
    print(f"at {here.x:.1f} east, {here.y:.1f} north, {here.z:.1f} up from home")
    return 0 if reached else 1


COMMANDS = {
    "status": command_status,
    "arm": command_arm,
    "takeoff": command_takeoff,
    "land": command_land,
    "gimbal": command_gimbal,
    "goto": command_goto,
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
