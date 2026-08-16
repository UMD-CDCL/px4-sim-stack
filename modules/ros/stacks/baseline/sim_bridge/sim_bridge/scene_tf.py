#!/usr/bin/env python3
"""Publish the frame tree for the airframe and its cameras.

This node owns the whole tree, so one file decides what every frame's stamp
means. MAVROS ships its own map -> base_link; mavros_overrides.yaml turns it
off, because a transform this node does not publish is one it cannot correct.

    map                         local ENU, origin at the launch point
     └── base_link              FLU body frame, from the MAVROS pose
          ├── gimbal_mount      static, where the gimbal bolts on
          │    └── gimbal_camera_link       turns with the gimbal
          │         └── gimbal_camera_optical_frame
          └── nadir_camera_link static, pitched to look straight down
               └── nadir_camera_optical_frame

A camera *link* frame uses the body convention, x forward along the view
axis. A camera *optical* frame uses REP 103, x right, y down, z forward,
which CameraInfo and every projection formula assume. The fixed rotation
between them is rpy (-90, 0, -90) degrees.

Stamps, and the delay correction
--------------------------------
Every moving transform goes out on the arrival of the message that produced
it, stamped with that message's own time less the chain's delay. Nothing is
republished on a timer: a timer stamps an old attitude with the present
instant, which reads to every consumer as "the camera is there now" and puts
the localizer's ray where the camera had already moved to. The error then
grows with slew rate, which is what it looked like from the outside.

vehicle_delay and gimbal_delay trim what is left on each chain, in seconds,
positive meaning the report trails the motion it describes. They are the only
motion correction in the stack: with the stamps right, tf2 interpolates the
pose at any instant between two reports, so no consumer needs an offset of
its own and none has one. Measure them with scripts/check-localization-lag.py,
which reports the offset that flattens the error against slew rate and speed.

A gimbal that stops reporting makes its frame go stale, and a lookup at a
recent time then fails instead of returning the last attitude. That is the
honest answer, and it is why the stale case is visible in the log.

The gimbal math here undoes three conventions in turn. docs/interfaces.md
section 6 records each one and the wrong answers they produced.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

try:
    from mavros_msgs.msg import GimbalDeviceAttitudeStatus
    HAVE_GIMBAL_MSG = True
except ImportError:  # pragma: no cover - only when mavros_extras is absent
    HAVE_GIMBAL_MSG = False

from sim_bridge.projection import (LINK_TO_OPTICAL, aerospace_to_ros,
                                   body_frd_to_flu, quat_conj, quat_from_rpy,
                                   quat_mul)
from sim_bridge.runtime import now_s, spin

# ------------------------------------------------------------------- tunables
# How long a commanded setpoint stays authoritative before the stale device
# report takes over, in gimbal_source "auto".
SETPOINT_TIMEOUT_S = 3.0
# A device report counts as fresh this long. With the 50 Hz report
# (patches/px4-gzgimbal-rate.patch plus the px4-rcS stream rate) the report
# is always fresh and wins in "auto": it carries the actual joints, so the
# frame tree matches what the camera image shows. At the mavlink default
# rate, one report every few seconds, the setpoint bridges the gaps.
STATUS_FRESH_S = 0.5

REPORT_INTERVAL_S = 30.0
# Under this, a chain reports more slowly than a frame is old by the time its
# detections arrive, so a lookup at the capture time falls past the newest
# report. Roughly the inverse of the detection pipeline delay.
MIN_REPORT_RATE_HZ = 10.0

# GIMBAL_DEVICE_FLAGS_YAW_LOCK
YAW_LOCK = 16


def quat_yaw_deg(q) -> float:
    x, y, z, w = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


def yaw_only(q):
    """The yaw part of a quaternion, as a rotation about z."""
    return quat_from_rpy(0.0, 0.0, math.radians(quat_yaw_deg(q)))


class SceneTf(Node):
    def __init__(self) -> None:
        super().__init__("scene_tf")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gimbal_mount_xyz", [0.0, 0.0, 0.10])
        self.declare_parameter("nadir_xyz", [0.10, 0.0, -0.06])
        # The nadir mounting, in degrees, roll pitch yaw against base_link.
        # 0 90 0 matches the sensor pose in x500_recon/model.sdf: a quarter
        # turn about y points the camera's view axis at the ground. A small
        # error here shifts where every detection from this camera lands, so
        # it stays trimmable without a rebuild.
        self.declare_parameter("nadir_rpy_deg", [0.0, 90.0, 0.0])
        # Which frame the reported gimbal attitude is in. PX4's simulated
        # gimbal reports an absolute attitude while its flag claims vehicle
        # frame (GZGimbal.cpp), so "earth" matches what PX4 sends and this
        # node divides the vehicle attitude back out. Set "vehicle" for a
        # gimbal that reports honestly, or with px4-gzgimbal-frame.patch
        # applied.
        self.declare_parameter("gimbal_reference", "earth")
        # Where the gimbal orientation comes from.
        #   setpoint  GIMBAL_DEVICE_SET_ATTITUDE, the commanded angles,
        #             already vehicle relative.
        #   status    GIMBAL_DEVICE_ATTITUDE_STATUS, what the device reports.
        #   auto      prefer a fresh device report, then a fresh setpoint.
        #             The report is the actual joints, so it matches the
        #             image; the setpoint leads the joints by the device
        #             cycle and covers a report stream that runs slow. An
        #             untouched gimbal produces no setpoint at all, so the
        #             stale report is the last resort.
        self.declare_parameter("gimbal_source", "auto")
        # A fixed mounting rotation applied after the attitude is made body
        # relative, in degrees, as roll, pitch, yaw. The diagnostics log
        # prints the number to put here.
        self.declare_parameter("gimbal_offset_rpy_deg", [0.0, 0.0, 0.0])
        # Seconds each chain's report trails the motion it describes. The one
        # motion correction in the stack; see the module docstring.
        self.declare_parameter("vehicle_delay", 0.0)
        self.declare_parameter("gimbal_delay", 0.0)
        # Log vehicle heading against gimbal heading once a second. With the
        # gimbal centered, "gimbal rel body" should read near zero at every
        # aircraft heading. A constant is the gimbal_offset_rpy_deg value; a
        # value that moves with the aircraft means a frame handling bug.
        self.declare_parameter("log_gimbal_diagnostics", False)

        self.base = self.get_parameter("base_frame").value
        self.gimbal_reference = self.get_parameter("gimbal_reference").value
        self.gimbal_source = self.get_parameter("gimbal_source").value
        offset = [float(v) for v in self.get_parameter("gimbal_offset_rpy_deg").value]
        self.gimbal_offset = quat_from_rpy(*(math.radians(v) for v in offset))
        self.vehicle_delay = float(self.get_parameter("vehicle_delay").value)
        self.gimbal_delay = float(self.get_parameter("gimbal_delay").value)

        # Identity until the gimbal reports. A gimbal that never reports
        # leaves the camera pointing along the airframe, which is visibly
        # wrong in the 3D view rather than silently wrong in the numbers.
        self.q_status = (0.0, 0.0, 0.0, 1.0)
        self.q_setpoint = (0.0, 0.0, 0.0, 1.0)
        self.vehicle_q = (0.0, 0.0, 0.0, 1.0)
        self.gimbal_q = (0.0, 0.0, 0.0, 1.0)
        self.last_setpoint = None
        self.last_status = None
        # The stamp of the message each orientation came from, so it travels
        # with the value rather than with whatever arrived last.
        self.setpoint_stamp = None
        self.status_stamp = None
        self.gimbal_seen = False
        self.vehicle_seen = False
        self.vehicle_reports = 0
        self.gimbal_reports = 0
        self.logged_yaw_lock = False
        self.logged_setpoint = False

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self._publish_static()

        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_vehicle, qos_profile_sensor_data)
        if HAVE_GIMBAL_MSG:
            self.create_subscription(
                GimbalDeviceAttitudeStatus,
                "/mavros/gimbal_control/device/attitude_status",
                self._on_gimbal_status, 10)
            try:
                from mavros_msgs.msg import GimbalDeviceSetAttitude
                self.create_subscription(
                    GimbalDeviceSetAttitude,
                    "/mavros/gimbal_control/device/set_attitude",
                    self._on_gimbal_setpoint, 10)
            except ImportError:
                pass
        else:
            self.get_logger().warn(
                "mavros_msgs has no GimbalDeviceAttitudeStatus. The gimbal "
                "frame will stay fixed to the airframe.")

        # Identity once, so the tree is connected before the first report and
        # a startup lookup fails on the pose rather than on a missing frame.
        self._publish_gimbal()
        self.create_timer(REPORT_INTERVAL_S, self._report)
        if self.get_parameter("log_gimbal_diagnostics").value:
            self.create_timer(1.0, self._log_diagnostics)

    # ------------------------------------------------------------------ static
    def _static(self, parent: str, child: str, xyz, q) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        return t

    def _publish_static(self) -> None:
        mount = self.get_parameter("gimbal_mount_xyz").value
        nadir = self.get_parameter("nadir_xyz").value
        nadir_rotation = quat_from_rpy(*(math.radians(v) for v in
                                         self.get_parameter("nadir_rpy_deg").value))
        identity = (0.0, 0.0, 0.0, 1.0)

        self.static_broadcaster.sendTransform([
            self._static(self.base, "gimbal_mount", mount, identity),
            self._static("gimbal_camera_link", "gimbal_camera_optical_frame",
                         (0, 0, 0), LINK_TO_OPTICAL),
            self._static(self.base, "nadir_camera_link", nadir, nadir_rotation),
            self._static("nadir_camera_link", "nadir_camera_optical_frame",
                         (0, 0, 0), LINK_TO_OPTICAL),
        ])

    def _delayed(self, stamp, delay: float):
        """A message stamp less a chain's delay, as a message stamp."""
        if delay == 0.0:
            return stamp
        return (Time.from_msg(stamp)
                - Duration(seconds=delay)).to_msg()

    # ---------------------------------------------------------------- vehicle
    def _on_vehicle(self, msg) -> None:
        q = msg.pose.orientation
        self.vehicle_q = (q.x, q.y, q.z, q.w)
        self.vehicle_seen = True
        self.vehicle_reports += 1

        t = TransformStamped()
        t.header.stamp = self._delayed(msg.header.stamp, self.vehicle_delay)
        t.header.frame_id = msg.header.frame_id or "map"
        t.child_frame_id = self.base
        p = msg.pose.position
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = msg.pose.orientation
        self.dynamic_broadcaster.sendTransform(t)

    # ----------------------------------------------------------------- gimbal

    def _on_gimbal_setpoint(self, msg) -> None:
        if self.gimbal_source == "status":
            return
        self.last_setpoint = now_s(self)

        # The setpoint is already vehicle relative on any axis whose LOCK
        # flag is clear, so it needs the body axis conversion and nothing
        # else.
        flags = int(getattr(msg, "flags", 0))
        body = body_frd_to_flu((msg.q.x, msg.q.y, msg.q.z, msg.q.w))
        if flags & YAW_LOCK:
            # Yaw is earth referenced on this axis. Take the vehicle heading
            # back out to get a rotation that belongs under gimbal_mount.
            body = quat_mul(quat_conj(yaw_only(self.vehicle_q)), body)
            if not self.logged_yaw_lock:
                self.logged_yaw_lock = True
                self.get_logger().info(
                    "gimbal yaw is earth locked, removing the vehicle heading")

        if not self.logged_setpoint:
            self.logged_setpoint = True
            self.gimbal_seen = True
            self.get_logger().info(
                f"gimbal orientation from the commanded setpoint, flags={flags}")

        self.q_setpoint = quat_mul(body, self.gimbal_offset)
        self.setpoint_stamp = self._delayed(msg.header.stamp, self.gimbal_delay)
        self._publish_gimbal()

    def _on_gimbal_status(self, msg) -> None:
        if not self.gimbal_seen:
            self.gimbal_seen = True
            self.get_logger().info(
                f"gimbal attitude received, flags={getattr(msg, 'flags', 0)}, "
                f"treated as {self.gimbal_reference} referenced")

        # MAVROS passes the PX4 quaternion through untouched, in aerospace
        # convention: NED reference, FRD body axes.
        self.last_status = now_s(self)
        self.gimbal_reports += 1
        absolute = aerospace_to_ros((msg.q.x, msg.q.y, msg.q.z, msg.q.w))
        if self.gimbal_reference == "earth":
            # The report is absolute, so divide the vehicle attitude off the
            # left: q_abs = q_vehicle * q_rel. Dividing on the right is a
            # conjugation, which looks perfect with the gimbal centered and
            # swaps axes once it moves. See docs/interfaces.md section 6.
            body = quat_mul(quat_conj(self.vehicle_q), absolute)
        else:
            body = absolute
        self.q_status = quat_mul(body, self.gimbal_offset)
        self.status_stamp = self._delayed(msg.header.stamp, self.gimbal_delay)
        self._publish_gimbal()

    def _fresh(self, stamp: float | None, window: float) -> bool:
        return stamp is not None and (now_s(self) - stamp) < window

    def _gimbal_choice(self) -> tuple[str, tuple, object]:
        """The orientation the frame tree gets, its source for the log, and
        the stamp of the message it came from. The stamp travels with the
        value, so a report arriving while the other source is authoritative
        never puts a fresh time on a stale attitude."""
        if self.gimbal_source in ("setpoint", "status"):
            source = self.gimbal_source
        elif self._fresh(self.last_status, STATUS_FRESH_S):
            source = "status"
        elif self._fresh(self.last_setpoint, SETPOINT_TIMEOUT_S):
            source = "setpoint"
        else:
            source = "status"    # nothing fresh: the stale report is the last resort
        if source == "setpoint":
            return source, self.q_setpoint, self.setpoint_stamp
        return source, self.q_status, self.status_stamp

    def _publish_gimbal(self) -> None:
        _, self.gimbal_q, stamp = self._gimbal_choice()

        t = TransformStamped()
        t.header.stamp = stamp or self.get_clock().now().to_msg()
        t.header.frame_id = "gimbal_mount"
        t.child_frame_id = "gimbal_camera_link"
        t.transform.rotation.x = self.gimbal_q[0]
        t.transform.rotation.y = self.gimbal_q[1]
        t.transform.rotation.z = self.gimbal_q[2]
        t.transform.rotation.w = self.gimbal_q[3]
        self.dynamic_broadcaster.sendTransform(t)

    # ------------------------------------------------------------ diagnostics
    def _log_diagnostics(self) -> None:
        vehicle_yaw = quat_yaw_deg(self.vehicle_q)
        absolute_yaw = quat_yaw_deg(quat_mul(self.vehicle_q, self.gimbal_q))
        relative_yaw = quat_yaw_deg(self.gimbal_q)
        self.get_logger().info(
            f"yaw deg: vehicle {vehicle_yaw:+7.1f}  gimbal absolute "
            f"{absolute_yaw:+7.1f}  gimbal rel body {relative_yaw:+7.1f}  "
            f"source={self._gimbal_choice()[0]}")

    def _report(self) -> None:
        if not self.vehicle_seen:
            self.get_logger().warn(
                "no vehicle pose yet, so there is no map -> base_link and "
                "every projection fails. Is mavros connected?")
        if not self.gimbal_seen:
            self.get_logger().warn(
                "no gimbal attitude yet; gimbal_camera_link is aligned with "
                "the airframe. Is the gimbal_control plugin running?")
        self._check_rates()
        self.vehicle_reports = self.gimbal_reports = 0

    def _check_rates(self) -> None:
        """Warn when a chain reports too slowly to cover the pipeline delay.

        A transform is published on arrival and never faked in between, so a
        chain reporting slower than a frame's age cannot answer a lookup at
        the instant that frame was captured, and those detections drop.
        """
        for name, count in (("vehicle pose", self.vehicle_reports),
                            ("gimbal attitude", self.gimbal_reports)):
            rate = count / REPORT_INTERVAL_S
            if count and rate < MIN_REPORT_RATE_HZ:
                self.get_logger().warn(
                    f"{name} arrives at {rate:.1f} Hz, under {MIN_REPORT_RATE_HZ:.0f} Hz. "
                    f"A lookup at a frame's capture time can land past the "
                    f"newest report and drop. Raise the stream rate; for the "
                    f"gimbal see patches/px4-gzgimbal-rate.patch.")


def main() -> None:
    spin(SceneTf)


if __name__ == "__main__":
    main()
