#!/usr/bin/env python3
"""Publish the frame tree for the airframe and its cameras.

MAVROS supplies map -> base_link once tf.send is on, which
`config/mavros_overrides.yaml` does. Everything below base_link is this node's
job, because MAVROS knows nothing about where the payloads sit.

The tree

    map                         local ENU, origin at the launch point
     └── base_link              FLU body frame, from MAVROS
          ├── gimbal_mount      static, where the gimbal bolts on
          │    └── gimbal_camera_link       turns with the gimbal
          │         └── gimbal_camera_optical_frame
          └── nadir_cam_link    static, pitched to look straight down
               └── nadir_camera_optical_frame

Two conventions meet here and they are easy to confuse.

  A camera *link* follows the body convention, x forward along the view axis,
  which is what the Gazebo model uses.
  A camera *optical* frame follows REP 103, x right, y down, z forward, which
  is what CameraInfo and every projection formula assume.

The fixed rotation between them is rpy (-90, 0, -90) degrees. Get it wrong and
detections land at plausible but incorrect positions, which is far harder to
notice than a crash.

The node also draws a simple airframe as markers on base_link, so the 3D view
shows an aircraft rather than a bare set of axes. Markers rather than a URDF,
because a marker needs no mesh files and no robot_description plumbing.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       qos_profile_sensor_data)
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

try:
    from mavros_msgs.msg import GimbalDeviceAttitudeStatus
    HAVE_GIMBAL_MSG = True
except ImportError:  # pragma: no cover - only when mavros_extras is absent
    HAVE_GIMBAL_MSG = False

# GIMBAL_DEVICE_FLAGS_YAW_LOCK
YAW_LOCK = 16

LATCHED = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_mul(a, b):
    """Hamilton product, both as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


# Aerospace to ROS. NED_TO_ENU is a half turn about the (1, 1, 0) diagonal and
# swaps the reference frame; FRD_TO_FLU is a half turn about x and swaps the
# body axes. Applied as NED_TO_ENU * q * FRD_TO_FLU, which is what MAVROS does
# for every other attitude it converts.
NED_TO_ENU = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)


def aerospace_to_ros(q):
    """An absolute attitude, NED reference and FRD body, into ENU and FLU.

    Both ends change, so a rotation is needed on each side.
    """
    return quat_mul(quat_mul(NED_TO_ENU, q), FRD_TO_FLU)


def body_frd_to_flu(q):
    """A rotation *relative to the body*, from FRD axes to FLU axes.

    This is a similarity transform, not the one above. A relative rotation has
    no reference frame to change: the parent is the body in both conventions,
    so only the axis convention differs, and conjugating by a half turn about x
    reduces to negating y and z.

    Using aerospace_to_ros on a relative rotation, or this on an absolute
    attitude, both produce a frame that looks plausible and tracks the aircraft
    incorrectly.
    """
    return (q[0], -q[1], -q[2], q[3])


def quat_to_rpy(q):
    """Roll, pitch, yaw in radians, from (x, y, z, w)."""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def yaw_only(q):
    """The yaw part of a quaternion, as a rotation about z."""
    return quat_from_rpy(0.0, 0.0, math.radians(quat_yaw_deg(q)))


def quat_yaw_deg(q) -> float:
    x, y, z, w = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


# Body convention to REP 103 optical convention.
LINK_TO_OPTICAL = quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)


class SceneTf(Node):
    def __init__(self) -> None:
        super().__init__("scene_tf")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gimbal_mount_xyz", [0.0, 0.0, 0.10])
        self.declare_parameter("nadir_xyz", [0.10, 0.0, -0.06])
        # The nadir mounting, in degrees, roll pitch yaw against base_link.
        #
        # 0 90 0 comes straight from the sensor pose in x500_recon/model.sdf,
        # where a quarter turn about y puts the camera's x axis, which is its
        # view direction, pointing at the ground. With that, the image's up
        # edge is the airframe's nose and its right edge is the airframe's
        # right, so the picture is aligned with the direction of travel.
        #
        # It is a parameter because the alignment is worth being able to trim
        # without a rebuild: a quarter turn wrong here rotates the whole image
        # against the airframe, and a small error shifts where every detection
        # from this camera lands.
        self.declare_parameter("nadir_rpy_deg", [0.0, 90.0, 0.0])
        # Which frame the reported gimbal quaternion is actually in.
        #
        # PX4's simulated gimbal lies about this. In
        # src/modules/simulation/gz_bridge/GZGimbal.cpp, gimbalIMUCallback()
        # builds the quaternion from the gimbal's IMU, and a Gazebo IMU reports
        # orientation relative to the world, not relative to the vehicle. The
        # value is therefore absolute. publishDeviceAttitude() then hardcodes
        #
        #     device_flags = DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME
        #
        # so the message claims to be vehicle-relative while carrying an
        # earth-relative attitude.
        #
        # Obey the flag and the vehicle attitude gets applied twice: yaw the
        # airframe by 90 degrees and the camera appears to swing 180. It looks
        # like a gimbal that over-rotates, and it is why targets land in the
        # wrong place at any heading other than zero. It affects roll and pitch
        # as well, and only yaw is obvious because aircraft fly near level.
        #
        # "earth" is therefore the default and matches what PX4 sends. This
        # node divides the vehicle attitude back out, so the transform it
        # publishes under gimbal_mount is genuinely vehicle-relative.
        # Set "vehicle" if you run a gimbal that reports honestly, or if you
        # applied patches/px4-gzgimbal-frame.patch to PX4.
        self.declare_parameter("gimbal_reference", "earth")
        # Where the gimbal orientation comes from.
        #
        #   setpoint  GIMBAL_DEVICE_SET_ATTITUDE, the angles the gimbal manager
        #             commands. PX4 builds this in output_mavlink.cpp from
        #             _q_setpoint and tags each axis with a LOCK flag: unlocked
        #             means the angle is relative to the airframe, locked means
        #             it is relative to the earth. Those semantics are the
        #             standard ones and PX4 honours them, which is why
        #             `gimbal test pitch` behaves correctly against the drone.
        #             This is the default.
        #   status    GIMBAL_DEVICE_ATTITUDE_STATUS, what the device reports.
        #             In simulation that value is absolute while claiming to be
        #             vehicle relative, so it needs the vehicle attitude divided
        #             back out and is easy to get wrong.
        #
        #   auto      prefer the setpoint, fall back to the status. This is
        #             the default, and it is the default because PX4 only
        #             publishes a setpoint once something has commanded the
        #             gimbal. An untouched gimbal produces no setpoint at all,
        #             so a stack pinned to "setpoint" would sit at identity
        #             until the first command and then jump.
        #
        # The setpoint is a command rather than a measurement, so it leads the
        # real gimbal slightly and shows no stall or slew limit. For a simulated
        # gimbal that tracks its command exactly, it is the better source.
        self.declare_parameter("gimbal_source", "auto")
        # How long a setpoint stays authoritative after it arrives.
        self.declare_parameter("setpoint_timeout", 3.0)
        # MAVROS republishes the gimbal attitude untouched, with frame_id
        # base_link_frd. That value is aerospace convention on both ends: the
        # body axes are forward-right-down and the reference is NED. Every ROS
        # frame here is forward-left-up against ENU.
        #
        # Both ends change, so this is not a single 180 degree turn. Negating y
        # and z converts the body axes alone and leaves the reference wrong,
        # which measures as the camera turning about twice as far as the
        # aircraft and in the opposite direction. The conversion needs a
        # rotation on each side, which is what NED_TO_ENU and FRD_TO_FLU do.
        self.declare_parameter("gimbal_is_frd", True)
        # A fixed rotation applied to the gimbal attitude after it has been
        # made body relative, in degrees, as roll, pitch, yaw.
        #
        # Everything upstream of this is now pinned down:
        #
        #   PX4  src/modules/simulation/gz_bridge/GZGimbal.cpp reads the gimbal
        #        IMU, which Gazebo reports against the world, converts it with
        #        q_ENU_to_NED * q * q_FLU_to_FRD^-1, and then labels the result
        #        DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME even though it is absolute.
        #   MAVROS  gimbal_control.cpp calls mavlink_to_quaternion and nothing
        #        else, so the quaternion arrives exactly as PX4 sent it. Its
        #        frame_id, base_link_frd, is honest.
        #   Model  in the gimbal SDF the camera_imu and the camera sensor carry
        #        the same rotation, 0 0 3.14, and differ only in x. So the IMU
        #        measures the camera's own frame and no extra rotation is due
        #        between them.
        #
        # This node undoes each of those in turn. If a constant offset still
        # remains, it belongs here rather than buried in the maths, because a
        # constant is a mounting convention and not a bug in the conversion.
        # The diagnostic below prints the number to put in it.
        self.declare_parameter("gimbal_offset_rpy_deg", [0.0, 0.0, 0.0])
        # Which side the vehicle attitude is divided off. See _store.
        self.declare_parameter("gimbal_compose", "left")
        # Log the vehicle heading against the gimbal heading once a second, so
        # a constant offset is readable rather than inferred.
        self.declare_parameter("log_gimbal_diagnostics", True)
        self.declare_parameter("gimbal_rate_hz", 30.0)
        self.declare_parameter("publish_markers", True)

        self.base = self.get_parameter("base_frame").value
        self.gimbal_reference = self.get_parameter("gimbal_reference").value
        self.gimbal_is_frd = bool(self.get_parameter("gimbal_is_frd").value)
        off = [float(v) for v in self.get_parameter("gimbal_offset_rpy_deg").value]
        self.gimbal_offset = quat_from_rpy(*(math.radians(v) for v in off))
        self.gimbal_abs = (0.0, 0.0, 0.0, 1.0)
        self.gimbal_source = self.get_parameter("gimbal_source").value
        self.compose = self.get_parameter("gimbal_compose").value
        self.setpoint_timeout = float(self.get_parameter("setpoint_timeout").value)
        self.logged_lock = False
        self.last_setpoint = None
        # Both derivations are published all the time, side by side, so they can
        # be compared instead of argued about. gimbal_camera_link follows
        # gimbal_source; the other two are always what their own source says.
        #
        #   gimbal_status_camera_link    from GIMBAL_DEVICE_ATTITUDE_STATUS,
        #                                absolute, with the vehicle attitude
        #                                divided back out.
        #   gimbal_setpoint_camera_link  from GIMBAL_DEVICE_SET_ATTITUDE,
        #                                already relative to the airframe.
        #
        # When the two agree, the frame handling is right and either source can
        # be trusted. When they diverge, the difference is the bug, and the
        # diagnostic below prints it in degrees.
        self.q_status = (0.0, 0.0, 0.0, 1.0)
        self.q_setpoint = (0.0, 0.0, 0.0, 1.0)
        self.have_setpoint = False

        self.static_bc = StaticTransformBroadcaster(self)
        self.dyn_bc = TransformBroadcaster(self)
        self._publish_static()

        # Identity until the gimbal reports. A gimbal that never reports leaves
        # the camera pointing along the airframe, which is visibly wrong in the
        # 3D view rather than silently wrong in the numbers.
        self.gimbal_q = (0.0, 0.0, 0.0, 1.0)
        self.gimbal_seen = False
        self.gimbal_from_setpoint = False
        self.last_status = None
        # The vehicle attitude in map, needed to divide out of an earth
        # referenced gimbal report.
        self.vehicle_q = (0.0, 0.0, 0.0, 1.0)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_vehicle, qos_profile_sensor_data)

        if HAVE_GIMBAL_MSG:
            self.create_subscription(
                GimbalDeviceAttitudeStatus,
                "/mavros/gimbal_control/device/attitude_status",
                self._on_gimbal, 10)
            # The commanded attitude. This is the default source, and also the
            # fallback when gimbal_source is "status" and the device is silent.
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
                "mavros_msgs has no GimbalDeviceAttitudeStatus. The gimbal frame "
                "will stay fixed to the airframe.")

        rate = float(self.get_parameter("gimbal_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._publish_gimbal)

        if self.get_parameter("publish_markers").value:
            self.marker_pub = self.create_publisher(
                MarkerArray, "/drone/markers", LATCHED)
            self.create_timer(1.0, self._publish_markers)

        self.create_timer(30.0, self._report)
        if self.get_parameter("log_gimbal_diagnostics").value:
            self.create_timer(1.0, self._diagnostics)

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
        identity = (0.0, 0.0, 0.0, 1.0)

        self.static_bc.sendTransform([
            self._static(self.base, "gimbal_mount", mount, identity),
            self._static("gimbal_camera_link", "gimbal_camera_optical_frame",
                         (0, 0, 0), LINK_TO_OPTICAL),
            self._static("gimbal_status_camera_link",
                         "gimbal_status_camera_optical_frame",
                         (0, 0, 0), LINK_TO_OPTICAL),
            # Pitched a quarter turn so the link's x axis looks straight down,
            # matching the sensor pose in x500_recon/model.sdf.
            self._static(self.base, "nadir_cam_link", nadir,
                         quat_from_rpy(*(math.radians(v) for v in
                                         self.get_parameter("nadir_rpy_deg").value))),
            self._static("nadir_cam_link", "nadir_camera_optical_frame",
                         (0, 0, 0), LINK_TO_OPTICAL),
        ])

    # ----------------------------------------------------------------- gimbal
    def _on_vehicle(self, msg) -> None:
        q = msg.pose.orientation
        self.vehicle_q = (q.x, q.y, q.z, q.w)

    def _on_gimbal_setpoint(self, msg) -> None:
        if self.gimbal_source == "status":
            return          # the device report is the source; ignore commands

        self.last_setpoint = self.get_clock().now().nanoseconds / 1e9

        # The setpoint is already relative to the airframe on any axis whose
        # LOCK flag is clear, which is the normal case and what `gimbal test`
        # produces. So it needs the body axis conversion and nothing else: no
        # vehicle attitude to divide out, and no chance of counting it twice.
        flags = int(getattr(msg, "flags", 0))
        body = body_frd_to_flu((msg.q.x, msg.q.y, msg.q.z, msg.q.w))

        if flags & YAW_LOCK:
            # Yaw is earth referenced on this axis, so take the vehicle heading
            # back out to get a rotation that belongs under gimbal_mount.
            body = quat_mul(quat_conj(yaw_only(self.vehicle_q)), body)
            if not self.logged_lock:
                self.logged_lock = True
                self.get_logger().info(
                    "gimbal yaw is earth locked, removing the vehicle heading")

        if not self.gimbal_seen or self.gimbal_from_setpoint is False:
            self.gimbal_seen = True
            self.gimbal_from_setpoint = True
            self.get_logger().info(
                f"gimbal orientation from the commanded setpoint, flags={flags}"
                f"{' (yaw locked)' if flags & YAW_LOCK else ' (vehicle relative)'}")

        self.q_setpoint = quat_mul(body, self.gimbal_offset)
        self.have_setpoint = True

    def _store(self, q) -> None:
        raw = (q.x, q.y, q.z, q.w)
        if self.gimbal_is_frd:
            # Undo PX4's ENU to NED and FLU to FRD conversion. MAVROS passes the
            # quaternion through untouched, so this is the only place it happens.
            raw = aerospace_to_ros(raw)
        self.gimbal_abs = raw
        if self.gimbal_reference == "earth":
            # The report is absolute, so the vehicle attitude has to come back
            # out. Which side it comes off is the whole question.
            #
            #   left   conj(q_vehicle) * q_gimbal   the usual reading, where the
            #          gimbal attitude is the vehicle attitude followed by the
            #          gimbal's own rotation.
            #   right  q_gimbal * conj(q_vehicle)   correct instead when the two
            #          are composed the other way round.
            #
            # Both remove the vehicle attitude, and they differ by exactly the
            # frame the leftover rotation is expressed in. Picking wrong leaves
            # a residual that turns with the aircraft on every axis, which is
            # the symptom reported here: not a bad axis, a bad order.
            #
            # left is the correct one, and this is not a preference.
            #
            # An absolute attitude is the vehicle attitude followed by the
            # gimbal's own rotation, q_abs = q_vehicle * q_rel, so recovering
            # q_rel means dividing on the left. Dividing on the right gives
            # q_vehicle * q_rel * conj(q_vehicle), which is a conjugation: the
            # same rotation by the same angle, but about an axis rotated by the
            # vehicle heading.
            #
            # That has a signature worth recognising, because it fooled this
            # code once. Conjugation leaves identity alone, so a centred gimbal
            # looks perfect at every aircraft heading. It only goes wrong once
            # the gimbal moves off centre, and then it does not look like a
            # rotation error, it looks like the axes swapped: with the aircraft
            # at 90 degrees of yaw, a 30 degree gimbal pitch comes out as a 30
            # degree roll.
            #
            # "Correct at zero, wrong off zero" therefore means conjugation, not
            # a bad axis and not a missing offset.
            #
            # One caveat when judging this from the aircraft: a gimbal holding
            # an ROI is earth locked, so its angle relative to the airframe
            # genuinely changes as the aircraft yaws. That is correct behaviour
            # and reads exactly like the fault. Centre the gimbal before
            # deciding, or command it in vehicle relative mode.
            body = (quat_mul(raw, quat_conj(self.vehicle_q))
                    if self.compose == "right"
                    else quat_mul(quat_conj(self.vehicle_q), raw))
        else:
            body = raw
        # Any remaining fixed mounting rotation.
        self.q_status = quat_mul(body, self.gimbal_offset)

    def _on_gimbal(self, msg) -> None:
        self.last_status = self.get_clock().now().nanoseconds / 1e9
        # Always store it: gimbal_status_camera_link is published whatever
        # gimbal_source says, so the two can be compared.
        self._store(msg.q)

    def _setpoint_fresh(self) -> bool:
        if self.last_setpoint is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self.last_setpoint) < self.setpoint_timeout
        if not self.gimbal_seen:
            self.gimbal_seen = True
            # flags bit 32 is YAW_IN_VEHICLE_FRAME, 64 is YAW_IN_EARTH_FRAME.
            frame = "vehicle" if (getattr(msg, "flags", 0) & 32) else \
                    ("earth" if (getattr(msg, "flags", 0) & 64) else "unstated")
            self.get_logger().info(
                f"gimbal attitude received: flags={getattr(msg, 'flags', 0)} "
                f"({frame} yaw claimed), treating input as "
                f"{'FRD' if self.gimbal_is_frd else 'FLU'} and as "
                f"{self.gimbal_reference} referenced")
            if frame == "vehicle" and self.gimbal_reference == "earth":
                self.get_logger().info(
                    "The flag says vehicle frame and this node is ignoring it. "
                    "PX4's simulated gimbal sets that flag while reporting an "
                    "absolute attitude. See the note in scene_tf.py.")

    def _primary(self):
        """The rotation gimbal_camera_link follows."""
        if self.gimbal_source == "setpoint":
            return self.q_setpoint
        if self.gimbal_source == "auto" and self._setpoint_fresh():
            return self.q_setpoint
        return self.q_status

    def _publish_gimbal(self) -> None:
        # Always under gimbal_mount. An earth referenced report has already had
        # the vehicle attitude divided out in _store, so by this point the
        # rotation is vehicle relative either way.
        stamp = self.get_clock().now().to_msg()
        self.gimbal_q = self._primary()
        self.gimbal_abs = quat_mul(self.vehicle_q, self.gimbal_q)

        out = []
        # The primary frame, and the raw status derivation beside it. The
        # second is cheap and makes a future disagreement visible instead of
        # silent.
        for child, q in (("gimbal_camera_link", self.gimbal_q),
                         ("gimbal_status_camera_link", self.q_status)):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "gimbal_mount"
            t.child_frame_id = child
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            out.append(t)
        self.dyn_bc.sendTransform(out)

    # ---------------------------------------------------------------- markers
    def _publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        def add(idx, kind, xyz, scale, colour, frame=None, rpy=(0, 0, 0)):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = frame or self.base
            m.ns = "airframe"
            m.id = idx
            m.type = kind
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
            q = quat_from_rpy(*rpy)
            m.pose.orientation.x, m.pose.orientation.y = q[0], q[1]
            m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color = ColorRGBA(r=colour[0], g=colour[1], b=colour[2], a=colour[3])
            arr.markers.append(m)

        body = (0.15, 0.15, 0.17, 1.0)
        add(0, Marker.CUBE, (0.0, 0.0, 0.0), (0.22, 0.16, 0.08), body)
        # Four arms and rotor discs, at the x500 geometry.
        for i, (ax, ay) in enumerate([(0.18, 0.18), (-0.18, 0.18),
                                      (-0.18, -0.18), (0.18, -0.18)]):
            add(10 + i, Marker.CUBE, (ax / 2, ay / 2, 0.0),
                (abs(ax) + 0.02, 0.02, 0.02), (0.1, 0.1, 0.1, 1.0),
                rpy=(0.0, 0.0, math.atan2(ay, ax)))
            # Front rotors red, rear ones dark, the usual orientation cue.
            colour = (0.85, 0.2, 0.2, 0.55) if ax > 0 else (0.2, 0.2, 0.25, 0.55)
            add(20 + i, Marker.CYLINDER, (ax, ay, 0.03), (0.26, 0.26, 0.01), colour)
        add(30, Marker.SPHERE, tuple(self.get_parameter("gimbal_mount_xyz").value),
            (0.07, 0.07, 0.07), (0.9, 0.75, 0.1, 1.0))
        self.marker_pub.publish(arr)

    def _diagnostics(self) -> None:
        """Print the three headings that matter, so an offset is readable.

        With the gimbal centred and untouched, `gimbal rel body` should read
        about zero at every aircraft heading. A constant elsewhere is the
        number for gimbal_offset_rpy_deg. A value that moves with the aircraft
        means the frame handling is still wrong, not the mounting.
        """
        v = quat_yaw_deg(self.vehicle_q)
        a = quat_yaw_deg(self.gimbal_abs)
        rel = quat_yaw_deg(self.gimbal_q)
        self.get_logger().info(
            f"yaw deg: vehicle {v:+7.1f}  gimbal absolute {a:+7.1f}  "
            f"gimbal rel body {rel:+7.1f}  (absolute minus vehicle "
            f"{((a - v + 540) % 360) - 180:+7.1f})  source="
            f"{'setpoint' if self._setpoint_fresh() else 'status'}")

    def _report(self) -> None:
        if not self.gimbal_seen:
            self.get_logger().warn(
                "no gimbal attitude yet; gimbal_camera_link is aligned with the "
                "airframe. Is the gimbal_control plugin running?")


def main() -> None:
    rclpy.init()
    node = SceneTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
