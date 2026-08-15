#!/usr/bin/env python3
"""Point the gimbal at a pixel clicked in Foxglove.

Foxglove publishes the clicked pixel as a PointStamped, x and y in image
coordinates. This node casts a ray through that pixel with the gimbal
camera intrinsics, rotates the ray into the reference frame with the live
TF, and commands the gimbal to put its optical axis on the ray. The command
is a full three axis attitude: roll is zero so the image stays level, and
pitch and yaw point the axis, computed earth referenced in that frame.

The command leaves as the MAVLink GIMBAL_MANAGER_SET_ATTITUDE message. The
node publishes a mavros_msgs/GimbalManagerSetAttitude, and the mavros
gimbal_control plugin sends it to the autopilot.

The flags and the yaw offset below are calibrated against PX4's
simulated gimbal, which does not obey the MAVLink frame conventions.
GZGimbal.cpp (pollSetpoint) maps the setpoint's Euler angles straight
onto the CGO3 joints and ignores the frame flags, and that joint chain
runs through a yaw joint on -z and two 180 degree mounts
(gimbal/model.sdf, x500_recon/model.sdf). Measured against the frame
tree scene_tf builds from the device report, with the vehicle at rest
and the lock flags clear: the achieved pitch equals the commanded pitch,
and the achieved compass yaw is the commanded yaw plus 90 degrees, on
six probe attitudes to within 0.1 degrees. So the node sends the lock
flags clear, which stops PX4's gimbal module from adding its own earth
to body conversion on top, and subtracts the 90 degrees from the
commanded yaw. One heading was tried, so a heading term inside that
offset cannot be ruled out. The desired attitude comes from the live TF,
so repeated clicks converge on the target either way. For a gimbal that
obeys the MAVLink convention, set the gimbal_convention parameter to
"mavlink": the three lock flags, no offset. docs/interfaces.md section 6
records the sibling bug in the reported attitude.

PX4 ignores the setpoint unless its sender holds primary gimbal control
(input_mavlink.cpp, _process_set_attitude), and it drops it silently. The
node therefore claims control with DO_GIMBAL_MANAGER_CONFIGURE at startup,
and claims again when a click arrives while someone else, usually QGC,
holds control. It never wrestles control back on a timer: outside a click
there is no user intent to act on. The claim goes through the generic
/mavros/cmd/command service, not the gimbal_control configure service.
The configure handler blocks its plugin's executor until the command is
acknowledged, so one lost acknowledgment stops that plugin from sending
any gimbal message at all. The command plugin times out and recovers.
A claim whose response never arrives is treated as a stuck gimbal, and
the recovery is the center action QGC offers: command pitch and yaw
zero, straight ahead. That command also hands primary control to its
sender, so it doubles as the claim.

Subscribes
    <click_topic>        geometry_msgs/PointStamped, pixels on the image
    <camera_info_topic>  sensor_msgs/CameraInfo
    /mavros/gimbal_control/manager/status   who holds control

Publishes
    /mavros/gimbal_control/manager/set_attitude
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import GimbalManagerSetAttitude, GimbalManagerStatus
from mavros_msgs.srv import CommandLong
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener

from sim_bridge.projection import (intrinsics_ready, pointing_rpy_ned,
                                   quat_from_rpy, quat_rotate, ray_in_optical)

# ------------------------------------------------------------------- tunables
# How often the startup claim retries until the autopilot accepts one.
CLAIM_RETRY_S = 5.0
# A claim whose response never arrives is treated as a stuck gimbal after
# this long. The recovery is the one QGC uses: command the gimbal to
# center, pitch and yaw zero, straight ahead. That command also hands
# primary control to its sender, so it doubles as the claim.
CLAIM_RECOVER_S = 10.0
# How long a click waits for the transform before it is dropped.
TF_TIMEOUT_S = 0.2

MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000
MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001

# The ids mavros stamps on outgoing MAVLink, so the ids that must hold
# primary control for PX4 to accept the setpoint. mavros defaults, not set
# anywhere in this stack. The manager status subscription warns when the
# ids on the wire disagree with these.
MAVROS_SYSID = 1
MAVROS_COMPID = 191
# The autopilot, matching target_system_id in stack.launch.py.
TARGET_SYSTEM = 1
TARGET_COMPONENT = 1

# The gimbal_convention "gz_sim" calibration. See the module docstring.
GZ_SIM_YAW_OFFSET_DEG = -90.0

EARTH_LOCK_FLAGS = (
    GimbalManagerSetAttitude.GIMBAL_MANAGER_FLAGS_ROLL_LOCK
    | GimbalManagerSetAttitude.GIMBAL_MANAGER_FLAGS_PITCH_LOCK
    | GimbalManagerSetAttitude.GIMBAL_MANAGER_FLAGS_YAW_LOCK)


class ClickToGimbal(Node):
    def __init__(self) -> None:
        super().__init__("click_to_gimbal")

        self.declare_parameter("click_topic", "/foxglove/cursor/click")
        self.declare_parameter("camera_info_topic", "/camera/gimbal/camera_info")
        self.declare_parameter("optical_frame", "gimbal_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")
        # Which command convention the gimbal obeys.
        #   gz_sim    PX4's simulated gimbal: lock flags clear, yaw offset
        #             -90 degrees. The calibration in the module docstring.
        #   mavlink   a gimbal that reads the flags honestly: the three lock
        #             flags, no offset.
        # The sibling knob for the reported attitude is scene_tf's
        # gimbal_reference.
        self.declare_parameter("gimbal_convention", "gz_sim")

        self.optical = self.get_parameter("optical_frame").value
        self.reference = self.get_parameter("reference_frame").value
        honest = self.get_parameter("gimbal_convention").value == "mavlink"
        self.cmd_yaw_offset = (0.0 if honest
                               else math.radians(GZ_SIM_YAW_OFFSET_DEG))
        self.setpoint_flags = EARTH_LOCK_FLAGS if honest else 0

        self.tf_buffer = Buffer()
        # spin_thread=True is required. On this node's executor, a lookup that
        # waits for a transform would block the callback that delivers it.
        self.tf_listener = TransformListener(self.tf_buffer, self,
                                             spin_thread=True)

        self.info: CameraInfo | None = None
        # None until the first manager status arrives. True while these
        # MAVROS_SYSID and MAVROS_COMPID ids hold primary control.
        self.in_control: bool | None = None
        self.claim_acked = False
        self.claim_inflight = False

        # Sensor QoS on the mavros topics: they are best effort, and a
        # reliable subscription to a best effort publisher receives nothing.
        self.create_subscription(CameraInfo,
                                 self.get_parameter("camera_info_topic").value,
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(GimbalManagerStatus,
                                 "/mavros/gimbal_control/manager/status",
                                 self._on_status, qos_profile_sensor_data)
        # Foxglove publishes clicks reliable, the default.
        self.create_subscription(PointStamped,
                                 self.get_parameter("click_topic").value,
                                 self._on_click, 10)

        self.setpoint_pub = self.create_publisher(
            GimbalManagerSetAttitude,
            "/mavros/gimbal_control/manager/set_attitude", 10)
        self.claim_client = self.create_client(CommandLong, "/mavros/cmd/command")
        self.claim_sent_at = 0.0
        self.claim_future = None

        self.create_timer(CLAIM_RETRY_S, self._claim_tick)
        # The timer first fires a full period from now. Against an already
        # running mavros the service is ready at once, so try now and close
        # the window where an early click is silently dropped.
        self._claim_tick()

    # ------------------------------------------------------------------ inputs
    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_status(self, msg: GimbalManagerStatus) -> None:
        ours = (msg.sysid_primary == MAVROS_SYSID
                and msg.compid_primary == MAVROS_COMPID)
        if ours != self.in_control:
            if ours:
                self.get_logger().info("gimbal control is ours")
            else:
                self.get_logger().warn(
                    f"gimbal control is held by "
                    f"{msg.sysid_primary}/{msg.compid_primary}, not "
                    f"{MAVROS_SYSID}/{MAVROS_COMPID}. A click claims it back.")
        self.in_control = ours

    # ------------------------------------------------------------------- claim
    def _claim_tick(self) -> None:
        if self.claim_inflight:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.claim_sent_at > CLAIM_RECOVER_S:
                self.claim_inflight = False
                self.get_logger().warn(
                    "gimbal control claim got no response, centering the "
                    "gimbal to recover it")
                self._center()
            return
        if self.claim_acked:
            return
        self._claim()

    def _claim(self) -> None:
        request = CommandLong.Request()
        request.command = MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
        request.param1 = float(MAVROS_SYSID)
        request.param2 = float(MAVROS_COMPID)
        # Leave secondary control unchanged. param7 addresses every gimbal.
        request.param3 = -1.0
        request.param4 = -1.0
        request.param7 = 0.0
        self._send_command(request)

    def _center(self) -> None:
        # Pitch zero, yaw zero, straight ahead. Rates NaN, no rate setpoint.
        # Flags zero, vehicle relative. The autopilot hands primary control
        # to the sender of this command.
        request = CommandLong.Request()
        request.command = MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        request.param1 = 0.0
        request.param2 = 0.0
        request.param3 = math.nan
        request.param4 = math.nan
        request.param5 = 0.0
        request.param7 = 0.0
        self._send_command(request)

    def _send_command(self, request) -> None:
        if self.claim_inflight or not self.claim_client.service_is_ready():
            return
        self.claim_inflight = True
        self.claim_sent_at = self.get_clock().now().nanoseconds / 1e9
        self.claim_future = self.claim_client.call_async(request)
        self.claim_future.add_done_callback(self._on_claim_done)

    def _on_claim_done(self, future) -> None:
        if future is not self.claim_future:
            # A response from a command that was already given up on. The
            # recovery has moved on, so this answer decides nothing.
            return
        self.claim_inflight = False
        try:
            response = future.result()
        except Exception as err:  # noqa: BLE001 - a dead service is expected early
            self.get_logger().warn(f"gimbal control claim failed: {err}")
            return
        if response.success:
            if not self.claim_acked:
                self.get_logger().info(
                    f"gimbal control claimed for {MAVROS_SYSID}/{MAVROS_COMPID}")
            self.claim_acked = True
        else:
            self.get_logger().warn(
                f"gimbal control claim rejected, result={response.result}")

    # ------------------------------------------------------------------- click
    def _on_click(self, msg: PointStamped) -> None:
        if not intrinsics_ready(self.info):
            self.get_logger().warn("click ignored: no camera intrinsics yet")
            return
        u, v = msg.point.x, msg.point.y
        if not (0.0 <= u < self.info.width and 0.0 <= v < self.info.height):
            self.get_logger().warn(
                f"click ignored: pixel ({u:.0f}, {v:.0f}) is outside the "
                f"{self.info.width}x{self.info.height} image")
            return
        try:
            # Latest available rather than the click stamp: the command
            # steers from where the camera looks now.
            tf = self.tf_buffer.lookup_transform(
                self.reference, self.optical, rclpy.time.Time(),
                timeout=Duration(seconds=TF_TIMEOUT_S))
        except Exception as err:  # noqa: BLE001 - lookup raises several types
            self.get_logger().warn(
                f"click dropped: no transform {self.reference} -> "
                f"{self.optical} ({err})")
            return

        r = tf.transform.rotation
        direction = quat_rotate((r.x, r.y, r.z, r.w),
                                ray_in_optical(u, v, self.info.k))
        roll, pitch, yaw = pointing_rpy_ned(direction)

        if self.in_control is False:
            # The setpoint races the claim to the autopilot, and a setpoint
            # that arrives first is dropped. The click still goes out: at
            # worst this click restores control and the next one lands.
            self._claim()
            self.get_logger().warn(
                "another party held gimbal control at click time. If the "
                "gimbal does not move, click again.")
        self._publish_setpoint(quat_from_rpy(
            roll, pitch, yaw + self.cmd_yaw_offset))
        self.get_logger().info(
            f"click ({u:.0f}, {v:.0f}) -> gimbal pitch "
            f"{math.degrees(pitch):+.1f} deg, yaw {math.degrees(yaw):+.1f} deg, "
            f"earth referenced")

    def _publish_setpoint(self, q) -> None:
        cmd = GimbalManagerSetAttitude()
        cmd.target_system = TARGET_SYSTEM
        cmd.target_component = TARGET_COMPONENT
        cmd.flags = self.setpoint_flags
        cmd.gimbal_device_id = 0
        cmd.q.x, cmd.q.y, cmd.q.z, cmd.q.w = q
        # NaN: an angle setpoint with no rate setpoint.
        cmd.angular_velocity_x = math.nan
        cmd.angular_velocity_y = math.nan
        cmd.angular_velocity_z = math.nan
        self.setpoint_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = ClickToGimbal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
