#!/usr/bin/env python3
"""Turn clicks on the gimbal image into gimbal action.

Foxglove publishes the clicked pixel as a PointStamped, x and y in image
coordinates. The click_mode parameter decides what one click does, and
because it is one parameter the modes cannot overlap:

    roi     hold the camera on the ground point under the pixel. Default.
    point   put the camera axis on the pixel: pitch and roll hold against
            the horizon, and yaw follows the vehicle heading.
    off     ignore clicks.

Foxglove can flood the click topic with cursor events, and each
processed click runs a TF lookup that can block for TF_TIMEOUT_S. A
trailing-edge guard therefore processes at most one click each
CLICK_MIN_INTERVAL_S: a click inside the interval waits as the pending
click, newest wins, and a one-shot timer lands it when the interval
ends. No final click is dropped, and a same-pixel re-click lands like
any other, because it is the user's recovery action.

Both click behaviors are the gimbal protocol's stabilized ones. An ROI
holds every axis on the point. A point click uses the protocol's
default lock flags: roll and pitch locked to the horizon, yaw
unlocked, so the view turns with the aircraft. A hold persists across
mode changes and through off mode: the modes only decide what a new
click does. A new click replaces the hold, and /gimbal/center releases
it.

The mode changes at runtime with `ros2 param set` or the Foxglove
Parameters panel, and the current mode is published latched on
/gimbal/click_mode for the layout's indicator. Three Trigger services,
/gimbal/click_mode/roi, /gimbal/click_mode/point and
/gimbal/click_mode/off, set the mode with an empty request, so a
Foxglove service-call button needs no payload. Each goes through the
parameter, which stays the single source of truth. A fourth service,
/gimbal/center, releases any hold and points the gimbal straight
ahead, pitch and yaw zero, the same center command the recovery path
uses. The mode does not change.

In roi mode the node casts the pixel ray with the camera intrinsics,
meets the ground plane, and records the region of interest: latched
NavSatFix on /gimbal/roi, altitude in the datum of the vehicle's own
fix, and a latched reference-frame PointStamped on /gimbal/roi_local.
The ground plane sits rel_alt below the camera, or at ground_z with
use_rel_alt false, the same convention the ground projector uses. A ray
that meets no ground within the footprint limit drops the click with a
warning. What happens next depends on the gimbal convention. With
"gz_sim", roi_tracker.py holds the camera on the point, because PX4
cannot: its v2 gimbal output computes the ROI attitude once per command,
and the simulated gimbal ignores the frame flags. With "mavlink" the
node sends DO_SET_ROI_LOCATION and the autopilot does the holding, the
same path QGC uses, and a point click on top of an ROI sends
DO_SET_ROI_NONE first.

In point mode the command leaves as the MAVLink
GIMBAL_MANAGER_SET_ATTITUDE message through the mavros gimbal_control
plugin: the clicked pitch against the horizon, the clicked yaw as an
offset from the vehicle heading, roll zero. An honest gimbal
stabilizes it because the flags say so, and roi_tracker.py does the
same for the simulated gimbal.

How a command is expressed depends on which convention the gimbal
obeys. A gimbal that obeys MAVLink reads the lock flags: the node
sends the roll and pitch lock flags with yaw unlocked, so the
quaternion carries a horizon-referenced pitch and a heading-relative
yaw, and the device does the stabilizing. That is the "mavlink"
convention.

PX4's simulated gimbal does not read the flags. PX4's gimbal module
passes the setpoint quaternion through unchanged (output_mavlink.cpp,
OutputMavlinkV2). GZGimbal.cpp (pollSetpoint) converts it to Euler
angles and writes them onto the CGO3 joint position controllers, and
the joints ride on the airframe. The joint chain is the 180 degree
mount in x500_recon/model.sdf, the yaw joint on -z and the pitch joint
on +y in gimbal/model.sdf, and the camera sensor yawed 180 degrees
inside its link. That chain realizes the commanded Euler angles
exactly, vehicle relative, in the aerospace sign convention, verified
against Gazebo link poses to 0.1 degrees. The device is a follow mode
gimbal on every axis, whatever the flags say. So the "gz_sim"
convention sends a vehicle-relative attitude with the lock flags
clear, which also labels the setpoint honestly for scene_tf.

The world direction of a click comes from the full map to optical TF
chain, which is world true. The gimbal segment of that tree alone
carries the EKF heading error, measured at 16 degrees in flight,
because it divides the device report, which Gazebo builds from ground
truth, by the EKF attitude. Composing the vehicle attitude back on
cancels the error exactly, so the full chain is safe and the
vehicle-relative half is not. roi_tracker.py therefore converts world
targets back to vehicle-relative joints with a corrected vehicle
attitude, built from that chain and the joint state in
cmd_q_body_link. A gimbal moved by another controller, usually the
QGC joystick, leaves that state stale. Each click therefore compares
the joint state the TF chain implies, under the standing correction,
against cmd_q_body_link, and adopts the TF answer when they disagree:
the disagreement belongs to the joints, because the correction only
drifts at EKF speed. A click from any gimbal pose then lands without
an initial offset. GIMBAL_DEVICE_SET_ATTITUDE would carry the joint
setpoint directly, but mavros does not translate it, so the TF chain
is the one live source.

Two earlier wrong answers are worth recording. A calibration at rest
measured a constant 90 degree yaw error and subtracted it as a mount
offset, but the vehicle spawns facing east, compass 90 degrees, so the
constant was the vehicle heading. A later version read the ray from
the TF vehicle segment and inherited the EKF heading error above.
docs/interfaces.md section 6 records the sibling bug in the reported
attitude.

PX4 ignores the setpoint unless its sender holds primary gimbal control
(input_mavlink.cpp, _process_set_attitude), and it drops it silently. The
node therefore claims control with DO_GIMBAL_MANAGER_CONFIGURE at startup,
and claims again when a command goes out while someone else, usually QGC,
holds control. It never wrestles control back on a timer: outside a click
there is no user intent to act on. The claim goes through the generic
/mavros/cmd/command service, not the gimbal_control configure service.
The configure handler blocks its plugin's executor until the command is
acknowledged, so one lost acknowledgment stops that plugin from sending
any gimbal message at all. The command plugin times out and recovers.
A claim whose response never arrives is treated as a stuck gimbal, and
the recovery is the center action QGC offers: command pitch and yaw
zero, straight ahead. That command also hands primary control to its
sender, so it doubles as the claim. ROI location commands go to the
navigator instead and need no gimbal control at all.

Subscribes
    <click_topic>        geometry_msgs/PointStamped, pixels on the image
    <camera_info_topic>  sensor_msgs/CameraInfo
    /mavros/gimbal_control/manager/status   who holds control
    /mavros/local_position/pose             the vehicle attitude
    /mavros/global_position/rel_alt         the ground plane height
    /mavros/global_position/global          the ROI altitude datum
    /mavros/altitude                        AMSL for DO_SET_ROI_LOCATION,
                                            mavlink convention only

Publishes
    /mavros/gimbal_control/manager/set_attitude
    /gimbal/click_mode   std_msgs/String, latched
    /gimbal/roi          sensor_msgs/NavSatFix, latched
    /gimbal/roi_local    geometry_msgs/PointStamped, latched

Serves
    /gimbal/click_mode/{roi,point,off}   std_srvs/Trigger
    /gimbal/center                       std_srvs/Trigger
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import Altitude, GimbalManagerSetAttitude, GimbalManagerStatus
from mavros_msgs.srv import CommandInt, CommandLong
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from sim_bridge.geo import MapOrigin
from sim_bridge.projection import (GROUND_VIEW_MAX_DISTANCE_M,
                                   body_frd_to_flu, intersect_ground,
                                   intrinsics_ready, pointing_rpy_ned,
                                   quat_from_rpy, quat_rotate, ray_in_optical,
                                   ros_to_aerospace, rpy_from_quat, wrap_pi)
from sim_bridge.roi_tracker import RoiTracker

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
# The shortest interval between processed clicks, against a Foxglove
# cursor-event flood. A click inside the interval waits as the pending
# click, newest wins, and a one-shot timer lands it when the interval
# ends, so no final click is ever dropped.
CLICK_MIN_INTERVAL_S = 0.15

CLICK_MODES = ("roi", "point", "off")

MAV_CMD_DO_SET_ROI_LOCATION = 195
MAV_CMD_DO_SET_ROI_NONE = 197
MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000
MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001
MAV_FRAME_GLOBAL_INT = 5

# The ids mavros stamps on outgoing MAVLink, so the ids that must hold
# primary control for PX4 to accept the setpoint. mavros defaults, not set
# anywhere in this stack. The manager status subscription warns when the
# ids on the wire disagree with these.
MAVROS_SYSID = 1
MAVROS_COMPID = 191
# The autopilot, matching target_system_id in stack.launch.py.
TARGET_SYSTEM = 1
TARGET_COMPONENT = 1

# The protocol's default lock flags: roll and pitch to the horizon, yaw
# unlocked so the view follows the vehicle heading.
FOLLOW_LOCK_FLAGS = (
    GimbalManagerSetAttitude.GIMBAL_MANAGER_FLAGS_ROLL_LOCK
    | GimbalManagerSetAttitude.GIMBAL_MANAGER_FLAGS_PITCH_LOCK)

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class ClickToGimbal(Node):
    def __init__(self) -> None:
        super().__init__("click_to_gimbal")

        self.declare_parameter("click_topic", "/foxglove/cursor/click")
        self.declare_parameter("camera_info_topic", "/camera/gimbal/camera_info")
        self.declare_parameter("optical_frame", "gimbal_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")
        # What one click does: roi, point or off. Runtime switchable.
        self.declare_parameter("click_mode", "roi")
        # The ground plane, as in ground_projector: rel_alt below the
        # camera, or pinned to ground_z with use_rel_alt false.
        self.declare_parameter("use_rel_alt", True)
        self.declare_parameter("ground_z", 0.0)
        # Which command convention the gimbal obeys.
        #   gz_sim    PX4's simulated gimbal. Its joints ride on the airframe
        #             and ignore the frame flags, so the node sends a
        #             vehicle-relative attitude with the lock flags clear.
        #   mavlink   a gimbal that reads the flags honestly: an earth
        #             referenced attitude with the three lock flags set.
        # The sibling knob for the reported attitude is scene_tf's
        # gimbal_reference.
        self.declare_parameter("gimbal_convention", "gz_sim")

        self.optical = self.get_parameter("optical_frame").value
        self.reference = self.get_parameter("reference_frame").value
        self.earth_referenced = (
            self.get_parameter("gimbal_convention").value == "mavlink")
        # The simulated gimbal ignores flags and runs vehicle relative, so
        # zero labels its setpoints honestly.
        self.setpoint_flags = FOLLOW_LOCK_FLAGS if self.earth_referenced else 0
        self.ground_z_default = float(self.get_parameter("ground_z").value)
        # The camera link orientation the joints hold, from the last command
        # this node sent. roi_tracker re-derives it from the TF chain at
        # click time when another controller has moved the gimbal since.
        # Identity matches a device that was never commanded: GZGimbal
        # steers the joints to zero without a setpoint, and _center
        # commands the same zero.
        self.cmd_q_body_link = (0.0, 0.0, 0.0, 1.0)

        self.tf_buffer = Buffer()
        # spin_thread=True is required. On this node's executor, a lookup that
        # waits for a transform would block the callback that delivers it.
        self.tf_listener = TransformListener(self.tf_buffer, self,
                                             spin_thread=True)

        self.info: CameraInfo | None = None
        self.vehicle_q: tuple[float, float, float, float] | None = None
        self.rel_alt: float | None = None
        self.fix_altitude: float | None = None
        self.amsl: float | None = None
        # None until the first manager status arrives. True while these
        # MAVROS_SYSID and MAVROS_COMPID ids hold primary control.
        self.in_control: bool | None = None
        self.claim_acked = False
        self.claim_inflight = False
        # Trailing-edge flood guard state: the newest held-back click and
        # the one-shot timer that lands it.
        self.pending_click: PointStamped | None = None
        self.pending_click_timer = None
        self.last_click_at = -CLICK_MIN_INTERVAL_S

        self.setpoint_pub = self.create_publisher(
            GimbalManagerSetAttitude,
            "/mavros/gimbal_control/manager/set_attitude", 10)
        self.mode_pub = self.create_publisher(String, "/gimbal/click_mode",
                                              LATCHED)
        self.roi_fix_pub = self.create_publisher(NavSatFix, "/gimbal/roi",
                                                 LATCHED)
        self.roi_point_pub = self.create_publisher(PointStamped,
                                                   "/gimbal/roi_local", LATCHED)
        self.claim_client = self.create_client(CommandLong, "/mavros/cmd/command")
        # CommandInt carries only the ROI location commands, which exist
        # only on the mavlink convention.
        self.roi_client = (self.create_client(CommandInt,
                                              "/mavros/cmd/command_int")
                           if self.earth_referenced else None)
        self.claim_sent_at = 0.0
        self.claim_future = None

        # This node already subscribes to the pose and fix topics, so it
        # feeds the origin instead of letting it subscribe again.
        self.origin = MapOrigin(self, external_updates=True)
        # The whole stabilization emulation for the simulated gimbal lives
        # in roi_tracker.py. An honest gimbal stabilizes flagged commands
        # itself and needs none of it.
        self.roi_tracker = None if self.earth_referenced else RoiTracker(self)
        self.roi_active = False

        self.mode = "roi"
        self._apply_mode(self.get_parameter("click_mode").value, announce=True)
        self.add_on_set_parameters_callback(self._on_parameters)
        # One empty-request service per mode, for one-press buttons.
        for mode in CLICK_MODES:
            self.create_service(
                Trigger, f"/gimbal/click_mode/{mode}",
                lambda request, response, m=mode:
                    self._on_mode_service(m, response))
        self.create_service(
            Trigger, "/gimbal/center",
            lambda request, response: self._on_center_service(response))

        # Subscriptions come last. The TF listener's spin thread can run
        # this node's callbacks as soon as a subscription exists, so
        # everything a callback touches must already be built.
        # Sensor QoS on the mavros topics: they are best effort, and a
        # reliable subscription to a best effort publisher receives nothing.
        self.create_subscription(CameraInfo,
                                 self.get_parameter("camera_info_topic").value,
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(GimbalManagerStatus,
                                 "/mavros/gimbal_control/manager/status",
                                 self._on_status, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        if self.get_parameter("use_rel_alt").value:
            self.create_subscription(Float64, "/mavros/global_position/rel_alt",
                                     self._on_rel_alt, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._on_fix, qos_profile_sensor_data)
        if self.earth_referenced:
            # AMSL serves only DO_SET_ROI_LOCATION, a mavlink-convention
            # path.
            self.create_subscription(Altitude, "/mavros/altitude",
                                     self._on_altitude,
                                     qos_profile_sensor_data)
        # Foxglove publishes clicks reliable, the default.
        self.create_subscription(PointStamped,
                                 self.get_parameter("click_topic").value,
                                 self._on_click, 10)

        self.create_timer(CLAIM_RETRY_S, self._claim_tick)
        # The timer first fires a full period from now. Against an already
        # running mavros the service is ready at once, so try now and close
        # the window where an early click is silently dropped.
        self._claim_tick()

    # ------------------------------------------------------------------ inputs
    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_pose(self, msg: PoseStamped) -> None:
        # The one pose subscription in this process: the tracker and the
        # origin both feed from it.
        q = msg.pose.orientation
        self.vehicle_q = (q.x, q.y, q.z, q.w)
        if self.roi_tracker is not None:
            self.roi_tracker.on_vehicle(msg)
        self.origin.on_local(msg)

    def _on_rel_alt(self, msg: Float64) -> None:
        self.rel_alt = float(msg.data)

    def _on_fix(self, msg: NavSatFix) -> None:
        if msg.status.status >= 0:
            self.fix_altitude = msg.altitude
        self.origin.on_fix(msg)

    def _on_altitude(self, msg: Altitude) -> None:
        self.amsl = msg.amsl

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

    # -------------------------------------------------------------------- mode
    def _on_parameters(self, params) -> SetParametersResult:
        for param in params:
            if param.name != "click_mode":
                continue
            if param.value not in CLICK_MODES:
                return SetParametersResult(
                    successful=False,
                    reason=f"click_mode must be one of {CLICK_MODES}")
            self._apply_mode(param.value)
        return SetParametersResult(successful=True)

    def _on_mode_service(self, mode: str, response):
        """Set click_mode through the parameter, so every path agrees."""
        result = self.set_parameters([Parameter(
            "click_mode", Parameter.Type.STRING, mode)])[0]
        response.success = result.successful
        response.message = (f"click_mode: {mode}" if result.successful
                            else result.reason)
        return response

    def _release_hold(self) -> None:
        """Drop any standing hold, leaving the gimbal where it points."""
        if self.roi_tracker is not None:
            self.roi_tracker.clear()
        if self.roi_active:
            self.roi_active = False
            if self.earth_referenced:
                self._send_roi_none()

    def _on_center_service(self, response):
        self._release_hold()
        if self._center():
            response.success = True
            response.message = "gimbal centered, pitch and yaw zero"
        else:
            response.success = False
            response.message = "center not sent: the command service is busy"
        return response

    def _apply_mode(self, mode: str, announce: bool = False) -> None:
        if mode not in CLICK_MODES:
            self.get_logger().warn(
                f"click_mode '{mode}' is not one of {CLICK_MODES}, using roi")
            mode = "roi"
        # A standing hold continues across mode changes, as a real gimbal
        # keeps its last earth referenced command. The modes only decide
        # what a new click does, and /gimbal/center releases the hold.
        changed = mode != self.mode
        self.mode = mode
        self.mode_pub.publish(String(data=mode))
        if changed or announce:
            self.get_logger().info(f"click_mode: {mode}")

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

    def _center(self) -> bool:
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
        if not self._send_command(request):
            return False
        # Only a dispatched center resets the joint state, so the state
        # keeps matching the joints when the command cannot go out.
        self.cmd_q_body_link = (0.0, 0.0, 0.0, 1.0)
        return True

    def _send_command(self, request) -> bool:
        if self.claim_inflight or not self.claim_client.service_is_ready():
            return False
        self.claim_inflight = True
        self.claim_sent_at = self.get_clock().now().nanoseconds / 1e9
        self.claim_future = self.claim_client.call_async(request)
        self.claim_future.add_done_callback(self._on_claim_done)
        return True

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
        """The trailing-edge flood guard. _process_click does the work."""
        now = time.monotonic()
        if now - self.last_click_at >= CLICK_MIN_INTERVAL_S:
            # This click supersedes any held one: disarm the timer so a
            # stale older click can never land after a newer one.
            self.pending_click = None
            if self.pending_click_timer is not None \
                    and not self.pending_click_timer.is_canceled():
                self.pending_click_timer.cancel()
            self.last_click_at = now
            self._process_click(msg)
            return
        # Inside the interval: hold the click, newest wins, and let the
        # timer land it. A genuine rapid re-click arrives at most one
        # interval late, never dropped.
        self.pending_click = msg
        if self.pending_click_timer is not None:
            if not self.pending_click_timer.is_canceled():
                return    # already armed; the newest click rides it
            # A spent timer from an earlier burst. This subscription
            # callback is a safe place to destroy it; its own was not.
            self.destroy_timer(self.pending_click_timer)
        remaining = CLICK_MIN_INTERVAL_S - (now - self.last_click_at)
        self.pending_click_timer = self.create_timer(
            remaining, self._on_pending_click)

    def _on_pending_click(self) -> None:
        self.pending_click_timer.cancel()
        msg, self.pending_click = self.pending_click, None
        if msg is None:
            return    # a newer click already consumed the pending state
        self.last_click_at = time.monotonic()
        self._process_click(msg)

    def _process_click(self, msg: PointStamped) -> None:
        if not intrinsics_ready(self.info):
            self.get_logger().warn("click ignored: no camera intrinsics yet")
            return
        u, v = msg.point.x, msg.point.y
        if not (0.0 <= u < self.info.width and 0.0 <= v < self.info.height):
            self.get_logger().warn(
                f"click ignored: pixel ({u:.0f}, {v:.0f}) is outside the "
                f"{self.info.width}x{self.info.height} image")
            return
        if self.mode == "off":
            self.get_logger().info("click ignored: click_mode is off",
                                   throttle_duration_sec=5.0)
            return

        camera = self._camera_pose()
        if camera is None:
            return
        position, rotation = camera
        direction = quat_rotate(rotation, ray_in_optical(u, v, self.info.k))
        if self.mode == "roi":
            self._roi_click(u, v, position, direction)
        else:
            self._point_click(u, v, direction)

    def _point_click(self, u: float, v: float, direction) -> None:
        """Hold pitch and roll on the horizon, yaw follows the heading."""
        if self.vehicle_q is None:
            self.get_logger().warn("click dropped: no vehicle attitude yet")
            return
        self._release_hold()
        _, pitch, yaw = pointing_rpy_ned(direction)
        if self.earth_referenced:
            heading = rpy_from_quat(ros_to_aerospace(self.vehicle_q))[2]
            self.command_body_attitude(0.0, pitch, wrap_pi(yaw - heading))
        else:
            self.roi_tracker.follow(pitch, yaw)
        self.get_logger().info(
            f"click ({u:.0f}, {v:.0f}) -> hold pitch "
            f"{math.degrees(pitch):+.1f} deg on the horizon, yaw "
            f"{math.degrees(yaw):+.1f} deg now, following the heading")

    def _roi_click(self, u: float, v: float, position, direction) -> None:
        """Hold the camera on the ground point under the clicked pixel."""
        ground_z = (self.ground_z_default if self.rel_alt is None
                    else position[2] - self.rel_alt)
        hit = intersect_ground(position, direction, ground_z,
                               GROUND_VIEW_MAX_DISTANCE_M)
        if hit is None:
            self.get_logger().warn(
                f"click dropped: pixel ({u:.0f}, {v:.0f}) meets no ground "
                f"within {GROUND_VIEW_MAX_DISTANCE_M:.0f} m")
            return

        self.roi_active = True
        self._publish_roi(hit)
        if self.earth_referenced:
            self._send_roi_location(hit)
        else:
            self.roi_tracker.track(hit)
        self.get_logger().info(
            f"click ({u:.0f}, {v:.0f}) -> ROI at map "
            f"({hit[0]:.1f}, {hit[1]:.1f}, {hit[2]:.1f})")

    def _camera_pose(self):
        """Reference frame position and rotation of the optical frame, or
        None with a warning when the transform is not available."""
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
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    # --------------------------------------------------------------------- roi
    def _publish_roi(self, hit) -> None:
        stamp = self.get_clock().now().to_msg()
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = self.reference
        point.point.x, point.point.y, point.point.z = hit
        self.roi_point_pub.publish(point)

        latlon = self.origin.to_lla(hit[0], hit[1])
        if latlon is None:
            self.get_logger().warn(
                "ROI has no lat/lon yet: the map origin is not known")
            return
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = self.reference
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude = latlon
        # The ground in the datum of the vehicle's own fix.
        fix.altitude = (float("nan")
                        if self.fix_altitude is None or self.rel_alt is None
                        else self.fix_altitude - self.rel_alt)
        self.roi_fix_pub.publish(fix)

    def _send_roi_location(self, hit) -> None:
        latlon = self.origin.to_lla(hit[0], hit[1])
        if latlon is None or self.amsl is None or self.rel_alt is None:
            self.get_logger().warn(
                "DO_SET_ROI_LOCATION not sent: no origin or altitude yet")
            return
        request = CommandInt.Request()
        request.frame = MAV_FRAME_GLOBAL_INT
        request.command = MAV_CMD_DO_SET_ROI_LOCATION
        request.x = int(latlon[0] * 1e7)
        request.y = int(latlon[1] * 1e7)
        request.z = self.amsl - self.rel_alt
        self._send_roi_command(request)

    def _send_roi_none(self) -> None:
        request = CommandInt.Request()
        request.command = MAV_CMD_DO_SET_ROI_NONE
        self._send_roi_command(request)

    def _send_roi_command(self, request) -> None:
        if not self.roi_client.service_is_ready():
            self.get_logger().warn("ROI command not sent: no command_int "
                                   "service yet")
            return
        command = request.command
        future = self.roi_client.call_async(request)
        future.add_done_callback(
            lambda f: self._on_roi_command_done(command, f))

    def _on_roi_command_done(self, command: int, future) -> None:
        try:
            response = future.result()
        except Exception as err:  # noqa: BLE001 - a dead service can drop it
            self.get_logger().warn(f"ROI command {command} failed: {err}")
            return
        if not response.success:
            self.get_logger().warn(
                f"ROI command {command} rejected, result={response.result}")

    # ----------------------------------------------------------------- command
    def command_body_attitude(self, roll: float, pitch: float,
                              yaw: float) -> None:
        """Command a vehicle-relative attitude, radians in the aerospace
        convention. roi_tracker calls this too."""
        if self.in_control is False:
            # The setpoint races the claim to the autopilot, and a setpoint
            # that arrives first is dropped. The command still goes out: at
            # worst it restores control and the next one lands.
            self._claim()
            self.get_logger().warn(
                "another party held gimbal control at command time. If the "
                "gimbal does not move, click again.",
                throttle_duration_sec=5.0)
        setpoint = quat_from_rpy(roll, pitch, yaw)
        if not self.earth_referenced:
            self.cmd_q_body_link = body_frd_to_flu(setpoint)
        self._publish_setpoint(setpoint)

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
