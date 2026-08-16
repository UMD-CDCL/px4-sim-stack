#!/usr/bin/env python3
"""Turn clicks on the gimbal image into gimbal action.

Foxglove publishes the clicked pixel as a PointStamped. click_mode decides
what one click does:

    roi     hold the camera on the scene point under the pixel. Default.
    point   hold the clicked pitch against the horizon, yaw follows the
            vehicle heading.
    off     ignore clicks.

A hold persists across mode changes and through off mode; the modes only
decide what a new click does. A new click replaces the hold, and
/gimbal/center releases it.

gimbal_convention selects how a command is expressed. "mavlink" sends an
earth-referenced attitude with the lock flags set and lets the device
stabilize, and uses DO_SET_ROI_LOCATION for a hold. "gz_sim" sends a
vehicle-relative attitude with the flags clear and lets roi_tracker.py do
the stabilizing, because PX4's simulated gimbal ignores the flags and
computes an ROI attitude only once. docs/px4-simulated-gimbal.md states
every device behavior this accommodates, and why a command must never be
built from the TF gimbal segment alone, which carries the EKF heading error.

PX4 drops a setpoint silently unless its sender holds primary gimbal
control, so the node claims control at startup and again whenever a command
goes out while another party holds it. The claim goes through the generic
/mavros/cmd/command service rather than the gimbal_control configure
service, whose handler blocks its plugin's executor until acknowledged: one
lost acknowledgment would stop that plugin sending any gimbal message at
all. A claim that never answers is treated as a stuck gimbal and recovered
with the center command, which also hands control to its sender.

Subscribes
    <click_topic>        geometry_msgs/PointStamped, pixels on the image
    <camera_info_topic>  sensor_msgs/CameraInfo
    /mavros/gimbal_control/manager/status   who holds control
    /mavros/local_position/pose             the vehicle attitude
    /mavros/global_position/rel_alt         the ground plane height
    /mavros/global_position/global          feeds the map origin
    /mavros/altitude                        AMSL for DO_SET_ROI_LOCATION,
                                            mavlink convention only

Publishes
    /mavros/gimbal_control/manager/set_attitude
    /gimbal/click_mode   std_msgs/String, latched
    /gimbal/roi_local    geometry_msgs/PointStamped, latched
    /gimbal/roi_geojson  foxglove_msgs/GeoJSON, latched, empty when unset.
                         GeoJSON rather than NavSatFix because a latched fix
                         cannot be taken back when the hold is released.

Serves
    /gimbal/click_mode/{roi,point,off}   std_srvs/Trigger
    /gimbal/center                       std_srvs/Trigger
"""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import Altitude, GimbalManagerSetAttitude, GimbalManagerStatus
from mavros_msgs.srv import CommandInt, CommandLong
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger

from sim_bridge.frames import CameraFrame
from sim_bridge.geo import MapOrigin
from sim_bridge.localization import GroundLocalizer
from sim_bridge.projection import (GROUND_VIEW_MAX_DISTANCE_M,
                                   body_frd_to_flu, intrinsics_ready,
                                   pointing_rpy_ned, quat_from_rpy,
                                   quat_rotate, ray_in_optical,
                                   ros_to_aerospace, rpy_from_quat, wrap_pi)
from sim_bridge.roi_tracker import RoiTracker
from sim_bridge.runtime import (LATCHED, geojson_publisher, now_s,
                                publish_features, spin, tf_buffer)

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


class ClickToGimbal(Node):
    def __init__(self) -> None:
        super().__init__("click_to_gimbal")

        self.declare_parameter("click_topic", "/foxglove/cursor/click")
        self.declare_parameter("camera_info_topic", "/camera/gimbal/camera_info")
        self.declare_parameter("optical_frame", "gimbal_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("click_mode", "roi")
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
        # Declares localization_mode, surface_file, use_rel_alt and
        # ground_z, and answers every click ray the same way the detection
        # localizers answer theirs. Its ground_plane keeps rel_alt readable
        # for the ROI altitude below. See sim_bridge/localization.py.
        self.localizer = GroundLocalizer(self)
        # The camera link orientation the joints hold, from the last command
        # this node sent. roi_tracker re-derives it from the TF chain at
        # click time when another controller has moved the gimbal since.
        # Identity matches a device that was never commanded: GZGimbal
        # steers the joints to zero without a setpoint, and _center
        # commands the same zero.
        self.cmd_q_body_link = (0.0, 0.0, 0.0, 1.0)
        self.camera_frame = CameraFrame(self, self.optical, self.reference)

        self.info: CameraInfo | None = None
        self.vehicle_q: tuple[float, float, float, float] | None = None
        self.amsl: float | None = None
        # None until the first manager status arrives. True while these
        # MAVROS_SYSID and MAVROS_COMPID ids hold primary control.
        self.in_control: bool | None = None
        self.claim_acked = False
        self.claim_inflight = False
        self.pending_click: PointStamped | None = None
        self.pending_click_timer = None
        self.last_click_at = -CLICK_MIN_INTERVAL_S

        self.setpoint_pub = self.create_publisher(
            GimbalManagerSetAttitude,
            "/mavros/gimbal_control/manager/set_attitude", 10)
        self.mode_pub = self.create_publisher(String, "/gimbal/click_mode",
                                              LATCHED)
        self.roi_point_pub = self.create_publisher(PointStamped,
                                                   "/gimbal/roi_local", LATCHED)
        self.roi_geojson_pub = geojson_publisher(
            self, "/gimbal/roi_geojson", "the Map panel gets no ROI marker")
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

    def _on_fix(self, msg: NavSatFix) -> None:
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
            self._publish_roi_geojson(None)
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
            if now_s(self) - self.claim_sent_at > CLAIM_RECOVER_S:
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
        self.claim_sent_at = now_s(self)
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
        """Hold the camera on the scene point under the clicked pixel."""
        hit = self.localizer.intersect(position, direction,
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
        """Where the camera looks now, or None with a warning. The newest
        pose rather than the click stamp: the command steers from here."""
        pose = self.camera_frame.latest(timeout_s=TF_TIMEOUT_S)
        if pose is None:
            self.get_logger().warn(
                f"click dropped: no pose {self.reference} -> {self.optical}")
            return None
        return pose.position, pose.rotation

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
        self._publish_roi_geojson(latlon)

    def _publish_roi_geojson(self, latlon) -> None:
        """Publish the ROI as one GeoJSON point, or clear it with an empty
        collection when latlon is None."""
        features = [] if latlon is None else [{
            "type": "Feature",
            "geometry": {"type": "Point",
                         "coordinates": [latlon[1], latlon[0]]},
            "properties": {"name": "gimbal roi"},
        }]
        publish_features(self.roi_geojson_pub, features)

    def _send_roi_location(self, hit) -> None:
        rel_alt = self.localizer.ground_plane.rel_alt
        latlon = self.origin.to_lla(hit[0], hit[1])
        if latlon is None or self.amsl is None or rel_alt is None:
            self.get_logger().warn(
                "DO_SET_ROI_LOCATION not sent: no origin or altitude yet")
            return
        request = CommandInt.Request()
        request.frame = MAV_FRAME_GLOBAL_INT
        request.command = MAV_CMD_DO_SET_ROI_LOCATION
        request.x = int(latlon[0] * 1e7)
        request.y = int(latlon[1] * 1e7)
        # amsl - rel_alt is the AMSL of the latched plane; the hit's height
        # above that plane carries a roof or a slope into the ROI altitude.
        # On the plane itself the correction is exactly zero.
        request.z = (self.amsl - rel_alt
                     + (hit[2] - self.localizer.ground_plane.z()))
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
    spin(ClickToGimbal)


if __name__ == "__main__":
    main()
