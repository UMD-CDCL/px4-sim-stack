# The PX4 simulated gimbal

How PX4's simulated gimbal really behaves over the MAVLink gimbal protocol
v2, and how to command it so it points where you intend. The gimbal is
`GZGimbal.cpp` in PX4's gz_bridge driving the CGO3 model, the one the
`gz_x500_gimbal` airframe carries. Each behavior below differs from what
the protocol promises, and each produces plausible wrong pointing instead
of an error. This stack accommodates all of them in `sim_bridge`, and this
page states them once so another project does not have to rediscover them.

## It executes every command vehicle relative

The MAVLink contract: with the ROLL, PITCH and YAW lock flags set, a
GIMBAL_MANAGER_SET_ATTITUDE quaternion is earth referenced, and the device
holds it against the horizon and North.

The simulation ignores that contract. PX4's gimbal module passes the
quaternion through unchanged (`output_mavlink.cpp`, OutputMavlinkV2), and
GZGimbal converts it to Euler angles and writes them onto joint position
controllers that ride on the airframe. The result is a follow mode device
on every axis: the command runs as vehicle-relative roll, pitch and yaw in
the aerospace sign convention, whatever the flags say. The joints track a
setpoint to about 0.1 degrees.

To point at something earth fixed, convert the desired direction into the
vehicle frame first, and send that attitude with the lock flags clear. Do
not send earth referenced yaw and trim the miss with a fixed offset. The
miss equals the vehicle heading, so the offset only works at the heading
where it was measured. Keep the honest earth referenced command path
behind a switch for real hardware.

## It computes a region of interest once

DO_SET_ROI_LOCATION reaches PX4's gimbal module, but the v2 output
computes the attitude toward the point one time, when the command
arrives (`output_mavlink.cpp`, OutputMavlinkV2 recomputes only on new
setpoints). That attitude is earth referenced with the yaw lock flag
set, and the section above says what the simulated gimbal then does
with it. So an ROI does not track the point as the vehicle moves, and
it does not point correctly even at the start.

To hold the simulated gimbal on a world point, or to stabilize pitch
and roll while yaw follows the heading, recompute the vehicle-relative
attitude yourself as the vehicle moves. This stack does that in
`sim_bridge/roi_tracker.py`, one small file built to be easy to drop
when PX4 learns to do it.

## It reports an absolute attitude labeled vehicle relative

GZGimbal builds GIMBAL_DEVICE_ATTITUDE_STATUS from the gimbal IMU, and a
Gazebo IMU reports orientation against the world. The quaternion is
therefore absolute, but the message flags claim
DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME. A consumer that believes the flag
composes the vehicle attitude onto an attitude that already contains it,
and the camera appears to turn twice as far as it does.

Treat the report as earth referenced, and recover the vehicle-relative
part by dividing the vehicle attitude off the left:
`q_rel = conj(q_vehicle) * q_abs`. Dividing on the right is a conjugation.
It looks correct with the gimbal centered and swaps axes once the gimbal
moves. `patches/px4-gzgimbal-frame.patch` corrects the label at the source
instead.

## Do not mix the report with the EKF attitude

The report comes from simulation ground truth. The vehicle attitude you
divide out comes from the EKF, which is an estimate, and its heading error
reached 16 degrees in flight here. The division above therefore moves the
whole EKF heading error into the derived vehicle-relative orientation.

The absolute camera orientation is safe: composing the EKF vehicle
attitude back on cancels the error exactly, so earth-frame projections
stay correct. The vehicle-relative half is not safe, and a command
computed from it misses by the EKF error. Since the joints track the last
setpoint exactly, the last commanded attitude is the true joint state.
Compose new vehicle-relative commands from your own last command, not from
the report divided by the EKF. `GIMBAL_DEVICE_SET_ATTITUDE` would carry
the joint setpoint whoever commanded it, but mavros does not translate
that message, so it cannot cover a gimbal moved by another controller,
such as the QGC joystick. When that happens, the last command is stale:
re-derive the joint state by dividing your standing vehicle-attitude
correction off the world-true camera orientation, as
`sim_bridge/roi_tracker.py` does at click time. The correction drifts
only at EKF speed, so a sudden disagreement always belongs to the joints.

## Summary

| The simulation does | You do |
|---|---|
| Executes commands vehicle relative, flags ignored | Send a vehicle-relative attitude, lock flags clear |
| Computes an ROI attitude once, earth referenced | Re-command the point yourself as the vehicle moves |
| Reports an absolute attitude flagged vehicle relative | Divide the vehicle attitude off the left, or apply the patch |
| Reports truth while the EKF estimates | Build earth-frame poses from the report, build command state from your own setpoints, re-derived from TF after an external move |

`sim_bridge/click_to_gimbal.py` and `sim_bridge/scene_tf.py` implement
these accommodations, switchable back to honest MAVLink behavior for real
hardware through `gimbal_convention` and `gimbal_reference`.

## The rates, and the patch that raises them

Stock GZGimbal runs its whole cycle at 5 Hz (`ScheduleOnInterval(200_ms)`):
setpoint poll, joint move, and attitude report alike. The joints therefore
step toward a stabilization target five times a second and the camera
visibly lags the airframe through any maneuver. On top of that, the
GIMBAL_DEVICE_ATTITUDE_STATUS mavlink stream runs at the profile default,
one report every few seconds, so the ROS frame tree is seconds stale.
`patches/px4-gzgimbal-rate.patch` raises the cycle to 50 Hz, and px4-rcS
streams the report at 50 Hz to match. With both in place the joints follow
a streamed setpoint within 20 ms and `scene_tf` can trust the report as
the live camera orientation, which is what keeps image overlays glued to
the video.
