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
the report divided by the EKF.

## Summary

| The simulation does | You do |
|---|---|
| Executes commands vehicle relative, flags ignored | Send a vehicle-relative attitude, lock flags clear |
| Reports an absolute attitude flagged vehicle relative | Divide the vehicle attitude off the left, or apply the patch |
| Reports truth while the EKF estimates | Build earth-frame poses from the report, build command state from your own setpoints |

`sim_bridge/click_to_gimbal.py` and `sim_bridge/scene_tf.py` implement
these accommodations, switchable back to honest MAVLink behavior for real
hardware through `gimbal_convention` and `gimbal_reference`.
