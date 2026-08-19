# The PX4 simulated gimbal

How PX4's simulated gimbal behaves over the MAVLink gimbal protocol v2, and how
to command it so that it points where you intend. The gimbal is `GZGimbal.cpp`
in PX4's gz_bridge, driving the CGO3 model that the `gz_x500_gimbal` airframe
carries. Each behavior below differs from what the protocol promises, and each
one produces plausible wrong pointing instead of an error. Two of them have a
patch here that corrects them.

The flight code does not accommodate the simulator, so every accommodation is
here: two patches against PX4, the model in
`modules/sim/scenes/models/gimbal/model.sdf`, and the stream rates in
`modules/sim/px4-rcS`.

## It executes every command vehicle relative

The MAVLink contract: with the ROLL, PITCH and YAW lock flags set, a
GIMBAL_MANAGER_SET_ATTITUDE quaternion is earth referenced, and the device holds
it against the horizon and North.

The simulation ignores that contract. PX4's gimbal module passes the quaternion
through unchanged (`output_mavlink.cpp`, OutputMavlinkV2), and GZGimbal converts
it to Euler angles and writes them onto joint position controllers that ride on
the airframe. The result is a follow mode device on every axis: the command runs
as vehicle-relative roll, pitch and yaw in the aerospace sign convention,
whatever the flags say. The joints track a setpoint to about 0.1 degrees.

`umd_uas/gimbal.py` sends flags 12, ROLL_LOCK and PITCH_LOCK, so the yaw it
sends is already vehicle relative and the simulation executes that half
correctly. The roll and pitch halves are the ones the simulation gets wrong: the
node asks for a level horizon, and a real device holds it while a banked
aircraft rolls under it. This one does not.

To point at something earth fixed, convert the direction into the vehicle frame
first, and send that attitude with the lock flags clear. Do not send earth
referenced yaw and trim the miss with a fixed offset. The miss equals the
vehicle heading, so the offset only works at the heading where it was measured.

## It computes a region of interest once

DO_SET_ROI_LOCATION reaches PX4's gimbal module, but the v2 output computes the
attitude toward the point one time, when the command arrives
(`output_mavlink.cpp`, OutputMavlinkV2 recomputes only on new setpoints). That
attitude is earth referenced with the yaw lock flag set, and the section above
says what the simulated gimbal then does with it. So an ROI does not track the
point as the vehicle moves, and it does not point correctly even at the start.

`umd_uas/survey.py` points through `gimbal_control/manager/set_roi`, so a
simulated survey inherits this. A click from the operator does not: it becomes a
`DetectionInterface`, and `gimbal.py` turns that into a vehicle-relative
attitude, which the simulation executes correctly.

To hold the simulated gimbal on a world point, recompute the vehicle-relative
attitude as the vehicle moves, and send it as an attitude rather than as an ROI.

## It reported an absolute attitude with no lock flag

GZGimbal builds GIMBAL_DEVICE_ATTITUDE_STATUS from the gimbal IMU, and a Gazebo
IMU reports orientation against the world. The quaternion is therefore earth
referenced. The stock message says the opposite, because it sets
DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME and no other bit.

The label is only half of the fault. The MAVLink specification puts the frame of
the quaternion in the ROLL_LOCK, PITCH_LOCK and YAW_LOCK bits, and the stock
report sets none of them. MAVInsight builds `d<N>_gimbal_frame_ref` from those
bits (`models/vehicle.py`), then applies the quaternion to that frame
(`models/sensor.py`). With no lock bit the reference frame stays the body frame,
so the earth referenced report goes onto the airframe attitude a second time.

`patches/px4-gzgimbal-frame.patch` makes GZGimbal report what a real gimbal
reports. It sends the lock bits of the last setpoint, it sets
YAW_IN_VEHICLE_FRAME or YAW_IN_EARTH_FRAME to agree with them, and it moves the
quaternion into the frame those flags declare. PX4 asks for ROLL_LOCK and
PITCH_LOCK by default, and the Chimera flight code asks for the same two, so the
usual report is flags 44: the lock bits 12, plus 32 for the vehicle frame. The
quaternion then holds the camera attitude against a level frame at the airframe
heading, which is what the aircraft gimbal sends.

The arithmetic, with the airframe at roll 20, pitch 10, yaw 40 degrees and the
camera 30 degrees down and 25 degrees right of the nose. That yaw is more than
the two-axis mount can make, and it is here to move all three axes:

| the report | flags | the camera frame MAVInsight builds |
|---|---|---|
| stock | 32 | 44.0 degrees away from the truth |
| patched | 44 | exact |

Across 2000 random attitudes the stock error reached 180 degrees, and the
patched error stayed at zero.

The patch does not make the simulated gimbal hold the horizon. The lock bits
report the mode that was commanded, as a real device does, while the quaternion
reports where the camera is. So the report now shows the roll and pitch error
that the first section describes.

MAVInsight took one fix with this. Its earth reference frame, which the YAW_LOCK
bit selects, was aligned with ENU. The MAVLink earth frame is NED, so a yaw
locked report built a camera frame 90 degrees off. A region of interest sets
YAW_LOCK, and a real gimbal has the same fault, so the fix is not a simulator
accommodation. `models/frame_utils.py` now holds `R_enu_nwu` for that turn.

A consumer that wants the joint angles must still divide, and it divides on the
LEFT:

```
q_rel = conj(q_reference) * q_camera
```

An earth referenced attitude is the airframe attitude followed by the rotation
of the gimbal, `q_abs = q_vehicle * q_rel`, which is where that comes from.
Dividing on the right instead leaves `q_vehicle * q_rel * conj(q_vehicle)`, a
conjugation: the same rotation through the same angle, about an axis turned by
the vehicle heading. That has a distinctive signature. Conjugation maps identity
to identity, so a **centred gimbal looks perfect at every heading** and the
error appears only once the gimbal moves off centre, where it reads as swapped
axes rather than as a rotation error:

| aircraft at 90 deg yaw, gimbal pitched 30 deg down | roll | pitch | yaw |
|---|---|---|---|
| truth | 0.0 | -30.0 | 0.0 |
| `conj(qv) * qabs` | 0.0 | -30.0 | 0.0 |
| `qabs * conj(qv)` | +30.0 | 0.0 | 0.0 |

**Correct at zero, wrong off zero** means conjugation. It does not mean a bad
axis, and no constant offset can fix it.

One more trap when you judge any of this from the aircraft: a gimbal that holds
an ROI is earth locked, so its angle relative to the airframe genuinely changes
as the aircraft yaws. That is correct behavior and it reads exactly like the
fault. Centre the gimbal, or command it in vehicle relative mode, before you
decide.

## Do not mix the report with the EKF attitude

The quaternion starts as simulation ground truth, but the patched report is
measured against the airframe, and the airframe attitude comes from the EKF.
GZGimbal reads `vehicle_attitude`, whose heading error reached 16 degrees in
flight here, and divides it off before it sends the report.

The camera frame in the world is safe. MAVInsight multiplies the same estimate
back on, so the error cancels exactly and earth-frame projections stay correct.
The vehicle-relative half is not safe, and a command computed from it misses by
the EKF error. Since the joints track the last setpoint exactly, the last
commanded attitude is the true joint state. Build a new vehicle-relative command
from your own last command, not from the report.

`GIMBAL_DEVICE_SET_ATTITUDE` would carry the joint setpoint whoever commanded
it, but MAVROS does not translate that message, so it cannot cover a gimbal
moved by another controller, such as the QGC joystick. After such a move the
last command is stale. Re-derive the joint state by dividing your standing
vehicle-attitude correction off the world-true camera orientation. The
correction drifts only at EKF speed, so a sudden disagreement always belongs to
the joints.

## The yaw axis is locked, on purpose

The Chimera mount is two axis, roll and pitch. It has no yaw actuator, and the
camera's azimuth comes from the airframe heading. `model.sdf` therefore gives
the yaw joint equal lower and upper limits, which holds the arm at zero and
leaves the joint and its controller in place, so PX4 still finds everything it
commands.

Do not open that axis to make a pointing click work. The aircraft would have to
turn, so the simulator must turn too.

## The rates, and the patch that raises them

Stock GZGimbal runs its whole cycle at 5 Hz (`ScheduleOnInterval(200_ms)`):
setpoint poll, joint move, and attitude report alike. The joints therefore step
toward a stabilization target five times a second, and the camera visibly lags
the airframe through any maneuver. On top of that, the
GIMBAL_DEVICE_ATTITUDE_STATUS stream runs at the profile default, one report
every few seconds, so the frame tree is seconds stale.

`patches/px4-gzgimbal-rate.patch` raises the cycle to 50 Hz, and `px4-rcS`
streams the report at 50 Hz to match. With both in place the joints follow a
streamed setpoint within 20 ms, and the frame tree can trust the report as the
live camera orientation. That is what keeps an image overlay glued to the video.

`hold-stream-rates.sh` reissues those rates every ten seconds, because PX4 drops
a link back to the profile default when a ground station appears on it.

## Summary

| The simulation does | You do |
|---|---|
| Executes commands vehicle relative, flags ignored | Send a vehicle-relative attitude, lock flags clear |
| Computes an ROI attitude once, earth referenced | Re-command the point yourself as the vehicle moves |
| Reported an absolute attitude with no lock bit | Apply the frame patch, then check the gimbal frame against the vehicle heading |
| Measures the patched report against the EKF airframe attitude | Build earth-frame poses from the report, build command state from your own setpoints |
| Runs its cycle at 5 Hz | Apply the rate patch, and stream the report at 50 Hz |

To apply the patches, from `src/PX4-Autopilot`:

```bash
git apply ../../patches/px4-gzgimbal-frame.patch
git apply ../../patches/px4-gzgimbal-rate.patch
```

Then restart `sim`. The entrypoint rebuilds what changed.
