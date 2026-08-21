# The PX4 simulated gimbal

How PX4's simulated gimbal behaves over the MAVLink gimbal protocol v2, and how
to command it so that it points where you intend. The gimbal is `GZGimbal.cpp`
in PX4's gz_bridge, driving the CGO3 model that the `gz_x500_gimbal` airframe
carries. Each behavior below differs from what the protocol promises, and each
one produces plausible wrong pointing instead of an error. Three of them have a
patch here that corrects them.

The flight code does not accommodate the simulator, so every accommodation is
here: three patches against PX4, the model in
`modules/sim/scenes/models/gimbal/model.sdf`, and the stream rates in
`modules/sim/px4-rcS`.

## It executed every command vehicle relative

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
sends is already vehicle relative and the simulation executed that half
correctly. The roll and pitch halves were the ones it got wrong: the node asks
for a level horizon, and a real device holds it while a banked aircraft rolls
under it. At 20 degrees of bank the simulated camera rolled 20 degrees with the
airframe, and every consumer that casts a pixel through the camera frame
inherited that error.

`patches/px4-gzgimbal-lock.patch` corrects it. It takes the airframe attitude
off the setpoint before the joint angles are written, which is the arithmetic
the frame patch does for the report, the other way round:

```
report:  q_report = conj(q_reference) * q_camera
command: q_joint  = conj(q_vehicle) * q_reference * q_setpoint
```

`q_reference` is the frame the lock bits name, from `attitudeReferenceFrame()`.
With no lock bit that frame is the airframe, the two conjugates cancel, and the
joints take the setpoint unchanged, so a follow mode command behaves as it
always did. The patch also runs that arithmetic every cycle rather than only
when a setpoint arrives, because a locked axis is held against the world and a
real device stabilizes continuously from its own IMU.

That arithmetic alone is not enough, and the second half is the one that bites.

## The arms turn in a different order than the angles are read

`model.sdf` builds the mount as three revolute joints in a chain: the vertical
arm about the yaw axis, then the horizontal arm about the roll axis, then the
camera about the pitch axis. The attitude those arms build is

```
R = Rz(yaw) * Rx(roll) * Ry(pitch)
```

`matrix::Eulerf` reads `Rz * Ry * Rx`, the aerospace order, which puts pitch
before roll. **The two orders give the same three angles only while roll is
zero.** PX4 commands roll zero by default and the Chimera flight code sends a
level horizon on every command, so roll has been zero for the life of this
simulator and GZGimbal has always read the setpoint in the wrong order without
it ever mattering. Stabilization is the first thing that ever asked these arms
for a roll.

The miss is large and it does not look like a gain error. Across the flight
envelope the aerospace order leaves up to 40 degrees of pointing error, and it
is worst where the camera is pitched furthest down, which is where this camera
works. The reason is physical: the roll arm turns before the camera is pitched,
so with the camera pitched down a rotation of the roll arm swings the boresight
sideways in azimuth instead of rolling the picture.

Measured on the running simulator, from the Gazebo link poses: a commanded roll
of 2.43 degrees with the camera at pitch -45 moved the boresight 2.05 degrees
in azimuth and left the roll of the picture almost unchanged.

So the patch reads the angles in the chain's own order, and both the locked
path and the plain path use the one function that does it. Over 20000 random
airframe attitudes with the setpoint the flight code sends, the aerospace order
leaves up to 40.096 degrees of error and the chain order leaves 1.7e-06.

The yaw arm never has to move for any of it. For a setpoint of roll zero and
vehicle-frame yaw zero, the yaw joint this decomposition asks for is zero at
every airframe attitude, to 8e-15 degrees. **The roll arm and the pitch arm
hold the horizon on their own**, so the yaw arm turns only when a commander
asks for an azimuth. Stabilization and pointing never fight over that axis.

If you ever suspect this class of fault again, the signature is that the
correction is in the right direction and the wrong size, and that the size
depends on the pitch angle. A fixed offset cannot fix it and halving it only
moves the error to a different pitch.

With the patch in place, send the attitude the aircraft gets: earth referenced
roll and pitch, vehicle referenced yaw, flags 12. Do not convert a direction
into the vehicle frame yourself, and do not trim a miss with a fixed offset.
The miss equals the vehicle attitude, so an offset only works at the attitude
where it was measured.

## It computes a region of interest once

DO_SET_ROI_LOCATION reaches PX4's gimbal module, but the v2 output computes the
attitude toward the point one time, when the command arrives
(`output_mavlink.cpp`, OutputMavlinkV2 recomputes only on new setpoints). That
attitude is earth referenced with the yaw lock flag set, and the section above
says what the simulated gimbal then does with it. So an ROI does not track the
point as the vehicle moves, and it does not point correctly even at the start.

This is not a simulator fault, so there is nothing here to patch.
`umd_uas/gimbal.py` holds the place instead: it casts the clicked pixel at the
ground, keeps the answer as WGS84, and re-sends the attitude at 10 Hz while the
aircraft moves. `survey.py` asks that node for its survey points rather than
asking the autopilot, so one thing owns the setpoint.

Measured on the running simulator, this is what a second commander looks like.
The gimbal report read flags 92 and pitch -55.8 degrees an hour after
`gimbal.py` last commanded pitch -60. Flag 16 is YAW_LOCK, which PX4 sets for a
region of interest (`output.cpp`, `_absolute_angle[2] = true`) and which
`gimbal.py` never sends. A standing region of interest had held the gimbal ever
since, and nothing reported the disagreement.

A region of interest also skips the control owner check. DO_SET_ROI_LOCATION
goes to the navigator, not to the gimbal module (`navigator_main.cpp`), and
`vehicle_roi` carries no sender, so any station can point the gimbal with one
whoever holds primary control.

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
PITCH_LOCK by default, and the Chimera flight code asks for the same two, so
the report reads flags 44 after either commands: the lock bits 12, plus 32 for
the vehicle frame. The quaternion then holds the camera attitude against a
level frame at the airframe heading, which is what the aircraft gimbal sends.

Read the flags rather than assuming them. The running simulator reported flags
92 instead, because the last setpoint came from a region of interest and
carried YAW_LOCK. 92 is the lock bits 28 plus 64 for the earth frame, and the
quaternion is earth referenced there. The number tells you who commanded last.

The arithmetic, with the airframe at roll 20, pitch 10, yaw 40 degrees and the
camera 30 degrees down and 25 degrees right of the nose, which moves all three
axes:

| the report | flags | the camera frame MAVInsight builds |
|---|---|---|
| stock | 32 | 44.0 degrees away from the truth |
| patched | 44 | exact |

Across 2000 random attitudes the stock error reached 180 degrees, and the
patched error stayed at zero.

This patch alone does not make the simulated gimbal hold the horizon. The lock
bits report the mode that was commanded, as a real device does, while the
quaternion reports where the camera is, so before the lock patch the report
showed the roll and pitch error the first section describes. That is the right
order to read them in: this patch makes the report honest, and the lock patch
makes the joints match it.

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

## The yaw axis, and the three places that state its travel

The Chimera mount is three axis: yaw, roll and pitch. The camera takes an
azimuth off the nose without the aircraft turning, and a pointing click that
lands left or right of the boresight is a yaw command like any other.

Three places state the travel of that axis, and they have to agree. `model.sdf`
gives the yaw joint -180 to +180 degrees. `px4-gzgimbal-lock.patch` reports the
same range in GIMBAL_DEVICE_INFORMATION, which is also what GZGimbal clamps the
joint command to. `yaw_angle_min` and `yaw_angle_max` in the flight code
parameters bound what `umd_uas/gimbal.py` will ask for. Change one and change
all three, or the mount stops at a limit that nothing else knows about.

That is exactly how this axis was lost. `model.sdf` was given equal yaw limits,
which pins the arm at zero while the joint and its controller stay in place, so
PX4 still found everything it commanded and nothing failed. The flight code went
on sending an azimuth, the report went on declaring an infinite yaw, and the
camera answered in pitch alone. A click left of the boresight moved the picture
down. **A locked joint is silent: the command is accepted, clamped and gone.**

The capability flags are the honest half of it. Stock GZGimbal declared
HAS_YAW_AXIS, HAS_YAW_FOLLOW and SUPPORTS_INFINITE_YAW while the joint could not
move at all. The patch keeps the first two, drops SUPPORTS_INFINITE_YAW because
the travel is now a range rather than an infinite one, and adds HAS_ROLL_LOCK,
HAS_PITCH_LOCK and HAS_YAW_LOCK, because the device holds all three axes against
the world once it honours the lock bits.

Nothing in PX4 reads those flags. They are what a consumer reads to learn the
mount, and `umd_uas/gimbal.py` prints them from `gimbal_control/device/info` and
says out loud when they disagree with its own yaw limits. On the simulator that
topic is advertised and carries nothing, because PX4 sends
GIMBAL_DEVICE_INFORMATION only when something asks for it and no stream rate
does, so read the flags from the aircraft and the travel from this file.

An azimuth past the travel is still the aircraft's work. `umd_uas/gimbal.py`
clips the demand at its own limits and says how far the aircraft has to turn for
the rest; nothing in the flight code turns it.

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
| Executed commands vehicle relative, flags ignored | Apply the lock patch, then send the attitude a real device gets |
| Reads the setpoint in the aerospace angle order, which is not the order its arms turn in | Apply the lock patch, which reads it in the chain's order |
| Computes an ROI attitude once, earth referenced | Hold the place in the flight code, as gimbal.py does |
| Reported an absolute attitude with no lock bit | Apply the frame patch, then check the gimbal frame against the vehicle heading |
| Measures the patched report against the EKF airframe attitude | Build earth-frame poses from the report, build command state from your own setpoints |
| Runs its cycle at 5 Hz | Apply the rate patch, and stream the report at 50 Hz |

`./px4sim setup` applies the patches, and applying them again is safe:

```bash
./px4sim setup px4
```

It reads `patches/px4-*.patch` in name order, which is also the order they
depend on: the lock patch builds on the two helpers the frame patch adds, and
the rate patch stands alone. What it applied is written to
`src/PX4-Autopilot/.px4sim-patches`, one checksum for each patch, because two
of these patches change the same lines and a patch that is already in the tree
stops reversing cleanly once the next one lands. Reading the record instead of
the tree means a correctly patched tree is never reported as unpatched, and an
edited patch is applied again.

A tree that was patched by hand before that record existed is recognized and
recorded on the next run.

To EDIT one of these patches, put the files it touches back first. A new
version of a patch does not apply on top of the old one, and the record only
knows that the checksum changed:

```bash
git -C src/PX4-Autopilot checkout -- src/modules/simulation/gz_bridge
rm src/PX4-Autopilot/.px4sim-patches
./px4sim setup px4
```

Then restart `sim`. The entrypoint rebuilds what changed.
