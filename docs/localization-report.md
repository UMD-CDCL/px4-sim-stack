# What the localization work changed, 2026-08-21

A record of one pass over localization accuracy: the root cause, what each
change bought, and what to know before the next run.
[localization-error.md](localization-error.md) holds the error budget itself
and is the page to read first.

## The root cause

Localization was good at a hover and poor while the drone moved or the gimbal
slewed. **The stamp on a detection did not name the instant the shutter
opened.** `tf_loc` looks every transform up at that stamp, so the error was
that lag times the rate the view moved at.

Nothing was wrong with the ray math, the intrinsics or the terrain, and all
three were suspected. Three measurements agreed on the stamp instead:

- the slope of the error against the gimbal rate,
- direct instrumentation of the video path,
- the same cast run again with the stamp shifted by hand.

It took two changes, because no absolute time survives the video router. The
router makes the RTCP sender report again from a clock of its own.

1. Take the instant from the buffer pts, and not from `time.time_ns()` at
   ingest. This removes the decode leg.
2. Write the true render instant into each picture, as an H.265 SEI. That costs
   33 bytes for each frame, or 4.0 kbit/s, which is 0.05 percent of the stream.

Measured at the wide framing, 20 m over the ground, 45 degrees under the
horizon, against the ground truth of the scenario:

| | Lag | Error at 30 deg/s |
|---|---|---|
| before | +99 ms | 1.54 m |
| after the first change | +33 ms | 0.61 m |
| after the second | **-9 ms** | **0.41 m** |
| *hover, to compare* | | *0.38 m* |

A slew at 30 degrees a second now costs 0.03 m over the settled floor. Motion
across the ground costs 0.04 m. The error is flat against motion.

## The other faults, and what each cost

| Fault | What it cost |
|---|---|
| the calibration changed about a second before the lens did | 13.7 m during a recall, now 0.75 m |
| `scene` left the companions on the terrain of the scene before it | up to 17 m, and nothing reported it |
| each detection was localized and published twice | 14.4 percent were duplicates, the scorer dropped 19 percent, and `tf_loc` sat on a whole core |
| `./px4sim zoom` printed success without asking the vehicle | every test at a named framing was invalid |
| `./px4sim uas <N> heading` raised NameError on each call | two checks in the ground stage could only fail |
| a transform went out at 20 Hz with no stamp | tf2 could never serve it, and it was 11 percent of `/tf` |

After the single door for localization, the vehicle delivers one localization
for each detection: 5.87 Hz became 11.64 Hz, `tf_loc` fell from a whole core to
53 percent, and duplicates and frames out of order both went to zero. `/tf`
fell from 152.6 to 80.0 Hz while it carried one more transform.

## What was measured and left alone

- The attitude of the airframe goes around through the gimbal report and does
  not cancel. It was suspected at 0.5 to 3 m. It measures **0.2 mm** at a
  hover, and it scales with the yaw rate of the airframe and not with the
  gimbal rate, so it explains none of the complaint.
- A higher rate for the rangefinder buys no accuracy at all. The range never
  enters the cast of a ray.
- The size of a pixel on the ground, the jitter of a box, the backlash of the
  lens and the grid of the terrain are floors.
  [localization-error.md](localization-error.md) gives the arithmetic.

## Two traps

- **The MAVROS `sys_time` plugin must stay off.** PX4 SITL runs in lockstep
  with Gazebo, so its clock is sim time: 0.63 to 0.76 of the wall clock, and it
  drifted 8 s in 45 s. The plugin would move each stamp in the tree onto that
  clock while the pictures stay on wall time.
- **A higher rate for `GIMBAL_DEVICE_ATTITUDE_STATUS` would have made the error
  worse**, by about 12 percent, until the stamp on the picture was honest. The
  age of a packed message is phase locked at 0.28 ms, not the 14 ms that a
  reading of the nominal rate suggests.

## Before the next run

- **The wide framing gives no detections at 40 m.** A casualty is about 4 by 15
  pixels there, and the detector does not fire. Wide works at 20 m. Every
  earlier test at "wide" really ran at mid, because the zoom command was
  failing without saying so.
- **MAVROS died once** in this session, with a heap abort. Nothing here touches
  it. It takes the whole frame tree down and it reads as broken localization,
  so check that the node is alive before you believe a bad number.
- Measure with `./px4sim uas <N> record`. Read the settled floor and the slope
  against `pitch_rate` apart from each other.

## State at the end

`./px4sim verify` passes 71 of 71. It passed 67 of 71 at the start of the run,
and two of those four failures were faults this work introduced and the suite
caught. `./px4sim check` passes.

One decision is open. `target_locations/for_air` carries 479 kbit/s with seven
boxes in the frame, and the contract names 38 kbit/s. That figure was worked
out for one box, and the vehicle used to deliver half the frames only because
the localizer was saturated. To thin that topic is a change to the contract, so
it sits in the deferred table and not in the code.
