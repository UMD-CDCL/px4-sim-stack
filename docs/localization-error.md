# Localization error

This page says where the error in a reported target position comes from. It
says what each part of it costs, and which parts are floors that no change to
the code will move.

Every number here was measured on uas11 over the `lorton` scene, against the
ground truth of the scenario, with `./px4sim uas 11 record`. Two geometries
appear. One is the wide framing at 20 m, and the other is the mid framing at
40 m. Both of them look 45 degrees under the horizon at the casualties.

## Measure it before you change it

```bash
./px4sim uas 11 record --seconds 40 > run.tsv
```

The command writes one row for each localized box. The error columns and the
motion that caused them sit on the same row. So you can judge a change by what
it does to `err_horiz` at a given `pitch_rate`, and never by one reading.

Two readings matter:

- **The settled floor.** Hover, and hold the gimbal still. This is the
  detector, the lens and the ground. If a change moves this, it changed the
  geometry and not the timing.
- **The slope against `pitch_rate`.** Sweep the gimbal, then fit `err_north`
  against the signed rate. Divide the slope by the metres for each degree that
  the row itself gives. The answer is in seconds, and it is the lag between the
  instant of the picture and the instant that the pose is read at.

**A fault in the timing lifts the slope and leaves the floor alone. A fault in
the geometry does the opposite.** One number alone tells you neither.

`slew_rate` is a magnitude, because a boresight can turn about two axes at
once. `pitch_rate` carries the sign, and the sign is what tells a lag from
noise. A late stamp moves each ray the way that the boresight goes, so the
error changes sign when a sweep reverses.

## What the fault in the timing cost, and what closed it

The stamp on a frame decides which pose of the camera the ray is cast from.
Here are three answers to "when was this picture taken", each measured the same
way:

| The stamp names | Lag | Error at 30 deg/s |
|---|---|---|
| when the decoded frame reached the detector | +99 ms | 1.54 m |
| when the frame left the video router | +33 ms | 0.61 m |
| when the camera rendered the frame | -12 ms | **0.40 m** |
| *hover, to compare* | | *0.32 m* |

The 12 ms that remain are the transport of the gimbal report. It shows now
because the stamp on the frame no longer hides it. A slew at 30 degrees a
second costs 0.08 m over the settled floor, which is inside the noise between
one run and the next.

The last row of the table needed 33 bytes on each picture. That is 4.0 kbit/s,
or 0.05 percent of the stream. Nothing outside the bitstream survives the trip,
because the router makes the RTCP sender report again from a clock of its own.

## The rest of the budget

| Cause | What it costs | State |
|---|---|---|
| the stamp on a frame is not the instant of capture | 1.2 m at 30 deg/s | fixed |
| the calibration changed before the lens moved | 13.7 m, for about a second after a recall | fixed |
| `scene` left the companions on the terrain of the scene before it | up to 17 m, and silently | fixed |
| each frame was localized and published twice | 14.4 percent were duplicates, and the scorer dropped 19 percent | fixed |
| the scorer named the nearest target | an error of 18.9 m was reported as 2 m | fixed |
| the camera leaves the terrain tile | up to 9.5 m, as a cliff and not a slope | deferred |
| the lock flags change on another edge | 1 to 2 frames, and up to 28 degrees at 20 degrees of bank | deferred |
| the attitude goes around through the gimbal report | 0.2 mm at a hover | recorded |

## Floors. Stop here.

Each of these was measured. None of them is worth another day.

| Floor | How big | Why it cannot move |
|---|---|---|
| the size of a pixel on the ground | 0.043 m across the view, and 0.063 m along it | Pixels. The only lever is the input size of the detector, and not the size of the published image. |
| the jitter of a box | about 1 pixel rms on the anchor | The edge of a box is where the model says it is. |
| the report of the gimbal is quantized | to 0.6 degrees | That is what the report carries. |
| where the lens comes to rest | 0.42 percent off nominal, or 0.04 to 0.14 m | The emulator models coast and backlash faithfully. It is why you calibrate a real lens at its preset. |
| the grid of the terrain, and the march of a ray | under 0.2 m | Cells of 1.5625 m, read bilinear, then bisected to micrometres. |
| the curve of the Earth over the scene | under 0.1 m | The scene is 200 m of ground. |
| the real time factor of the simulator | 0.63 to 0.86, and it wanders inside 100 ms | The host, a step of 4 ms, and four cameras on one GPU. It is a factor to convert by, and not an error. Take any arithmetic between sim time and wall time through `/world/<name>/clock`, and never through a nominal factor. |

Two traps look like floors and are not:

- **The clock of PX4 is sim time.** It runs at 0.63 to 0.76 of the wall clock
  here, and it drifts 8 s in 45 s. The MAVROS `sys_time` plugin would move each
  frame in the tree onto that clock, while the stamps on the pictures stay on
  wall time. It reads as an obvious fix for accuracy. It would destroy
  localization quietly.
- **The rangefinder never enters the cast of a ray.** It is telemetry only, and
  it saturates at 50 m. A higher rate buys latency and robustness, and not
  accuracy.

## Deferred, with the seam it plugs back into

An entry here is a decision, and not an omission.

| Item | Why it waits | Where it plugs in |
|---|---|---|
| the lock flags are applied on the edge of the pose | The flags are latched with no stamp of their own, and applied on the next pose. So the two edges change convention up to one pose apart, and tf2 interpolates across the switch. Only flags 44 appeared in 6197 samples, so the profile we fly never reaches it. | `MAVInsight/models/sensor.py`. Publish the reference edge from the callback for the gimbal status, with the flags and the stamp of that message. |
| `delta_yaw` from the report of the gimbal | It would cancel the round trip of the attitude exactly. It is on the wire, and absent from `mavros_msgs`. To carry it means a change to a message upstream, a change to a plugin, and a rebuild of each container that holds `mavros_msgs`. It buys 0.2 mm at a hover. | upstream `mavros_msgs`, then `MAVInsight/models/sensor.py` |
| a ray that leaves the tile | The surface is 200 m across, and the first tile whose square holds the origin of the ray wins. Past it each ray meets the flat plane, which is a cliff of up to 9.5 m at 45 degrees, and nothing reports it. | the choice of a tile in `5g_drone/umd_uas/terrain.py`, or a wider surface out of `scenegen build` |
| what `target_locations/for_air` really costs | The stripped message measures 5128 B with seven boxes. So the topic for the air carries 479 kbit/s at the full rate, and not the 38 kbit/s that the contract names. The figure in the contract was worked out for one box. The vehicle used to deliver half the frames only because the localizer was saturated. | `5g_drone/umd_uas/image_strip.py`, and the table in `uas-contract.md` |
| the wide framing over about 20 m | At 40 m a casualty is about 4 by 15 pixels, and the detector does not fire. Wide works at 20 m. This is the lens and the model, and not a fault. | the profile you fly, or the input size in `ds_ros_pipeline/infer_configs.py` |
| how `image_rehydrate` pairs a frame | It refills the image of a localization from a preview frame up to 0.5 s away. The coordinates are right and the picture is a neighbour, which makes flight under motion look worse than it is. | `5g_drone/umd_uas/image_rehydrate.py` |
