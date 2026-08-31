# Which altitude the ground is measured from, 2026-08-30

A record of one pass over a systematic localization bias in the simulator: how
it was measured, what it turned out to be, and the several things it was not.
[localization-error.md](localization-error.md) holds the error budget and
[localization-report.md](localization-report.md) the timing work that preceded
this. Read those first.

## The finding

A fiducial survey of a marker standing at its own surveyed coordinate, where
the correction must come out at zero, published this instead:

    d11_fiducial_offset -> uas11_home_position   0.00  -0.35  0.01

**Every localization lands about 0.35 m further from the vehicle than it
should, because the ray starts about 0.31 m higher than the camera really is.**
The two agree to 4 cm, which is what closes the case.

The error is in the ALTITUDE the ray is cast from, not in the ray, the
intrinsics, the terrain or the box. It is deterministic: the same geometry
gives the same answer to the centimetre, run after run and across respawns.

## How it was pinned

The bias scales as one over the tangent of the depression angle and does not
move with height. That pair is the whole diagnosis: it is a vertical offset
between the ray origin and the surface, not an angular error.

| depression | correction north | error | implied height error |
|---|---|---|---|
| 30 degrees | 8.36 | -0.64 | -0.370 |
| 45 degrees | 8.63 | -0.37 | -0.370 |
| 60 degrees | 8.79 | -0.21 | -0.364 |

against an expected 9.00, with the marker stood 6 m east and 9 m north of its
survey. Flying the same 45 degrees at 12 m and at 28.6 m gave -0.41 and -0.42.
An angular error would have given -0.21 and -0.62 there. A height error gives
what was seen.

The absolute check needs a vertical reference the estimator does not supply.
The downward rangefinder is the only one the vehicle carries:

    rangefinder, gimbal at -90        19.530 m above the ground below
    terrain_z under the vehicle       +0.250 m
    scene datum, world z = 0           8.1432 m ellipsoidal
    => camera really at               27.9232 m ellipsoidal

    home_position/fix altitude         8.4662 m
    tf home_position -> d11_rgb_offset 19.7698 m
    => localizer believes             28.2360 m

## Where the 0.31 m comes from

Two errors of the same sign, and both are in the altitude chain.

**home_position/fix reads about 0.13 m high.** PX4 latches home when the
vehicle arms, from an estimate that is still settling, and it latches it at
`base_link`, which this airframe carries 0.24 m above the model origin while it
sits on the ground. The simulator spawns and arms identically every run, so the
error is not noise: it reproduces exactly.

**The tf chain reads about 0.18 m high** between home and the camera, which is
the EKF's own local solution plus the bookkeeping between two origins that do
not coincide.

Those origins are worth stating plainly, because the topics do not agree on
which they use:

    local_position/pose     z = 20.171   in uas11_ekf_origin
    global_position/local   z = 20.013   in uas11_home_position
    home_position/home  position.z = 0.157, the gap between them

Measured against Gazebo at one hover, `ekf_origin` sat at world +0.031 and
`home_position` at world +0.189. `mavros/altitude` offers six more numbers on
top of those two: `monotonic` 64.186, `amsl` 63.976, `local` 20.170,
`relative` 20.013, `terrain` 0.003, `bottom_clearance` 20.167. Anything that
picks the wrong one inherits a fixed offset and never says so.

**The ground is not flat at the origin either.** The terrain stood at +0.250 m
under the vehicle and at 0.000 m under the marker, 17 m apart. Treating height
above home as height above the ground the target stands on collects that
difference directly.

## What it was not

Each of these was measured and cleared, and each looked plausible first.

- **Not the ray direction.** Height independent, as above.
- **Not the marker.** It is a 2 cm disc lying flat, and the fiducial path
  anchors on the box centre, which is right for it.
- **Not the bounding box anchor.** `bbox_anchor` is centralised in
  `umd_uas/bbox.py` and has exactly one caller, at the single point a box
  becomes a ray. Nothing bypasses it.
- **Not the terrain model.** It matches Gazebo to 2 cm at the marker, and the
  ray lands at the right altitude: the node logs a surveyed 8.13 m against a
  measured 8.12 m.
- **Not the geoid.** `site-params.sh` computes the separation from the scene's
  own origin with GeoidEval and exports it. It read -35.4968 m, which agrees
  with the authority to 1 cm.
- **Not, in the end, the camera mounting**, though that was genuinely wrong and
  is now fixed. The simulated airframe carries its gimbal 74 mm further back
  and 78 mm lower than the aircraft, with the camera on a 178 mm arm rather
  than at the pivot. MAVInsight now describes it separately. Correcting a
  quarter of a metre there moved the bias by 0.05 m, because the altimeter
  plane moves with the camera and the shift is largely common mode.

## The trap

Most of the obvious cross-checks are the estimator restating itself.
`global_position/global` altitude equals the home fix plus the local solution
at every sample, so it agrees with them by construction and confirms nothing.
The same holds on the aircraft, where it was measured as exactly `ekf_z` plus a
constant.

**The downward rangefinder is the only ground-independent vertical reference
the vehicle carries.** It is what closed this, and it is the only instrument
that would have caught it.

## What to do

The ground frame anchors its altitude on `home_position/fix`, a GNSS height
latched at arming. The terrain it casts against is anchored on the scene's
surveyed datum, which is exact. The error is the gap between those two datums.

1. **Anchor the ground frame's altitude on the terrain rather than on home.**
   The ray and the surface then share one datum and both terms go away.
2. **Re-derive home's altitude once airborne**, from a settled estimate, rather
   than trusting the arming latch. This helps everything downstream of home.
3. **Cross-check against the rangefinder** at known gimbal angles, which is the
   only check that can detect this class of error rather than absorb it.

On the aircraft there is no surveyed terrain to lean on, so 2 and 3 are the
ones that transfer. See the note below on why this matters more there than
here.

## The same fault is larger on the aircraft, and it moves

A characterization of `rosbag2_2026_08_18-16_02_00-fid`, UAS3 at Webster Field,
found the EKF's vertical estimate sliding 2.353 m downward across one 640 s
flight while the drone sat on the same ground at both ends, the downward lidar
reading 0.040 m each time. The EKF tracked raw GNSS height to within 5 cm; the
barometer over the same interval moved only 0.781 m. Downstream it biased
altimeter plane localizations 3.17 m toward the drone, and recomputing them
against barometer height cut that to 0.96 m.

Two differences from what is written above matter.

**It is larger, and it is the other way round.** The simulator reads about
0.31 m high and so overshoots. That flight read low and so undershot, toward
the vehicle.

**It moves.** The simulator's error is latched at arming and constant. The
aircraft's grows through the flight. A fiducial survey publishes ONE static
correction, which is the right shape for a constant offset and the wrong shape
for a drifting one. No single survey can track it, however well it is flown.
