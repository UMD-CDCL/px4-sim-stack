# Bench mode

Bench mode is an explicitly guarded, GPS-free test configuration for a real
aircraft on the bench. Enable it only while the stack is stopped, through
`./px4sim ui`'s stack menu or:

```text
./px4sim bench enable "ENABLE BENCH MODE"
```

When enabled, the real-aircraft launch:

- publishes a fixed home reference at the UROC scene origin;
- uses the UROC scene terrain and the nearby surveyed UROC fiducial;
- supplies the configured home/fiducial reference to localization and the
  Foxglove scene so the vehicle appears near the fiducial;
- keeps the normal camera, detector, gimbal, and visualization paths running.

The stored bench altitudes are WGS84 ellipsoid heights (the UROC mean-sea-level
heights plus the site's geoid offset), matching `NavSatFix` and the terrain
renderer.

Bench mode disables no flight software, but it does replace the missing
ROS-side home/fiducial reference at the localization input. It does not
publish fake global GPS, arm, move, or alter PX4 state, and it must never be
used for flight. It is not a navigation or flight-safety aid. Disable it before
flight:

```text
./px4sim bench disable
```

The stack must be stopped before either transition. The `BENCH_MODE` setting is
persisted in `.env`; the UROC scene selection is also made explicit when bench
mode is enabled.
