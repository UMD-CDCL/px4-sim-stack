# Bench mode

Bench mode is a guarded, GPS-free test configuration for a real aircraft or
ground station on a bench. Enable it only while the stack is stopped through
`./px4sim ui`'s stack menu or:

```text
./px4sim bench enable "ENABLE BENCH MODE"
```

When enabled, the real-aircraft launch:

- publishes a fixed home reference at the UROC scene origin.
- uses the UROC scene terrain and the nearby surveyed UROC fiducial.
- supplies the configured home and fiducial references to localization and
  Foxglove, so the vehicle appears near the fiducial.
- keeps the normal camera, detector, gimbal, and visualization paths running.

`./px4sim ui` shows a persistent red `BENCH MODE ENABLED — NOT FOR FLIGHT`
banner on both the aircraft and ground station while the setting is enabled.

The stored bench altitudes are WGS84 ellipsoid heights. They combine the UROC
mean-sea-level heights with the site geoid offset. They match `NavSatFix` and
the terrain renderer.

Bench mode disables no flight software. It replaces the missing ROS home and
fiducial reference at the localization input. It does not publish fake global
GPS. It does not arm, move, or alter PX4 state. Do not use it for flight.

It is not a navigation or flight-safety aid. Disable it before flight:

```text
./px4sim bench disable
```

Stop the stack before either transition. The `BENCH_MODE` setting stays in
`.env`. Bench mode also writes the UROC scene selection.
