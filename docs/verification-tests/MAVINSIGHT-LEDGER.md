# MAVInsight feature ledger

This intentionally over-includes behavior inferred from the current package
under `.build-contexts/mavinsight/`. Verify each row against the checked-out
source and classify it using the README status tags.

`Status` records intended scope, not verification. Add `Tested: YES|NO|BLOCKED`
with date, commit, Git status, and evidence; this inventory defaults to
`Tested: NO`.

| Status | Feature or responsibility | Source evidence to inspect | Intended evidence |
|---|---|---|---|
| `CURRENT` | Site selection and site metadata | `models/site.py`, `sites/*.yaml`, `launch_site.launch.py` | Site loads with origin, datum, bounds, and labels |
| `CURRENT` | Terrain mesh visualization | `models/terrain_viz.py`, `models/scene_ground.py` | Terrain appears at correct coordinates |
| `CURRENT` | Building mesh visualization | `models/buildings_viz.py`, `models/gltf.py` | Buildings load, orient, and render |
| `CURRENT` | Building footprints | `models/footprint_viz.py` | Footprints align with scene/map |
| `CURRENT` | Scene textures/materials | `models/scene_texture.py` | Textures resolve without broken assets |
| `CURRENT` | Ground/scene frame construction | `models/scene_ground.py`, `models/frame_utils.py` | Frame tree is connected and stable |
| `CURRENT` | Vehicle model visualization | `models/vehicle.py`, `vehicles/*.yaml` | Vehicle marker/model follows telemetry |
| `PLANNED` | Chimera visual mesh | `resource/chimera_v3_viz.stl`, `models/platforms.py` | Correct platform geometry renders |
| `CURRENT` | Gimbal frame and orientation | `models/gimbal_frame.py`, `models/frame_member.py` | Gimbal axes agree with telemetry |
| `CURRENT` | Sensor model representation | `models/sensor.py`, `models/sensor_types.py` | Sensor frusta/labels render |
| `CURRENT` | Down-camera representation | `sensors/*down_camera.yaml` | Camera frame and field of view are correct |
| `CURRENT` | RGB gimbal-camera representation | `sensors/*gimbal_rgb_camera.yaml` | Lens/FOV and pose agree with image |
| `CURRENT` | Rangefinder representation | `sensors/*rangefinder.yaml` | Range ray/footprint is visible and positioned |
| `CURRENT` | Simulation sensor variants | `sensors/sim_*.yaml`, `vehicles/sim_vehicle.yaml` | Sim-specific frames do not leak into real config |
| `CURRENT` | TargetBoxArray visualization | `models/tba_viz.py`, `resource/*tba.yaml` | Boxes, labels, confidence, and IDs render |
| `CURRENT` | Target track history/lifetime | `models/tba_viz.py`, `resource/tf_loc_tba.yaml` | Track identity and stale-data behavior are clear |
| `CURRENT` | Location visualization | `models/location_viz.py` | Target/vehicle locations map correctly |
| `CURRENT` | Scoring visualization | `models/scoring_viz.py` | Rates and score series render with units |
| `PlANNED` | Platforms and vehicle categories | `models/platforms.py` | Platform style/color follows model |
| `CURRENT` | Frame graph/member generation | `models/graph_member.py`, `frame_member.py` | All required TF-like relationships are emitted |
| `CURRENT` | QoS profiles for visualization | `models/qos_profiles.py` | Visualization receives sensor and reliable data |
| `CURRENT` | Global node configuration | `resource/global_node_config.yaml` | Defaults and namespaces are applied |
| `ORPHANED` | Site launch: exists partially, but should come from scenes like those built from px4sim genscene as the single source | `launch/launch_site.launch.py` | Site-only launch starts expected nodes |
| `CURRENT` | Simulation launch | `launch/launch_sim.launch.py`, `vehicles/sim_vehicle.yaml` | Sim launch consumes sim topics/config |
| `CURRENT` | Visualization launch | `launch/launch_viz.launch.py` | Viz launch starts all declared publishers |
| `CURRENT` | Real versus simulated sensor selection | launch arguments and sensor YAMLs | Correct variant is selected by mode |
| `CURRENT` | Multi-vehicle namespaces | launch files, vehicle config | Multiple vehicles do not collide |
| `CURRENT` | Missing/stale model handling | loaders in `models/` | Errors are explicit and do not silently render defaults |
| `CURRENT` | Coordinate transforms and geodetic conversion | `models/frame_utils.py`, `site.py` | WGS84/site/map transforms agree |
| `CURRENT` | Resource path portability | package resource/install metadata | Installed package works outside source tree |
| `CURRENT` | Foxglove-compatible output schemas | model publishers and layout consumers | Advertised schemas match panel expectations |

## Future additions to consider

| Status | Planned or suspected feature | Why it may matter |
|---|---|---|
| `CURRENT` | Explicit visualization health/status for use in foxglove (likely as table for easy display), px4sim ui, and 5g send as uav status or whatever,  (move to different ledger if it fits better), should include differentiate between not started, starting but initializing, running, stopped due to error, and stopped intentionally | Makes stale bridges and publishers visible |
| `CURRENT` | Multi-vehicle comparative view | Current configs suggest fleet operation |
| `CURRENT` | Recording/replay of visualization inputs | Needed for deterministic regression tests |
| `CURRENT` | Real-aircraft sensor calibration profiles | Separates sim assumptions from aircraft geometry |