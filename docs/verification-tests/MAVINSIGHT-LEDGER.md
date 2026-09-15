# MAVInsight feature ledger

This intentionally over-includes behavior inferred from the current package
under `.build-contexts/mavinsight/`. Verify each row against the checked-out
source and classify it using the README status tags.

| Status | Feature or responsibility | Source evidence to inspect | Intended evidence |
|---|---|---|---|
| `UNKNOWN` | Site selection and site metadata | `models/site.py`, `sites/*.yaml`, `launch_site.launch.py` | Site loads with origin, datum, bounds, and labels |
| `UNKNOWN` | Terrain mesh visualization | `models/terrain_viz.py`, `models/scene_ground.py` | Terrain appears at correct coordinates |
| `UNKNOWN` | Building mesh visualization | `models/buildings_viz.py`, `models/gltf.py` | Buildings load, orient, and render |
| `UNKNOWN` | Building footprints | `models/footprint_viz.py` | Footprints align with scene/map |
| `UNKNOWN` | Scene textures/materials | `models/scene_texture.py` | Textures resolve without broken assets |
| `UNKNOWN` | Ground/scene frame construction | `models/scene_ground.py`, `models/frame_utils.py` | Frame tree is connected and stable |
| `UNKNOWN` | Vehicle model visualization | `models/vehicle.py`, `vehicles/*.yaml` | Vehicle marker/model follows telemetry |
| `UNKNOWN` | Chimera visual mesh | `resource/chimera_v3_viz.stl`, `models/platforms.py` | Correct platform geometry renders |
| `UNKNOWN` | Gimbal frame and orientation | `models/gimbal_frame.py`, `models/frame_member.py` | Gimbal axes agree with telemetry |
| `UNKNOWN` | Sensor model representation | `models/sensor.py`, `models/sensor_types.py` | Sensor frusta/labels render |
| `UNKNOWN` | Down-camera representation | `sensors/*down_camera.yaml` | Camera frame and field of view are correct |
| `UNKNOWN` | RGB gimbal-camera representation | `sensors/*gimbal_rgb_camera.yaml` | Lens/FOV and pose agree with image |
| `UNKNOWN` | Rangefinder representation | `sensors/*rangefinder.yaml` | Range ray/footprint is visible and positioned |
| `UNKNOWN` | Simulation sensor variants | `sensors/sim_*.yaml`, `vehicles/sim_vehicle.yaml` | Sim-specific frames do not leak into real config |
| `UNKNOWN` | TargetBoxArray visualization | `models/tba_viz.py`, `resource/*tba.yaml` | Boxes, labels, confidence, and IDs render |
| `UNKNOWN` | Target track history/lifetime | `models/tba_viz.py`, `resource/tf_loc_tba.yaml` | Track identity and stale-data behavior are clear |
| `UNKNOWN` | Location visualization | `models/location_viz.py` | Target/vehicle locations map correctly |
| `UNKNOWN` | Scoring visualization | `models/scoring_viz.py` | Rates and score series render with units |
| `UNKNOWN` | Platforms and vehicle categories | `models/platforms.py` | Platform style/color follows model |
| `UNKNOWN` | Frame graph/member generation | `models/graph_member.py`, `frame_member.py` | All required TF-like relationships are emitted |
| `UNKNOWN` | QoS profiles for visualization | `models/qos_profiles.py` | Visualization receives sensor and reliable data |
| `UNKNOWN` | Global node configuration | `resource/global_node_config.yaml` | Defaults and namespaces are applied |
| `UNKNOWN` | Site launch | `launch/launch_site.launch.py` | Site-only launch starts expected nodes |
| `UNKNOWN` | Simulation launch | `launch/launch_sim.launch.py`, `vehicles/sim_vehicle.yaml` | Sim launch consumes sim topics/config |
| `UNKNOWN` | Visualization launch | `launch/launch_viz.launch.py` | Viz launch starts all declared publishers |
| `UNKNOWN` | Real versus simulated sensor selection | launch arguments and sensor YAMLs | Correct variant is selected by mode |
| `UNKNOWN` | Multi-vehicle namespaces | launch files, vehicle config | Multiple vehicles do not collide |
| `UNKNOWN` | Missing/stale model handling | loaders in `models/` | Errors are explicit and do not silently render defaults |
| `UNKNOWN` | Coordinate transforms and geodetic conversion | `models/frame_utils.py`, `site.py` | WGS84/site/map transforms agree |
| `UNKNOWN` | Resource path portability | package resource/install metadata | Installed package works outside source tree |
| `UNKNOWN` | Foxglove-compatible output schemas | model publishers and layout consumers | Advertised schemas match panel expectations |

## Future additions to consider

| Status | Planned or suspected feature | Why it may matter |
|---|---|---|
| `PLANNED` | Explicit visualization health/status panel | Makes stale bridges and publishers visible |
| `PLANNED` | Multi-vehicle comparative view | Current configs suggest fleet operation |
| `PLANNED` | Recording/replay of visualization inputs | Needed for deterministic regression tests |
| `PLANNED` | Real-aircraft sensor calibration profiles | Separates sim assumptions from aircraft geometry |
