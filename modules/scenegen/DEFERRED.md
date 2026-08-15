# Deferred to a later development pass

Each entry is a cut we chose, with the reason and the seam where it plugs
back in. An entry here is a decision, not an omission.

## Cut in the first pass

| Feature | Why it waits | Where it plugs in |
|---|---|---|
| Extruded meshes for L-shaped buildings | The first pass boxes each footprint with its minimum-area rectangle. A concave footprint needs triangulated extrusion and roof geometry. `scene.json` keeps the full outline per building, so the data survives. | `build_world.py`, next to the box emitter |
| OSM multipolygons with holes | Ways and simple outer rings cover most buildings. Courtyard holes need ring assembly and even-odd triangulation. The fetch report counts the skipped rings. | `sources.py: fetch_osm_buildings` |
| Automatic overhang detection | `bridge=yes` and `man_made=canopy` tags could seed flatten zones. The manual flatten-zone tool covers the need first. | `sources.py` and `scene_model.py` |
| Vehicle models scaled to the detected size | SDF `<include>` cannot scale a model. A generated per-instance wrapper could. Each vehicle keeps its detected size in `scene.json`, unused for now. | `build_world.py: vehicle emitter` |
| Vehicle front against back | Overhead imagery fixes the axis, not the direction. A heading can sit 180 degrees off. The editor has a flip button. | `detect_vehicles.py` |
| Collision mesh decimation | Visual and collision share one terrain mesh. That is fine for one drone. Decimate when a scene hosts many contacts. | `terrain_mesh.py` |
| Flatten zone edge blending | A zone steps sharply at its border. A blend band would look better. | `terrain_mesh.py` |
| Trees from OSM | `natural=tree` and `landuse=forest` sit one Overpass query away. | `sources.py`, then a pool of Fuel tree models |
| Lean of tall buildings | Satellite tiles lean tall roofs away from nadir. A roof-drawn footprint lands a few meters off at ground level. Nudge it in the editor. | documentation only |
| Casualties on rooftops | The build uses a given `alt` as is, and terrain height otherwise. It does not place a casualty on a detected roof by itself. | `build_world.py: casualty emitter` |

## Found while testing

| Feature | Why it waits | Where it plugs in |
|---|---|---|
| Detector recall tuning | Confidence 0.10 with the DOTA model finds most vehicles, and the editor absorbs the rest. Zoom-20 imagery or an aerial-trained model would lift recall. Not measured yet. | `detect_vehicles.py` tunables |
| Casualties in the editor | Casualties come from a YAML at build time, and the editor does not draw them yet. A read-only layer would help placement near cover. | `editor.html` and a `/casualties` endpoint |
| Editor undo | Each save keeps one backup, `scene.json.bak`. The editor itself has no undo. | `editor.html` |
| Fuel model heading offsets | Each Fuel vehicle has its own forward axis. A per-model yaw offset table would correct a model that spawns sideways. All offsets stay 0 until someone measures them in the sim. | `build_world.py: VEHICLE_MODEL_POOLS` |
