# Deferred to a later development pass

Each entry is a cut we chose, with the reason and the seam where it plugs
back in. An entry here is a decision, not an omission.

## Cut in the first pass

| Feature | Why it waits | Where it plugs in |
|---|---|---|
| Automatic overhang detection | `bridge=yes` and `man_made=canopy` tags could seed flatten zones. The manual flatten-zone tool covers the need first. | `sources.py` and `scene_model.py` |
| Vehicle models scaled to the detected size | SDF `<include>` cannot scale a model. A generated per-instance wrapper could. Each vehicle keeps its detected size in `scene.json`, unused for now. | `build_world.py: vehicle emitter` |
| Vehicle front against back | Overhead imagery fixes the axis, not the direction. A heading can sit 180 degrees off. The editor has a flip button. | `detect_vehicles.py` |
| Collision mesh decimation | Visual and collision share one terrain mesh. That is fine for one drone. Decimate when a scene hosts many contacts. | `terrain_mesh.py` |
| Flatten zone edge blending | A zone steps sharply at its border. A blend band would look better. | `terrain_mesh.py` |
| Trees from OSM | `natural=tree` and `landuse=forest` sit one Overpass query away. | `sources.py`, then a pool of Fuel tree models |
| Lean of tall buildings | Satellite tiles lean tall roofs away from nadir. A roof-drawn footprint lands a few meters off at ground level. Nudge it in the editor. | documentation only |

## Found while testing

| Feature | Why it waits | Where it plugs in |
|---|---|---|
| Detector recall tuning | Confidence 0.10 with the DOTA model finds most vehicles, and the editor absorbs the rest. Zoom-20 imagery or an aerial-trained model would lift recall. Not measured yet. | `detect_vehicles.py` tunables |
| Fuel model heading offsets for the cars | Each Fuel vehicle has its own forward axis. The offset table exists and the Bus carries its measured 90 degrees. The car models look right so far; measure and add an offset when one spawns sideways. | `build_world.py: VEHICLE_MODEL_YAW_OFFSET_DEG` |

## Cut with the extruded buildings

| Feature | Why it waits | Where it plugs in |
|---|---|---|
| Wall textures from imagery | Satellite pixels see roofs, not walls. A wall texture would smear the roof edge downward. The flat per-building gray reads better until oblique imagery exists. | `build_world.py: write_buildings_model` |
| Hole editing in the editor | Holes come from OSM and ride along with every rectangle edit. Drawing or reshaping one by hand needs a polygon tool the editor does not have. | `editor.html`, next to the flatten-zone tool |
| Outline redraw for scenes edited before the sync | The editor now moves the outline with the rectangle. A building nudged before that keeps its outline where it was; the build corrects it through the fitted-rectangle transform, but the editor draws the stored points. | `editor.html: draw`, with a JS minimum-area rectangle |
| Roof detail above the sampling grid | The Foxglove roof colors sample one satellite pixel per vertex on an 8 m grid. Finer detail means more vertices in every latched message. | `build_world.py: ROOF_SAMPLE_EDGE_M` |
| School buses, semis and tankers | Fuel has no such model from a credible owner (catalog searched 2026-08-16), only toy-scale scans. The bus pool carries the Bus, the box truck and the fire truck until real models appear. | `build_world.py: VEHICLE_MODEL_POOLS` |
