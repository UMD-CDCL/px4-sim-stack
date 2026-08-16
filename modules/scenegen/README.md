# scenegen

Give it a coordinate and a side length. It builds a Gazebo scene of that
square from real map data: terrain from elevation tiles, the satellite
image draped over the terrain, OSM buildings extruded from their
footprints with the same image over the roofs, and detected vehicles as
models. A browser editor then fixes what the data got wrong.

The module runs on demand and exits. The sim never depends on it.

## The pipeline

| Stage | What it does | Output |
|---|---|---|
| `create` | Fetch imagery, elevation and buildings for the square | `data/<name>/scene.json` |
| `import-casualties` | Load a lat/lon casualty file into the scene's targets | targets in `scene.json` |
| `detect` | Find cars and buses in the imagery | vehicles in `scene.json` |
| `edit` | Serve the browser editor on port 8090 | your corrections in `scene.json` |
| `build` | Write the world, the localization surface, the terrain and building models, the Foxglove building payload and the target scenario | files in `modules/sim/scenes/` |

Each stage is resumable. Run `detect` again after an edit and it replaces
only its own detections; hand-placed vehicles stay.

`all` runs the four stages in order and pauses at the editor. The build
starts when you click **Save & build** in the browser. Ctrl-C in the
terminal stops without building and prints the command that finishes the
job later.

## Quick start

```bash
./px4sim genscene all --name campus --center 38.9869,-76.9426 --side 600 \
    --casualties /data/my_casualties.yaml
```

Fix the scene in the browser on <http://localhost:8090>, click
**Save & build**, done. A re-run of `all` keeps an existing `scene.json`
and your edits in it, and skips detection once the scene holds vehicles;
`--force` refetches from scratch. The stages also run one at a time:

```bash
./px4sim genscene create --name campus --center 38.9869,-76.9426 --side 600
./px4sim genscene import-casualties --name campus --casualties /data/my.yaml
./px4sim genscene detect --name campus
./px4sim genscene edit   --name campus     # open http://localhost:8090
./px4sim genscene build  --name campus
```

The first `genscene` builds the image, which downloads the CPU torch wheel
and takes a few minutes. Paths you give the container, such as the
casualty file, must sit under a mounted directory: `/data` is
`modules/scenegen/data` and `/scenes` is `modules/sim/scenes`.

Git carries the scene sources (`scene.json`, the elevation grid and the
satellite image, per scene in `data/`), not the build products in
`modules/sim/scenes`. `./px4sim setup` runs `genscene build-all`, which
builds every scene in `data/` in one pass, so a fresh clone starts with
every scene in place. Run it yourself after a pull that brings new
scenes:

```bash
./px4sim genscene build-all
```

`build` prints the two .env lines that select everything:

```
SCENE=campus
SCENARIO=campus_casualties
```

The scenario file carries the world origin (`home_*`) and the survey
marker (`fiducial_*`); `px4sim` and `make` read them from it, so no
`HOME_*` or `FIDUCIAL_*` values move by hand. A hand-written scenario
without those lines leaves the `.env` values in force. Then:

```bash
./px4sim scene campus          # restart the sim with the new world
./px4sim scenario              # place the targets, no restart
```

The ros service reads the fiducial values when it starts. After you
switch to a scenario with a different fiducial, restart it:
`./px4sim restart ros`.

## The editor

Everything is graphical. You never edit a file to fix the scene.

- Drag a vehicle to move it. The round handle rotates, the square handle
  resizes, `F` flips it 180 degrees, `Del` removes it.
- `+ Car`, `+ Bus`, `+ Building`: click the map to place one. Hold shift
  to keep placing.
- Shift-click on the ground stamps a copy of the last vehicle or target
  you placed or selected. A copy takes the source's properties as they
  are at that moment, edits included; a target copy gets a fresh name,
  because the scorer treats the name as identity. Stamp a parking row or
  a cluster of casualties in a few clicks; a shift-drag grabs the row
  afterward as a group. Buildings do not stamp.
- Shift-drag draws a selection rectangle over vehicles, buildings and
  targets; shift-click adds or drops one object. A selection moves as a
  group. The white handle turns every selected object in place; positions
  hold. The group flips and takes class or model changes, and selected
  targets take floor, offset and scenario-inclusion changes together.
  Sizes and target names stay one at a time.
- Undo and redo cover every edit: `Ctrl+Z` and `Ctrl+Y`, or the arrow
  buttons in the toolbar. One drag is one step.
- Click a building to set its height, replace its mesh with a model URI,
  or take it out of the world (`Del` toggles an OSM building, removes a
  hand-placed one).
- `+ Target`: click to place a ground-truth casualty (green circle with
  its name). Drag to move it; the panel sets name, model, the floor it
  stands on (the terrain, or the building under it when snapped), an
  offset above that floor, and whether it enters the scenario. Imported
  targets appear the same way.
- `+ Flatten zone`: click the corners, double-click to close. The zone
  levels the terrain under it. This is the fix for a bridge or an
  overhang that the elevation data recorded as solid ground. On a
  selected zone, drag the body to move the whole zone, drag a corner to
  reshape it, double-click a corner to remove it, and double-click an
  edge to add one.
- Drag the orange fiducial to move the survey marker. Its coordinate
  lands in the printed `FIDUCIAL_SURVEYED_*` lines.

Save writes straight back to `scene.json`; the previous version becomes
`scene.json.bak`. Then run `build` again.

## Buildings

A building is its OSM footprint extruded to its height. A concave shape
stays concave, and a courtyard hole in the map becomes a hole in the
mesh. The roof carries the satellite image through the same georeference
as the terrain, so the real roof pixels sit on it. Walls get one flat
gray per building. The localization surface holds the same footprint, so
a detection ray into a courtyard lands on the ground inside it, not on a
phantom roof.

The scene square truncates every footprint. The terrain and the imagery
end at the square, so the part outside it is cut off rather than left to
float in the void. A building that stands entirely outside the square is
left out, and the build report counts the drops.

The editor edits the rectangle around the footprint. A drag, a turn or a
resize carries the outline and its holes along, and the build extrudes
the moved outline. A building placed by hand has no outline and builds
as its rectangle. A model URI set in the editor still replaces the whole
mesh with that model.

The Foxglove 3D panel shows the same buildings. The build writes
`worlds/<name>_buildings.json`, and the `scene_buildings` node publishes
it as one latched MarkerArray on `/scene/buildings`, with the roofs in
their satellite colors. Buildings only: vehicle props stay out of the
panel on purpose.

A scene fetched before courtyard support holds no holes in `scene.json`.
`create --force` refetches the footprints, but it also discards your
edits, so keep the scene as it is unless a courtyard matters.

## Ground-truth targets

`scene.json` is the source of truth for casualties. A lat/lon file is an
on-ramp, nothing more: import it once, and from then on the targets live
in the scene, the editor shows and moves them, and `build` writes the
scenario from the scene alone. One pipe:

```
casualty file --import--> scene.json --build--> scenarios/<name>_casualties.yaml
                          (the editor edits this)        (spawn + scorer read this)
```

```bash
./px4sim genscene import-casualties --name campus --casualties /data/my.yaml
```

`create --casualties` and `all --casualties` run the same import at
creation. A re-import replaces imported targets and keeps hand-placed
ones. The build draws nothing at random: a target without a model gets
one from the pool by a stable hash of its name, so a rebuild of an
unchanged scene writes the same bytes.

File format: `lat`/`lon` required; `agl` (meters above the terrain,
absent = on it), `model` and `name` optional. See
`examples/casualties_example.yaml`. Everything downstream is unchanged:
`spawn_scenario.py` places the scenario, so a target change after a
rebuild needs `./px4sim scenario`, not a sim restart. The scenario also
carries the `home_*` and `fiducial_*` lines the front doors read, and it
exists even for a scene with no targets, so SCENARIO always selects the
full configuration.

## Data sources

| Layer | Source | Note |
|---|---|---|
| Imagery | Google satellite tiles (default) or Esri | Check the imagery terms for your use. `--imagery esri` switches. |
| Elevation | AWS terrain tiles, terrarium encoding | Public, no key. About 4 m per pixel. |
| Buildings | OSM Overpass | Ways and multipolygon relations, courtyard holes included. Height from the `height` tag, else levels x 3.2 m, else 6 m. |
| Vehicle models | Gazebo Fuel | Downloaded by the sim on first world load. |

## Known limits

- The ROS localization nodes project onto a flat plane latched at the
  altitude the drone took off from. The build report warns when the
  terrain spans more than a few meters: expect offsets over ground far
  above or below the takeoff point, and use flatten zones to level the
  areas you fly over.
- A detected heading can be off by 180 degrees. Overhead imagery cannot
  tell front from back; the editor's flip button can.
- Detection favors recall: the default confidence is low because deleting
  a false box costs one keypress and placing a missed vehicle costs
  several. `--confidence` raises the bar.
- Tall buildings lean in satellite imagery, so a roof-drawn footprint
  sits a few meters off at ground level. Nudge it in the editor.

More cuts, each with its reason and its seam: [DEFERRED.md](DEFERRED.md).

## Development

The code is bind-mounted over the image, so an edit needs no rebuild.
To run outside docker: python3 with numpy, pillow, pyyaml, requests,
shapely, and ultralytics for `detect`. Tests carry their own ground
truth and run standalone:

```bash
python3 tests/test_geo.py        # geodesy against published constants
python3 tests/test_sources.py    # fetchers against known places (network)
python3 tests/test_build.py      # build against a hand-computed scene
```
