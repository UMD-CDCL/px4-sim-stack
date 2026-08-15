# scenegen

Give it a coordinate and a side length. It builds a Gazebo scene of that
square from real map data: terrain from elevation tiles, the satellite
image draped over the terrain, OSM buildings as boxes, and detected
vehicles as models. A browser editor then fixes what the data got wrong.

The module runs on demand and exits. The sim never depends on it.

## The pipeline

| Stage | What it does | Output |
|---|---|---|
| `create` | Fetch imagery, elevation and buildings for the square | `data/<name>/scene.json` |
| `detect` | Find cars and buses in the imagery | vehicles in `scene.json` |
| `edit` | Serve the browser editor on port 8090 | your corrections in `scene.json` |
| `build` | Write the world, terrain model and casualty scenario | files in `modules/sim/scenes/` |

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
./px4sim genscene detect --name campus
./px4sim genscene edit   --name campus     # open http://localhost:8090
./px4sim genscene build  --name campus --casualties /data/my_casualties.yaml
```

The first `genscene` builds the image, which downloads the CPU torch wheel
and takes a few minutes. Paths you give the container, such as the
casualty file, must sit under a mounted directory: `/data` is
`modules/scenegen/data` and `/scenes` is `modules/sim/scenes`.

`build` prints .env lines. Put them in `.env`, then:

```bash
./px4sim scene campus          # restart the sim with the new world
./px4sim scenario              # place the casualties, no restart
```

The world and PX4 must agree on the origin. If you skip the printed
`HOME_*` lines, QGC and the map disagree with the terrain.

## The editor

Everything is graphical. You never edit a file to fix the scene.

- Drag a vehicle to move it. The round handle rotates, the square handle
  resizes, `F` flips it 180 degrees, `Del` removes it.
- `+ Car`, `+ Bus`, `+ Building`: click the map to place one. Hold shift
  to keep placing.
- Shift-click on the ground stamps a copy of the last vehicle you placed
  or selected. A copy takes the source's properties as they are at that
  moment, edits included. Stamp a parking row in a few clicks; a
  shift-drag grabs the row afterward when you want to adjust it as a
  group. Buildings do not stamp.
- Shift-drag draws a selection rectangle; shift-click adds or drops one
  object. A selection moves as a group. The white handle turns every
  selected object in place; positions hold. The group flips and takes
  class or model changes. Sizes stay as they are.
- Undo and redo cover every edit: `Ctrl+Z` and `Ctrl+Y`, or the arrow
  buttons in the toolbar. One drag is one step.
- Click a building to set its height, replace its box with a model URI,
  or take it out of the world (`Del` toggles an OSM building, removes a
  hand-placed one).
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

## Casualties

Casualties live in a YAML list, not in the world, so a layout change
needs no sim restart. `build --casualties <file>` converts them to a
scenario for `spawn_scenario.py`:

```yaml
casualties:
  - lat: 38.98625        # required
    lon: -76.94330       # required
    alt: null            # AMSL meters. Absent or null: on the terrain.
    model: null          # model URI. Absent: drawn from the pool, seeded.
    name: null           # keep "casualty" or "person" in it; the scorer
                         # only counts names that carry one
```

See `examples/casualties_example.yaml`.

## Data sources

| Layer | Source | Note |
|---|---|---|
| Imagery | Google satellite tiles (default) or Esri | Check the imagery terms for your use. `--imagery esri` switches. |
| Elevation | AWS terrain tiles, terrarium encoding | Public, no key. About 4 m per pixel. |
| Buildings | OSM Overpass | Height from the `height` tag, else levels x 3.2 m, else 6 m. |
| Vehicle models | Gazebo Fuel | Downloaded by the sim on first world load. |

## Known limits

- The ROS localization nodes assume flat ground at z=0. The build report
  warns when the terrain spans more than a few meters; flatten zones can
  level the areas you fly over.
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
