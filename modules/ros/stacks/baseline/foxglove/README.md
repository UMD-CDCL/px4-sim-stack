# Foxglove layout

`px4-sim-stack.json` is the layout for this stack. It lives in the repository
so that the view is versioned with the code that feeds it.

## Use it

1. Start the stack with the ros profile.
2. Open the Foxglove desktop app, or the web app at
   <https://app.foxglove.dev>.
3. Connect to `ws://localhost:8765`.
4. Choose **Layout**, then **Import from file**. Pick `px4-sim-stack.json`.

## What you get

| Panel | Shows |
|---|---|
| 3D | The frame tree, the detection and truth spheres, each camera's footprint, and the live image projected onto the ground |
| Image, nadir | The downward camera, with detection boxes colored by verdict |
| Image, gimbal | The gimbal camera, with the same overlay |
| Map | The drone's own GPS fix on satellite imagery |
| Plot, scoring | Position error, recall and precision, per camera |
| Plot, altitude | Relative altitude and the rangefinder. The two agree over flat ground |

## Reading the 3D panel

Every person is a 1 m sphere. The color is the verdict, and the image boxes
use the same colors:

- **Blue, translucent** is ground truth: where the target really is, from the
  scenario file.
- **Green** is a true positive: an estimate within the gate of a target. A
  green sphere overlapping a blue one is a hit.
- **Red** is a false positive: an estimate with no target near it.
- **Yellow** is a false negative: a target inside the footprint that nothing
  found. It is drawn at the target, because there is no estimate to draw.
- The **cyan and orange outlines** are the ground each camera covers. The
  scorer counts only the targets inside a camera's outline, because the rest
  were never visible.
- The drone itself is its tf frames. There is no airframe model to draw.

## Every topic here is a stock ROS message

Nothing in this layout needs a Foxglove extension or a custom schema, except
`ImageAnnotations`, which is the message the Image panel reads for overlays.
The package is `ros-$ROS_DISTRO-foxglove-msgs`, from the normal ROS index.

| Topic | Type |
|---|---|
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` |
| `/ground_truth/markers` | `visualization_msgs/MarkerArray` |
| `/scoring/<camera>/markers` | `visualization_msgs/MarkerArray` |
| `/camera/<camera>/footprint` | `geometry_msgs/PolygonStamped` |
| `/camera/<camera>/image_raw/compressed` | `sensor_msgs/CompressedImage` |
| `/camera/<camera>/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/<camera>/annotations` | `foxglove_msgs/ImageAnnotations` |
| `/camera/<camera>/ground_projection` | `sensor_msgs/PointCloud2` |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` |
| `/scoring/<camera>/recall`, `precision`, `position_error` | `std_msgs/Float64` |

The image panels read the compressed topics. The raw ones are too large to
cross the websocket. See docs/troubleshooting.md.

## If a panel is empty

The layout format moves between Foxglove releases. A panel can come back with
default settings after an import, so check the topic on the panel first.

The other usual cause is that the stack does not publish the topic yet:

```bash
./px4sim shell ros
ros2 topic list
ros2 topic hz /perception/nadir/detections_3d
```

Ground truth appears once `ground_truth` reads the scenario. Verdict spheres
and colored boxes appear once detections arrive and the scorer compares them
against the truth.
