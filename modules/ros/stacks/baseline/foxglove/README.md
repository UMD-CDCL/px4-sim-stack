# Foxglove layout

`px4-sim-stack.json` is the layout for this stack. It lives in the repository so
that the view is versioned with the code that feeds it.

## Use it

1. Start the stack with the ros profile.
2. Open the Foxglove desktop app, or the web app at
   <https://app.foxglove.dev>.
3. Connect to `ws://localhost:8765`.
4. Choose **Layout**, then **Import from file**. Pick `px4-sim-stack.json`.

## What you get

| Panel | Shows |
|---|---|
| 3D | The airframe and its frames. The camera footprint. Ground truth in green, estimates in orange, and a line between each match |
| Image, nadir | The downward camera, which is the one perception reads by default |
| Image, gimbal | The gimbal camera |
| Map | The drone's own fix, and ground truth as GeoJSON points |
| Plot, scoring | Position error, recall and precision |
| Plot, altitude | Relative altitude and the rangefinder. The two agree over flat ground |
| Raw messages | The scoring summary as text |

## Every topic here is a stock ROS message

Nothing in this layout needs a Foxglove extension or a custom schema:

| Topic | Type |
|---|---|
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` |
| `/drone/markers`, `/ground_truth/markers` | `visualization_msgs/MarkerArray` |
| `/perception/<camera>/markers`, `/scoring/<camera>/markers` | `visualization_msgs/MarkerArray` |
| `/camera/*/footprint` | `geometry_msgs/PolygonStamped` |
| `/camera/*/boresight` | `geometry_msgs/PointStamped` |
| `/camera/*/image_raw/compressed` | `sensor_msgs/CompressedImage` |
| `/camera/*/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/*/annotations` | `foxglove_msgs/ImageAnnotations` |
| `/camera/*/ground_projection` | `sensor_msgs/PointCloud2` |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` |
| `/perception/*/detections_navsat` | `sensor_msgs/NavSatFix` |
| `/scoring/*/*` | `std_msgs/Float64`, `visualization_msgs/Marker` |
| `/ground_truth/geojson`, `/map_overlays/geojson` | `foxglove_msgs/GeoJSON` |

The image panels read the compressed topics. The raw ones are too large to
cross the websocket; see docs/troubleshooting.md.

The last one is the single exception. It is the message the Foxglove Map panel
reads for arbitrary geometry, and ROS ships no equivalent. A `NavSatFix` holds
one point, so six targets would need six topics.

The package is `ros-$ROS_DISTRO-foxglove-msgs`, from the normal ROS index. Only
the ground truth node uses it.

## Reading the 3D panel

- **Green pillars** are where targets actually are, from the scenario file.
- **Orange pillars** are where the detector thinks they are.
- **Yellow discs** are the one-sigma uncertainty from the covariance parameters.
- **Yellow lines** join an estimate to the target it matched, so the error is
  visible as a length rather than a number.
- **Cyan outline** is the ground the camera currently covers. Only targets
  inside it are scored, because the rest were never visible.

An orange pillar inside a green one is a hit. One beside it is the localization
error you want to reduce.

## If a panel is empty

The layout format moves between Foxglove releases. A panel can therefore come
back with default settings after an import. Check the topic on the panel first.

The other usual cause is that the stack does not publish the topic yet:

```bash
./px4sim shell ros
ros2 topic list
ros2 topic hz /perception/nadir/detections_3d
```

Ground truth appears once `ground_truth` reads the scenario. The Map panel
shows it once the vehicle reports a GPS fix, because the node needs that fix to
place the targets in latitude and longitude.
