# Foxglove layout

`px4-sim-stack.json` is the layout for this stack. It lives in the repository so
that the view is versioned with the code that feeds it.

## Use it

1. Start the stack with the ros profile, then open <https://app.foxglove.dev>
   or the Foxglove desktop app.
2. Connect to `ws://localhost:8765`.
3. **Layout → Import from file**, and choose `px4-sim-stack.json`.

## What you get

| Panel | Shows |
|---|---|
| 3D | The airframe, every frame, the camera footprint, ground truth in green, estimates in orange, and a line joining each match |
| Image, nadir | The downward camera, which is the one perception reads by default |
| Image, gimbal | The gimbal camera |
| Map | The drone's own fix, and ground truth as GeoJSON points |
| Plot, scoring | Position error, recall and precision |
| Plot, altitude | Relative altitude and the rangefinder, which should agree over flat ground |
| Raw messages | The scoring summary as text |

## Every topic here is a stock ROS message

Nothing in this layout needs a Foxglove extension or a custom schema:

| Topic | Type |
|---|---|
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` |
| `/drone/markers`, `/ground_truth/markers`, `/perception/markers` | `visualization_msgs/MarkerArray` |
| `/camera/*/footprint` | `geometry_msgs/PolygonStamped` |
| `/camera/*/boresight` | `geometry_msgs/PointStamped` |
| `/camera/*/image_raw`, `/camera/*/camera_info` | `sensor_msgs/Image`, `CameraInfo` |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` |
| `/scoring/*` | `std_msgs/Float64`, `visualization_msgs/Marker` |
| `/ground_truth/geojson` | `foxglove_msgs/GeoJSON` |

The last one is the single exception, and it is the message Foxglove's own Map
panel reads for arbitrary geometry. There is no stock ROS equivalent: a
`NavSatFix` carries one point, so six targets would need six topics. The
package is `ros-$ROS_DISTRO-foxglove-msgs`, from the normal ROS index, and only
the ground truth node uses it.

## Reading the 3D panel

- **Green pillars** are where targets actually are, from the scenario file.
- **Orange pillars** are where the detector thinks they are.
- **Yellow discs** are the one-sigma uncertainty from the covariance parameters.
- **Yellow lines** join an estimate to the target it matched, so the error is
  visible as a length rather than a number.
- **Cyan outline** is the ground the camera currently covers. Only targets
  inside it are scored, because the rest were never visible.

An orange pillar inside a green one is a hit. One sitting beside it is the
localization error you are trying to reduce.

## If a panel is empty

The layout format moves between Foxglove releases, so a panel can come back
with default settings after an import. Check the topic on the panel first.

Otherwise the usual cause is that the stack is not producing the topic yet:

```bash
./px4sim shell ros
ros2 topic list
ros2 topic hz /perception/detections_3d
```

Ground truth only appears once `ground_truth` has read the scenario, and the
Map panel only shows it once PX4 has published an EKF origin, which happens
after GPS lock.
