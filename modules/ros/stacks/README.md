# ROS stacks

One directory here is one autonomy stack. Switch between them with one variable:

```bash
ROS_STACK=baseline make up          # or edit ROS_STACK in .env
docker compose up -d --force-recreate ros
```

## The contract

A stack directory must contain:

| Item | Purpose |
|---|---|
| `stack.launch.py` at the root | The entry point. The ros container runs this file. |
| Any number of ROS packages | colcon builds the whole directory. |

That is all. A stack can be one Python package or twenty C++ ones. It can use
any ROS libraries in the image, and it can add more through its `package.xml`
plus a rebuild of the ros image.

## What a stack is allowed to see

Three addresses, supplied as environment variables:

| Variable | Value | What it carries |
|---|---|---|
| `FCU_URL` | `udp://:14555@mavlink-hub:14551` | MAVLink. Telemetry, commands, missions. |
| `RTSP_BASE` | `rtsp://video-router:8554` | Video. `/gimbal` and `/nadir`. |
| `MQTT_HOST` | `message-bus` | Detections, on topic `perception/detections`. |

A stack must not reach into Gazebo or link against PX4. That rule is what makes
the same stack run against real hardware: point the three variables at the
aircraft and nothing else changes.

The one supported exception is the `xrce` profile, which adds the uXRCE-DDS
agent and puts PX4 uORB topics on the ROS graph. Use it when a stack needs
`px4_msgs`, and know that the stack then depends on PX4 directly.

## Starting a new stack

```bash
cp -r modules/ros/stacks/baseline modules/ros/stacks/my_stack
# edit modules/ros/stacks/my_stack/stack.launch.py
ROS_STACK=my_stack docker compose up -d --force-recreate ros
docker compose logs -f ros
```

The build output goes to `src/ros2_ws/`, which is on the host. Two stacks share
that workspace, so run `rm -rf src/ros2_ws/{build,install,log}` if you switch
between stacks that declare packages with the same name.

## Stacks in this repository

| Name | What it does |
|---|---|
| `baseline` | MAVROS, both cameras, the detection bridge and a Foxglove websocket. It flies nothing. Run it to check the plumbing. |

## Debugging

```bash
make ros                                  # a shell, with the overlay sourced
ros2 topic list
ros2 topic hz /camera/gimbal/image_raw
ros2 topic echo /mavros/state
ros2 topic echo /perception/detections
```

Set `ROS_AUTOLAUNCH=0` to bring the container up without starting the stack.
The container then idles and waits for you.
