# `px4sim start` — static startup review

**Scope.** Code and configuration review only. No containers, services, processes, or hardware were started, stopped, restarted, or measured. Existing repository notes provide the few measured times below; all other time values are limits/inferences from code.

## Executive summary

The front door intentionally implements a **rebuild-and-recreate** workflow. Every `./px4sim start` tears down the selected stack, requests image builds, then recreates and boots the entire selected system. The rebuild is required deployment behavior, not a defect to remove. On a warm cache, Docker is reported to be ready in roughly one minute and nodes need a couple more minutes; this review therefore prioritizes slow shutdown and abnormal/long readiness waits.

The main cost centres are:

1. Container shutdown, which is paid before every intentional rebuild. The simulator's cleanup owner is replaced by `exec`, so its recorded child cleanup cannot run during the normal steady-state stop path.
2. DeepStream/TensorRT initialization after containers start. Existing notes record an 8m55s first classifier-engine build at 15 W and about 57 seconds to `pipelines PLAYING` when the engine already exists.
3. Simulated-companion camera readiness. The intended 300-second RTSP wait can consume up to **20m15s** because each 15-second probe is not included in the counter. `ds_node` can then wait a further 120 seconds for source discovery.

## Startup path

```text
./px4sim start
  |
  +-- resolve .env, DeepStream choice, scene, fleet profiles
  +-- docker compose down --remove-orphans
  +-- docker compose build ros-base
  |     +-- Chimera MAVROS patch
  |     +-- colcon interfaces, then 5g_drone + MAVInsight
  +-- docker compose build selected services
  +-- docker compose up -d
        +-- video/router/QGC/ground and onboard containers
        +-- sim: PX4 build -> Gazebo -> encoders -> scenario -> PX4 fleet
        +-- onboard<N>: calibration -> RTSP wait -> 5g_drone/MAVInsight graph
                         -> DeepStream/TensorRT pipeline
```

The `start` branch in [`px4sim`](../px4sim) always invokes `stop_stack`, `build_stack`, then `docker compose up -d`. `make up` and `make restart` forward to the same command.

## Findings

| Priority | Finding | Consequence | Evidence |
|---|---|---|---|
| P0 | Simulator child cleanup is inactive during steady-state shutdown. | The entrypoint records Gazebo, streamers, lenses, rangefinders, helper loops, and non-primary PX4 processes in `children`, but finally replaces its shell with primary PX4 using `exec`. The shell trap cannot run after that replacement, leaving teardown dependent on primary-PX4 exit/container-runtime escalation. | [`modules/sim/entrypoint.sh`](../modules/sim/entrypoint.sh) |
| P0 | Three core services explicitly allow 20 seconds to stop. | `sim`, every onboard companion, and offboard use `stop_grace_period: 20s`. If their PID 1 does not finish cleanly, each can consume that whole budget before Docker force-stops it; this cost is paid before each necessary rebuild. | [`compose.yaml`](../compose.yaml), [`px4sim`](../px4sim) |
| P0 | A missing or incompatible TensorRT engine is a large runtime blocker. | The image is up before `nvinfer` builds the engine. Existing docs record 8m55s for the first injury classifier build at 15 W; a warm pipeline still reaches `PLAYING` in about 57 s. | [`troubleshooting.md`](troubleshooting.md) |
| P1 | RTSP preflight has deadline-accounting bug. | The 300-second counter ignores the up-to-15-second probe. Worst case: 61 probes × 15 s + 60 sleeps × 5 s = **1,215 s / 20m15s** before ROS starts. Then `ds_node` has a separate 120-second source wait. | [`modules/onboard/entrypoint.sh`](../modules/onboard/entrypoint.sh), [`ds_compat.py`](../../ros2_ws/src/5g_drone/umd_uas/ds_ros_pipeline/ds_compat.py) |
| P1 | `ros-base` is requested before the remaining selected images. | This is expected deployment work, and reported warm-cache time is acceptable. It remains useful context because a cache miss spans the Chimera MAVROS overlay and 5g_drone/MAVInsight `colcon` build. | [`px4sim`](../px4sim), [`modules/ros-base/Dockerfile`](../modules/ros-base/Dockerfile) |
| P1 | 5g_drone and MAVInsight scale with fleet size. | Each onboard container launches a full flight/perception/frame-tree graph. The ground container starts another MAVROS, preview DeepStream, visualization, and MAVInsight tree per vehicle. Compose concurrency prevents simple serial multiplication, but process/DDS/GPU contention grows. | [`onboard.launch.py`](../../ros2_ws/src/5g_drone/launch/onboard.launch.py), [`offboard.launch.py`](../../ros2_ws/src/5g_drone/launch/offboard.launch.py), [`launch_sim.launch.py`](../../ros2_ws/src/MAVInsight/launch/launch_sim.launch.py) |
| P1 | Simulator boot includes deliberate serial waits. | PX4 is incrementally built every boot; Gazebo clock wait is capped at 90 s; scenario placement is synchronous; non-primary PX4 vehicles sleep 4 s apart. A four-vehicle fleet has 12 s of planned inter-vehicle delay. | [`modules/sim/entrypoint.sh`](../modules/sim/entrypoint.sh), [`spawn_scenario.py`](../modules/sim/scenes/spawn_scenario.py) |
| P2 | No-NVENC/NVDEC systems pay a startup and contention penalty. | Existing notes expect about 10 additional seconds per `ds_node` to resolve the CPU fallback, while encode/decode then compete with Gazebo and ROS for CPU. | [`troubleshooting.md`](troubleshooting.md) |

## Detailed findings

### Shutdown is the first place to improve

`px4sim start` begins with `docker compose down --remove-orphans`, so slow stop time is directly added to every necessary rebuild. `sim`, onboard, and offboard each declare a 20-second stop grace period.

The simulator has a specific lifecycle mismatch. It starts Gazebo, camera streamers, lens emulators, rangefinder bridges, helper loops, and non-primary PX4 instances in the background; their PIDs are stored in `children` and a shell trap calls `cleanup()`. The primary PX4 instance is then started with `exec`. `exec` replaces the shell—the only process that owns the trap and PID list—with PX4. Consequently, a later container stop cannot invoke that shell cleanup routine. A clean stop depends on PID 1 (PX4) exiting on its own; if it does not, Docker waits through the grace period and escalates.

This is a stronger shutdown suspect than the cached Docker build. The same pattern appears in a smaller form in the router and QGC entrypoints: they start background helpers and then `exec` the main program, without a supervisor to forward signals, wait for children, and reap them. The ROS onboard/offboard containers are better positioned because their PID 1 is directly `ros2 launch`, but they still have no explicit bounded staged shutdown or timing output for their large launch graphs.

### Chimera deploy: expensive only when the build layer executes

`compose.yaml` passes `CHIMERA_DEPLOY_DIR` to `ros-base` as a named build context. The base Dockerfile runs the Chimera MAVROS patch against that context's patch, MAVROS, and angles sources, producing `/opt/mavros`.

`px4sim start` always **requests** this base build. That does not mean Docker necessarily recompiles MAVROS on every start: a valid BuildKit cache can reuse it. It does mean a cache miss or relevant source/build-input change turns an ordinary restart into the documented 10–40 minute layer. The Dockerfile ordering is good: 5g_drone/MAVInsight source is copied after the MAVROS layer, so a flight-code-only edit should not invalidate the patch layer.

### 5g_drone and MAVInsight: build once per image, launch many times

The shared base clones/builds interfaces, then copies 5g_drone and MAVInsight and performs a second `colcon build`. The interface split protects the documented roughly five-minute interface build from normal node edits, but changes in either flight-code tree still invalidate the later build layer.

At runtime `onboard.launch.py` starts MAVROS, DeepStream, localization, mosaic, bridges, operator nodes, and MAVInsight for each vehicle. `offboard.launch.py` starts another MAVROS, preview DeepStream, visualization/localization set, and MAVInsight tree for every vehicle. This duplicated runtime work is intentional for the simulated air/ground architecture, but it makes a larger fleet slower and more resource-sensitive.

MAVInsight is not the strongest candidate for the multi-minute wall-clock delay by itself. It does dynamically expand a vehicle configuration tree into multiple ROS nodes, so it contributes process startup, Python import, DDS discovery, and frame-tree setup for every vehicle in both roles.

### DeepStream: the likely main post-container blocker

The Docker build appropriately compiles the DeepStream Yolo parser ahead of time. Model engines are different: a persistent model volume is used so a compatible TensorRT engine can deserialize on later runs; a missing/stale engine is generated at container runtime.

The current repository documentation provides the strongest concrete timing evidence:

- generic first engine creation: 1–3 minutes;
- missing injury-classifier engine on the 15 W bench: 8m55s;
- compatible warm engine: `ds_node` reaches `pipelines PLAYING` in about 57 seconds.

Therefore, Compose reporting a running onboard container is much earlier than the detector becoming usable.

### RTSP readiness: intended 300 s, actual worst-case 20m15s

The simulated onboard entrypoint waits for a full-rate RTSP stream before it invokes ROS:

```bash
until timeout 15 gst-launch ...; do
    [ "$waited" -ge "$STREAM_WAIT_S" ] && break
    sleep 5
    waited=$((waited + 5))
done
```

`STREAM_WAIT_S` defaults to 300, but `waited` advances only for sleep time. A probe that waits the full 15 seconds is free according to this counter. The physical upper bound is therefore 61 × 15 s probe time plus 60 × 5 s sleeps: 1,215 seconds. If the entrypoint gives up, it starts ROS anyway, where `ds_node`'s `discover_size()` has an independent 120-second wall-clock wait.

This can make a bad stream URI, non-serving encoder, or delayed Gazebo scene look like a 20+ minute start. It is a code-level issue independent of hardware speed.

### Simulator path: mostly bounded, plus serial fleet spacing

The simulator calls `make px4_sitl_default` every boot so source edits take effect. With the bind-mounted output and ccache it should be an incremental no-op in seconds; a missing output is documented as a 10–20 minute first build. It then waits for the Gazebo clock for at most 90 seconds, starts encoders, synchronously spawns the scenario, and begins PX4 instances.

Scenario entity service calls run with four workers and pose settling is capped at 1.5 seconds, so scenario logic is not the leading normal-path delay. Vehicle starts are deliberately serialized: each non-primary instance sleeps `UAS_START_DELAY_S` (default 4) after launch, for `4 × (fleet_size - 1)` deterministic seconds.

## Recommended follow-up

1. **Repair simulator signal ownership and fan-out.** Keep a supervisor shell as PID 1, or introduce a small init/supervisor, so `TERM` reaches the complete simulator process group, waits for bounded graceful exits, then kills remaining children. Do not rely on a trap in a shell that will be replaced by `exec`.
2. **Time shutdown by service and phase.** Record when Compose requests stop, when every PID 1 receives it, when Gazebo/PX4/ROS exit, and when Docker escalates. This will distinguish a slow `ros2 launch` graph from simulator orphan/child behavior before changing grace periods.
3. **Only then tune stop budgets.** Keep the present 20 seconds until the graceful path is correct and measured. A lower grace period alone would hide the delay by causing more forced termination, risking corrupted recordings, shared-memory leftovers, and unreliable next starts.
4. **Fix RTSP deadline accounting.** Use one monotonic absolute deadline around probe plus sleep, preserving the desired 300-second policy as elapsed wall time. Decide separately whether it is correct to launch ROS after the stream deadline expires.
5. **Prewarm/validate TensorRT engines before a mission.** Ensure the engine directory matches GPU, driver, and TensorRT before operational startup; otherwise the first start can legitimately take many minutes.
6. **Keep the rebuild path.** The existing cached build is intentional deployment behavior; retain it and focus optimization on stop/recreate reliability and actual operational readiness.

## Bottom line

The rebuild is intentional and the reported cached Docker time is not the problem to optimize. The key code-level concern is teardown: the simulator's cleanup owner disappears when the entrypoint `exec`s PX4, while core containers can wait 20 seconds for forced shutdown. After stop behavior, the leading long-readiness causes are DeepStream engine creation and the RTSP timeout-accounting error.
