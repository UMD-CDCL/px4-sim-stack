# Container builds and offline operation

Run `./px4sim prepare` once online for the profiles selected in `.env`. It
downloads ROS/CUDA dependencies, builds local dependency images, pulls any
image-only services, and builds the selected application images. It retains
external parent images locally and prepares scenegen on non-aircraft machines.
It does not start or stop containers. It also stages local sources; install `rsync` on
the host and populate these checkouts first:

- `ROS2_WS_DIR/src/{5g_drone,MAVInsight,px4_msgs,cdcl_umd_msgs}`
- `CHIMERA_DEPLOY_DIR/remote/mavros_patch`
- `CHIMERA_DEPLOY_DIR/submodules/{mavros,angles}`

`px4_msgs` now comes from your local checkout, including local edits, rather
than a network clone of `main`. Select the intended revision there explicitly.

## Daily commands

```bash
./px4sim start                  # existing images, no build or pull
./px4sim build                  # compile source changes offline, keep running
./px4sim build ground           # build ground and its shared ROS base
./px4sim restart                # build offline successfully, then restart
./px4sim restart --no-build     # restart existing images, e.g. after model changes
```

`start` does nothing if the stack is already running. `up` also uses existing
images. `restart` preserves running containers if a build fails. Image-only
services must be available locally before starting; missing images produce a
preparation instruction instead of an automatic pull.

## Cache boundaries

| Input changed | Build work |
|---|---|
| 5g_drone Python/config/launch | `umd_uas` packaging and image assembly |
| MAVInsight Python/resources | `mavinsight` packaging and image assembly |
| px4_msgs | That message package and image assembly |
| cdcl_umd_msgs | That message package and image assembly |
| MAVROS/angles/patch | Patched MAVROS overlay and image assembly |
| YOLO parser source | Parser, owning `umd_uas` package, and image assembly |
| Entry point | Only its image's final layers |
| Model weights or engines | No image build |
| ROS/CUDA/system dependencies | Explicit `prepare`, then affected stages |

The application packages currently contain Python, so packaging them does not
need the generated message libraries. Runtime message imports resolve to the
assembled interfaces. If a package gains compiled code, its build stage must
copy and source its actual dependency prefixes before compilation.

ROS source build instructions use `RUN --network=none`. ROS and parser
toolchains come from tagged local `ros-deps` and `yolo-deps` images selected by
`DS_TAG`. Those dependency images are not rebuilt by ordinary `build` or
`restart`. Changing dependency arguments, the ROS release, CUDA mapping, or
the selected hardware accommodation requires an online `prepare`.

Each ROS package has an independent build stage. Message compilation also
uses a persistent ccache keyed by package, architecture and DS tag; ccache
checks compiler and source contents. Install trees are rebuilt cleanly so
deleted files cannot remain installed. The final image merges isolated
package prefixes and contains no copied application source or build tree.

`scripts/build-contexts.sh` stages only selected package inputs into the
ignored `.build-contexts/` directory. It removes deleted inputs and excludes
models, generated workspaces, Git/editor state, and compiled artifacts.
MAVInsight's `models/` directory is included because it contains Python code.
New runtime directories added to either Python package's `setup.py` must also
be added to the staging allowlist. Module `.dockerignore` files keep scenes,
models and other runtime data out of Docker's primary contexts.

The shared ROS image is a Compose service with zero replicas so Compose can
resolve the explicit build dependency from onboard/offboard. It is never run.
For direct Compose builds, stage contexts first with the environment settings
from `.env`; the `px4sim` front door handles that and hardware selection.

## Models and offline limits

Keep ONNX files, labels and compatible TensorRT engines on the mounted model
volume. Changing weights or `/models/local/params.yaml` needs only a restart.
A new model can require engine compilation for this GPU/TensorRT combination;
that work is separate from Docker and ROS builds. The parser library is
updated atomically when its content hash changes, including within the same
DeepStream release.

Offline operation assumes the selected images, source checkouts, submodules,
models and scene resources are already present. Warm PX4's first build and
any Gazebo Fuel assets while online. New map tiles, remote model downloads,
new dependency versions and a different hardware target need preparation.
Do not prune the dependency images or build cache before field work. Image
tags retain dependencies even if intermediate build cache is evicted; losing
package cache can still require recompilation, but source builds need no
dependency downloads. No startup command silently substitutes stale images
after a failed source build.

## Verification

`python3 -m unittest discover -s verify -p test_build_inputs.py -v` checks model
exclusions, source deletion, CUDA mapping isolation, and concurrent parser
updates without Docker. After preparation, `python3 verify/build-cache.py`
exercises real offline package builds against temporary context copies and
checks unchanged, application, parser, and message cache boundaries. It uses
a temporary image tag and checks ROS package discovery and native message
type support without starting the stack.
