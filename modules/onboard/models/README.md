# Detector models

The onboard containers mount a model directory at `/models`, and
`onboard_sim_params.yaml` in 5g_drone names what is inside it. The offboard
ground station does not: its ds_node runs with `preview.only` and
`detect.enabled` false, so it loads no engine and has no model volume.

**5g_drone owns the artifacts.** `perception_models/` there holds one directory
per machine that builds engines, plus the portable ONNX and checkpoints they
are built from, and `scripts/fetch_models.py` fills in the one belonging to
this machine from a GitHub release. `.env` points `ONBOARD_MODEL_DIR` at that
tree, which is what makes `/models/local` -- the symlink for this machine's
group -- resolve inside the container.

Mount the WHOLE tree. Each group directory reaches the shared ONNX and labels
through relative symlinks, and mounting one group alone leaves them dangling.

This directory is the fallback when `ONBOARD_MODEL_DIR` is unset, and the place
to put artifacts when you are not using 5g_drone's tree. Nothing per-machine
lives here any more: `laptop_params.yaml` and the batch 2 classifier template
moved to `perception_models/t500/` in 5g_drone, beside the engines they belong
to, reached at `/models/local/params.yaml` on every machine.

nvinfer builds the TensorRT engine next to the ONNX file on the first run,
which takes 1 to 3 minutes, and keeps it afterwards. An engine belongs to one
GPU, one driver and one TensorRT version, so it is not shared between machines
and git does not carry it.

The bbox parser is not an artifact of either kind: `entrypoint.sh` installs
`libnvdsinfer_custom_impl_Yolo.so` into the root of the model volume from
`/opt/ds-yolo/` in the image and stamps `.parser-deepstream` with the release,
so it always matches the DeepStream that is running.

Whichever directory is mounted must exist before the first start. Docker
creates a missing bind-mount source as a root-owned directory, and the
container then cannot write its engine there.
