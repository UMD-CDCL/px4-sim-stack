# Detector models

The onboard and offboard containers mount this directory at `/models`, and
`onboard_sim_params.yaml` in 5g_drone names the files inside it.

Put the detector and classifier artifacts here, from
`scripts/convert_to_engine.py` in 5g_drone. nvinfer builds the TensorRT engine
next to the ONNX file on the first run, which takes 1 to 3 minutes, and keeps it
afterwards. An engine belongs to one GPU, one driver and one TensorRT version,
so it is not shared between machines and git does not carry it.

`ONBOARD_MODEL_DIR` in `.env` points somewhere else when the models live outside
this repository.

This directory must exist before the first start. Docker creates a missing
bind-mount source as a root-owned directory, and the container then cannot write
its engine there.
