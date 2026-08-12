# Models

This directory is mounted into the perception container at
`/opt/perception/models`. It holds two things: the TensorRT engine cache, and
whatever model you bring.

## The engine cache

DeepStream builds a TensorRT engine on first start, and that takes about 45
seconds. Later starts reuse it and reach the first frame in about 15 seconds.

The engine lands **next to the ONNX file**, not at the path in
`model-engine-file`. nvinfer reads that key but ignores it when it saves, and
it picks its own name:

```
<onnx-file>_b<batch>_gpu<id>_<precision>.engine
```

That is why the entrypoint copies the default model out of the DeepStream image
into this directory on first start. A model inside the image would put the
engine inside the image, and every container recreate would rebuild it.

An engine is tied to the GPU, the driver and the TensorRT version. Delete the
`.engine` file to force a rebuild after any of those change.

`cache/` is left for your own use. Nothing writes to it now.

## The default model

The stack ships with ResNet18 TrafficCamNet, which comes inside the DeepStream
image. It detects car, bicycle, person and road_sign, and
`config_infer_person.txt` filters the output down to person.

It works with no download, and it is the wrong model for casualty detection. It
was trained on upright pedestrians seen from a traffic camera. A person who is
prone, seen from 40 m above, is not in that distribution. Expect it to find
standing people in the default scenario and to miss a casualty on the ground.

## PeopleNet

PeopleNet is the next step and still needs no custom parser. It is a TAO
detector with the same output format, so only the nvinfer config changes.

```bash
mkdir -p modules/perception/models/peoplenet
cd modules/perception/models/peoplenet
wget 'https://api.ngc.nvidia.com/v2/models/nvidia/tao/peoplenet/versions/deployable_quantized_onnx_v2.6.3/files/resnet34_peoplenet_int8.onnx'
wget 'https://api.ngc.nvidia.com/v2/models/nvidia/tao/peoplenet/versions/deployable_quantized_onnx_v2.6.3/files/labels.txt'
```

Then copy `config_infer_person.txt`, point `onnx-file` and `labelfile-path` at
the new files, set `num-detected-classes=3`, and give `model-engine-file` a new
path in `cache/`. Point `config-file` in `camera_detector.txt` at your copy.

## Your own YOLO

A YOLO model needs an ONNX export and a bounding box parser, because its output
layout is not the TAO layout that nvinfer parses natively.

1. Export to ONNX. For Ultralytics weights, use the DeepStream-Yolo exporter
   rather than `yolo export`, because nvinfer wants a specific output shape.
2. Build the parser from
   [DeepStream-Yolo](https://github.com/marcoslucianops/DeepStream-Yolo)
   against DeepStream 8.0. It produces `libnvdsinfer_custom_impl_Yolo.so`.
3. Put both here, then add to your nvinfer config:

```ini
onnx-file=/opt/perception/models/yolo/model.onnx
model-engine-file=/opt/perception/models/cache/yolo_b1_gpu0_fp16.engine
custom-lib-path=/opt/perception/models/yolo/libnvdsinfer_custom_impl_Yolo.so
parse-bbox-func-name=NvDsInferParseYolo
network-type=0
cluster-mode=2
```

The parser must be built inside the DeepStream 8.0 container, because it links
against the DeepStream libraries:

```bash
docker compose exec perception bash
cd /opt/perception/models/yolo/DeepStream-Yolo
CUDA_VER=12.8 make -C nvdsinfer_custom_impl_Yolo
```

## Casualty detection

For prone and injured people, the honest answer is that no off-the-shelf model
in this list does it well. You need a model trained on that data. The stack
gives you the place to put it and the pipeline around it. Point
`config_infer_person.txt` at your weights and the rest of the system does not
change.
