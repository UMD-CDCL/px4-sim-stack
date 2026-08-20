#!/usr/bin/env python3
"""Write the calibration of a simulated camera, from the model that renders it.

cam_info publishes CameraInfo from a YAML calibration, and the aircraft's files
describe real lenses. A simulated camera is an ideal pinhole at whatever field
of view its airframe was rendered with, so its calibration is derived here
rather than stored: a real lens file would give a pinhole render a real lens's
distortion, and every ray would leave the image slightly bent.

The image size comes from the airframe model and the field of view from the
same list the simulator renders with, so neither is written down twice.
"""

import argparse
import math
import xml.etree.ElementTree as ElementTree


def image_size(model_sdf: str, sensor: str) -> tuple[int, int]:
    """The pixel size one sensor renders at, read from the airframe model."""
    root = ElementTree.parse(model_sdf).getroot()
    for element in root.iter("sensor"):
        if element.get("name") != sensor:
            continue
        image = element.find("./camera/image")
        if image is None:
            break
        return int(image.findtext("width")), int(image.findtext("height"))
    raise SystemExit(f"{model_sdf} declares no camera sensor named '{sensor}'")


def calibration(width: int, height: int, hfov_deg: float) -> dict:
    focal = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    centre_x, centre_y = width / 2.0, height / 2.0
    return {
        "image_width": width,
        "image_height": height,
        "camera_name": "simulated",
        "distortion_model": "plumb_bob",
        "camera_matrix": {"rows": 3, "cols": 3, "data": [
            focal, 0.0, centre_x, 0.0, focal, centre_y, 0.0, 0.0, 1.0]},
        "distortion_coefficients": {"rows": 1, "cols": 5,
                                    "data": [0.0, 0.0, 0.0, 0.0, 0.0]},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": [
            1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
        "projection_matrix": {"rows": 3, "cols": 4, "data": [
            focal, 0.0, centre_x, 0.0, 0.0, focal, centre_y, 0.0, 0.0, 0.0, 1.0, 0.0]},
    }


def as_yaml(values: dict) -> str:
    """The calibration as YAML. Written by hand to keep this script free of
    imports the companion image is not guaranteed to hold."""
    lines = []
    for key, value in values.items():
        if isinstance(value, dict):
            lines.append(f"{key}:")
            # repr of a float always carries a decimal point. CameraInfo
            # rejects a matrix holding an int, and 0 and 1 are the entries a
            # plain number format writes without one.
            lines += [f"  rows: {value['rows']}", f"  cols: {value['cols']}",
                      "  data: [" + ", ".join(repr(float(number)) for number in value["data"]) + "]"]
        elif isinstance(value, str):
            lines.append(f"{key}: {value}")
        else:
            lines.append(f"{key}: {value}")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="the airframe model.sdf")
    parser.add_argument("--sensor", default="gimbal_camera")
    parser.add_argument("--hfov-deg", type=float, required=True)
    parser.add_argument("--width", type=int, default=0,
                        help="pixel space of the calibration. The aircraft "
                             "calibrates at the preview size, and a panel that "
                             "draws a preview against a full size calibration "
                             "puts every annotation in the wrong place.")
    parser.add_argument("--height", type=int, default=0)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    width, height = image_size(args.model, args.sensor)
    # The field of view belongs to the camera and the pixel space to whoever
    # reads it. Naming a smaller space keeps the same view through fewer
    # pixels, which is what a preview is.
    if args.width and args.height:
        width, height = args.width, args.height
    with open(args.out, "w", encoding="utf-8") as handle:
        handle.write(as_yaml(calibration(width, height, args.hfov_deg)))
    print(f"camera: {args.sensor} {width}x{height} at {args.hfov_deg} degrees -> {args.out}")


if __name__ == "__main__":
    main()
