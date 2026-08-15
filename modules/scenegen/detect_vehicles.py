#!/usr/bin/env python3
"""Find cars and buses in the satellite image and write them to scene.json.

The detector is YOLOv8-OBB pretrained on DOTA v1, an aerial-imagery
dataset whose classes include "small vehicle" and "large vehicle". Oriented
boxes give position, size and axis; the axis fixes heading only modulo 180
degrees (the editor has a flip button; see DEFERRED.md).

Detection replaces earlier source="auto" vehicles and never touches
source="manual" ones, so a re-run with a different confidence keeps every
hand-placed vehicle. A review image with the boxes drawn lands in
report/detections.jpg next to scene.json.

The weights download once into DATA_DIR/weights and stay there.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

import geo
import scene_model
import sources

# ------------------------------------------------------------------- tunables
WEIGHTS_NAME = "yolov8s-obb.pt"
# DOTA class names to scene classes. Everything else is ignored.
CLASS_MAP = {"small vehicle": "car", "large vehicle": "bus"}
# Inference runs on square crops upsampled 2x, so a car spans about 40
# pixels at zoom 19, the size the DOTA training data favors. Bigger tiles
# with the default 640-pixel inference size shrank cars to 12 pixels and
# the detector missed most of them.
TILE_PX = 512
TILE_OVERLAP_PX = 96
INFERENCE_PX = 1024
# Two detections of one vehicle from neighboring tiles: the one with the
# lower confidence goes when the rotated boxes overlap this much.
DUPLICATE_IOU = 0.35
# A detected "car" outside these bounds is a shadow, a dumpster or a roof
# structure. Meters.
SIZE_BOUNDS = {"car": (2.2, 7.0, 1.2, 3.2), "bus": (5.0, 20.0, 1.8, 4.5)}


def _load_model(data_root: Path):
    from ultralytics import YOLO

    weights_dir = data_root / "weights"
    weights_dir.mkdir(parents=True, exist_ok=True)
    weights = weights_dir / WEIGHTS_NAME
    if weights.is_file():
        return YOLO(str(weights))
    # Ultralytics downloads a bare model name into the working directory,
    # so stand in the weights directory for the first run.
    previous = Path.cwd()
    os.chdir(weights_dir)
    try:
        return YOLO(WEIGHTS_NAME)
    finally:
        os.chdir(previous)


def _tiles(width: int, height: int) -> list[tuple[int, int]]:
    step = TILE_PX - TILE_OVERLAP_PX
    xs = list(range(0, max(width - TILE_OVERLAP_PX, 1), step))
    ys = list(range(0, max(height - TILE_OVERLAP_PX, 1), step))
    return [(x, y) for y in ys for x in xs]


def _rotated_corners(east: float, north: float, length: float, width: float,
                     heading_deg: float) -> list[tuple[float, float]]:
    yaw = math.radians(heading_deg)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    corners = []
    for dx, dy in [(length / 2, width / 2), (length / 2, -width / 2),
                   (-length / 2, -width / 2), (-length / 2, width / 2)]:
        corners.append((east + dx * cos_yaw - dy * sin_yaw,
                        north + dx * sin_yaw + dy * cos_yaw))
    return corners


def _deduplicate(candidates: list[dict]) -> list[dict]:
    import shapely

    kept: list[dict] = []
    kept_shapes: list = []
    for candidate in sorted(candidates, key=lambda c: -c["confidence"]):
        shape = shapely.Polygon(_rotated_corners(
            candidate["east_m"], candidate["north_m"], candidate["length_m"],
            candidate["width_m"], candidate["heading_deg"]))
        duplicate = False
        for other in kept_shapes:
            union = shape.union(other).area
            if union > 0 and shape.intersection(other).area / union > DUPLICATE_IOU:
                duplicate = True
                break
        if not duplicate:
            kept.append(candidate)
            kept_shapes.append(shape)
    return kept


def _write_review_image(image: Image.Image, scene: scene_model.SceneSpec,
                        georef: "sources.RasterGeoref", frame: geo.GeoFrame,
                        out_path: Path) -> None:
    review = image.convert("RGB")
    draw = ImageDraw.Draw(review)
    colors = {"car": (0, 220, 255), "bus": (255, 210, 0)}
    for vehicle in scene.vehicles:
        corners = _rotated_corners(vehicle.east_m, vehicle.north_m,
                                   vehicle.length_m, vehicle.width_m,
                                   vehicle.heading_deg)
        pixel_corners = []
        for east, north in corners:
            lat, lon, _ = frame.enu_to_latlon(east, north)
            pixel_corners.append(georef.latlon_to_raster_px(lat, lon))
        draw.polygon(pixel_corners, outline=colors.get(vehicle.cls, (255, 0, 0)),
                     width=2)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    review.save(out_path, quality=88)


def run(scene_data_dir: Path, confidence: float, replace_auto: bool) -> int:
    scene_path = scene_data_dir / "scene.json"
    if not scene_path.is_file():
        print(f"No scene at {scene_path}. Run create first.", file=sys.stderr)
        return 1
    scene = scene_model.load_scene(scene_path)
    frame = geo.GeoFrame(scene.center_lat, scene.center_lon, scene.origin_alt_m)
    georef = sources.RasterGeoref(scene.imagery["zoom"], scene.imagery["origin_px"],
                                  scene.imagery["origin_py"])
    meters_per_px = scene.imagery["m_per_px"]
    image = Image.open(scene_data_dir / scene.imagery["file"]).convert("RGB")

    model = _load_model(scene_data_dir.parent)
    class_names = {index: name for index, name in model.names.items()}

    candidates: list[dict] = []
    tiles = _tiles(image.width, image.height)
    half_side = scene.side_m / 2.0
    for tile_number, (tile_x, tile_y) in enumerate(tiles, start=1):
        crop = image.crop((tile_x, tile_y, min(tile_x + TILE_PX, image.width),
                           min(tile_y + TILE_PX, image.height)))
        results = model.predict(np.asarray(crop), conf=confidence,
                                imgsz=INFERENCE_PX, verbose=False)
        boxes = results[0].obb
        found = 0
        for row, class_id, box_confidence in zip(
                boxes.xywhr.tolist(), boxes.cls.tolist(), boxes.conf.tolist()):
            scene_class = CLASS_MAP.get(class_names[int(class_id)])
            if scene_class is None:
                continue
            center_x, center_y, box_w, box_h, rotation = row
            pixel_x = tile_x + center_x
            pixel_y = tile_y + center_y
            lat, lon = georef.raster_px_to_latlon(pixel_x, pixel_y)
            east, north, _ = frame.latlon_to_enu(lat, lon)
            if abs(east) > half_side or abs(north) > half_side:
                continue
            length = box_w * meters_per_px
            width = box_h * meters_per_px
            # Image rotation is clockwise-positive (y grows down); ENU
            # heading is counterclockwise from east.
            heading = -math.degrees(rotation)
            if width > length:
                length, width = width, length
                heading += 90.0
            heading = (heading + 90.0) % 180.0 - 90.0
            min_l, max_l, min_w, max_w = SIZE_BOUNDS[scene_class]
            if not (min_l <= length <= max_l and min_w <= width <= max_w):
                continue
            candidates.append({"cls": scene_class, "east_m": round(east, 2),
                               "north_m": round(north, 2),
                               "length_m": round(length, 2),
                               "width_m": round(width, 2),
                               "heading_deg": round(heading, 1),
                               "confidence": round(box_confidence, 3)})
            found += 1
        print(f"  tile {tile_number}/{len(tiles)}: {found} vehicles")

    unique = _deduplicate(candidates)

    kept = [v for v in scene.vehicles
            if v.source == "manual" or (not replace_auto)]
    for number, candidate in enumerate(unique, start=1):
        kept.append(scene_model.Vehicle(
            id=f"v_auto_{number}", source="auto", **candidate))
    scene.vehicles = kept
    scene_model.save_scene(scene, scene_path)

    cars = sum(1 for v in unique if v["cls"] == "car")
    buses = sum(1 for v in unique if v["cls"] == "bus")
    print(f"{len(unique)} vehicles ({cars} cars, {buses} buses) after "
          f"removing {len(candidates) - len(unique)} tile-overlap duplicates")
    _write_review_image(image, scene, georef, frame,
                        scene_data_dir / "report" / "detections.jpg")
    print(f"Review image: {scene_data_dir / 'report' / 'detections.jpg'}")
    print("Open the editor to fix misses and sizes: scenegen.py edit "
          f"--name {scene.name}")
    return 0
