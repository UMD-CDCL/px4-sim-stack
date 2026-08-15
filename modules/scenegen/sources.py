#!/usr/bin/env python3
"""Free data sources for the scene generator.

Three fetchers, one per layer of the scene:

  fetch_satellite_image   ground texture, from a web mercator tile server.
                          Google satellite tiles by default. The user asked
                          for them; check the imagery terms for your use.
  fetch_elevation_grid    terrain heights, from the AWS terrain tiles
                          (Mapzen "terrarium" encoding, SRTM and 3DEP based,
                          public, no API key).
  fetch_osm_buildings     building footprints and heights, from the
                          OpenStreetMap Overpass API.

Every tile lands in an on-disk cache first, so a re-run after an edit or a
crash downloads nothing it already has.
"""

from __future__ import annotations

import io
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import requests
from PIL import Image

import geo

# ------------------------------------------------------------------- tunables
IMAGERY_URL_TEMPLATES = {
    # {s} rotates 0..3 to spread load the way map clients do.
    "google": "https://mt{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
    "esri": ("https://server.arcgisonline.com/ArcGIS/rest/services/"
             "World_Imagery/MapServer/tile/{z}/{y}/{x}"),
}
ELEVATION_URL_TEMPLATE = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
# Two Overpass mirrors. The second answers when the first is saturated.
OVERPASS_URLS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]
USER_AGENT = "px4-sim-stack-scenegen/1.0"
DOWNLOAD_WORKERS = 8
DOWNLOAD_RETRIES = 3
REQUEST_TIMEOUT_S = 30
OVERPASS_TIMEOUT_S = 90
# 2048 tiles is about 500 MB of imagery. Above that the zoom is wrong for
# the requested side, so stop and say so instead of hammering the server.
MAX_TILES_PER_LAYER = 2048
# Terrarium tiles exist up to zoom 15 everywhere.
ELEVATION_ZOOM = 15
# Fetch buildings a little past the square, so a wall on the boundary
# still gets its whole footprint.
BUILDING_FETCH_MARGIN_M = 30.0
# When OSM gives storeys instead of meters. 3.2 m per storey is the usual
# assumption for offices and apartments.
METERS_PER_BUILDING_LEVEL = 3.2
DEFAULT_BUILDING_HEIGHT_M = 6.0
MIN_BUILDING_HEIGHT_M = 2.5


@dataclass(frozen=True)
class RasterGeoref:
    """Places a stitched raster in the world: the global mercator pixel of
    the raster's top-left corner, and the zoom it was fetched at."""
    zoom: int
    origin_px: float
    origin_py: float

    def latlon_to_raster_px(self, lat: float, lon: float) -> tuple[float, float]:
        gx, gy = geo.latlon_to_mercator_px(lat, lon, self.zoom)
        return gx - self.origin_px, gy - self.origin_py

    def raster_px_to_latlon(self, x: float, y: float) -> tuple[float, float]:
        return geo.mercator_px_to_latlon(self.origin_px + x, self.origin_py + y, self.zoom)


def _session() -> requests.Session:
    s = requests.Session()
    s.headers["User-Agent"] = USER_AGENT
    return s


def _fetch_tile(session: requests.Session, url: str, cache_path: Path) -> bytes:
    if cache_path.exists():
        return cache_path.read_bytes()
    last_error: Exception | None = None
    for attempt in range(DOWNLOAD_RETRIES):
        try:
            reply = session.get(url, timeout=REQUEST_TIMEOUT_S)
            reply.raise_for_status()
            cache_path.parent.mkdir(parents=True, exist_ok=True)
            cache_path.write_bytes(reply.content)
            return reply.content
        except Exception as error:  # noqa: BLE001 - retried, then re-raised
            last_error = error
            time.sleep(1.0 + attempt)
    raise RuntimeError(f"could not fetch {url}: {last_error}")


def _square_mercator_bounds(frame: geo.GeoFrame, side_m: float,
                            zoom: int) -> tuple[float, float, float, float]:
    """Global pixel bounds (min x, min y, max x, max y) that cover the ENU
    square. All four corners matter: the square is straight in ENU and very
    slightly curved in mercator."""
    half = side_m / 2.0
    xs, ys = [], []
    for east, north in [(-half, -half), (-half, half), (half, -half), (half, half)]:
        lat, lon, _ = frame.enu_to_latlon(east, north)
        px, py = geo.latlon_to_mercator_px(lat, lon, zoom)
        xs.append(px)
        ys.append(py)
    return min(xs), min(ys), max(xs), max(ys)


def _stitch_tiles(frame: geo.GeoFrame, side_m: float, zoom: int,
                  url_for_tile, cache_dir: Path, tile_suffix: str,
                  margin_tiles: int = 0) -> tuple[Image.Image, RasterGeoref]:
    min_x, min_y, max_x, max_y = _square_mercator_bounds(frame, side_m, zoom)
    tile_x0 = geo.tile_index(min_x) - margin_tiles
    tile_y0 = geo.tile_index(min_y) - margin_tiles
    tile_x1 = geo.tile_index(max_x) + margin_tiles
    tile_y1 = geo.tile_index(max_y) + margin_tiles
    count = (tile_x1 - tile_x0 + 1) * (tile_y1 - tile_y0 + 1)
    if count > MAX_TILES_PER_LAYER:
        raise ValueError(f"{count} tiles at zoom {zoom} for a {side_m:.0f} m side. "
                         f"The cap is {MAX_TILES_PER_LAYER}; lower the zoom.")

    stitched = Image.new("RGB", ((tile_x1 - tile_x0 + 1) * geo.TILE_PX,
                                 (tile_y1 - tile_y0 + 1) * geo.TILE_PX))
    session = _session()

    def fetch_one(tile: tuple[int, int]) -> tuple[tuple[int, int], bytes]:
        tx, ty = tile
        cache_path = cache_dir / str(zoom) / str(tx) / f"{ty}{tile_suffix}"
        return tile, _fetch_tile(session, url_for_tile(tx, ty, zoom), cache_path)

    tiles = [(tx, ty) for tx in range(tile_x0, tile_x1 + 1)
             for ty in range(tile_y0, tile_y1 + 1)]
    with ThreadPoolExecutor(max_workers=DOWNLOAD_WORKERS) as pool:
        for (tx, ty), payload in pool.map(fetch_one, tiles):
            tile_image = Image.open(io.BytesIO(payload))
            stitched.paste(tile_image.convert("RGB"),
                           ((tx - tile_x0) * geo.TILE_PX, (ty - tile_y0) * geo.TILE_PX))

    georef = RasterGeoref(zoom, tile_x0 * geo.TILE_PX, tile_y0 * geo.TILE_PX)
    return stitched, georef


def fetch_satellite_image(frame: geo.GeoFrame, side_m: float, zoom: int,
                          source: str, cache_dir: Path) -> tuple[Image.Image, RasterGeoref]:
    """The stitched imagery cropped to the scene square's mercator bounds.
    Row 0 of the result is the north edge."""
    template = IMAGERY_URL_TEMPLATES[source]

    def url_for_tile(tx: int, ty: int, z: int) -> str:
        return template.format(x=tx, y=ty, z=z, s=(tx + ty) % 4)

    stitched, georef = _stitch_tiles(frame, side_m, zoom, url_for_tile,
                                     cache_dir / source, ".jpg")
    min_x, min_y, max_x, max_y = _square_mercator_bounds(frame, side_m, zoom)
    left = int(math.floor(min_x - georef.origin_px))
    top = int(math.floor(min_y - georef.origin_py))
    right = int(math.ceil(max_x - georef.origin_px))
    bottom = int(math.ceil(max_y - georef.origin_py))
    cropped = stitched.crop((left, top, right, bottom))
    cropped_georef = RasterGeoref(zoom, georef.origin_px + left, georef.origin_py + top)
    return cropped, cropped_georef


def decode_terrarium(raster: np.ndarray) -> np.ndarray:
    """Terrarium PNG channels to meters: (R * 256 + G + B / 256) - 32768."""
    r = raster[:, :, 0].astype(np.float64)
    g = raster[:, :, 1].astype(np.float64)
    b = raster[:, :, 2].astype(np.float64)
    return r * 256.0 + g + b / 256.0 - 32768.0


def fetch_elevation_grid(frame: geo.GeoFrame, side_m: float, grid_n: int,
                         cache_dir: Path) -> np.ndarray:
    """Terrain heights in meters AMSL on a regular ENU grid over the square.

    Shape (grid_n + 1, grid_n + 1). Row 0 is the SOUTH edge, column 0 the
    west edge, so index [j][i] sits at
      east  = -side/2 + i * side / grid_n
      north = -side/2 + j * side / grid_n
    Bilinear-sampled from the terrarium raster, which is several meters per
    pixel, so the grid is smooth rather than adding detail.
    """
    def url_for_tile(tx: int, ty: int, z: int) -> str:
        return ELEVATION_URL_TEMPLATE.format(x=tx, y=ty, z=z)

    stitched, georef = _stitch_tiles(frame, side_m, ELEVATION_ZOOM, url_for_tile,
                                     cache_dir / "terrarium", ".png", margin_tiles=1)
    heights = decode_terrarium(np.asarray(stitched))

    grid = np.zeros((grid_n + 1, grid_n + 1), dtype=np.float32)
    half = side_m / 2.0
    for j in range(grid_n + 1):
        north = -half + j * side_m / grid_n
        for i in range(grid_n + 1):
            east = -half + i * side_m / grid_n
            lat, lon, _ = frame.enu_to_latlon(east, north)
            x, y = georef.latlon_to_raster_px(lat, lon)
            grid[j, i] = _bilinear(heights, x, y)
    return grid


def _bilinear(raster: np.ndarray, x: float, y: float) -> float:
    height, width = raster.shape
    x = min(max(x, 0.0), width - 1.001)
    y = min(max(y, 0.0), height - 1.001)
    x0, y0 = int(x), int(y)
    fx, fy = x - x0, y - y0
    top = raster[y0, x0] * (1 - fx) + raster[y0, x0 + 1] * fx
    bottom = raster[y0 + 1, x0] * (1 - fx) + raster[y0 + 1, x0 + 1] * fx
    return float(top * (1 - fy) + bottom * fy)


def parse_height_m(value: str) -> float | None:
    """OSM height tags arrive as '21.58', '22 m', or (rarely) feet: "72'"."""
    text = value.strip().lower()
    feet = text.endswith("'") or text.endswith("ft")
    text = text.rstrip("'").removesuffix("ft").removesuffix("m").strip()
    try:
        meters = float(text)
    except ValueError:
        return None
    return meters * 0.3048 if feet else meters


def resolve_building_height(tags: dict) -> tuple[float, str]:
    """The building's height in meters and where the number came from."""
    if "height" in tags:
        parsed = parse_height_m(tags["height"])
        if parsed is not None:
            return max(parsed, MIN_BUILDING_HEIGHT_M), "height tag"
    if "building:levels" in tags:
        try:
            levels = float(tags["building:levels"])
            return max(levels * METERS_PER_BUILDING_LEVEL, MIN_BUILDING_HEIGHT_M), "levels tag"
        except ValueError:
            pass
    return DEFAULT_BUILDING_HEIGHT_M, "default"


def fetch_osm_buildings(frame: geo.GeoFrame, side_m: float) -> tuple[list[dict], dict]:
    """Building outlines inside the square, from Overpass.

    Returns (buildings, report). Each building:
      id            "way/123" or "relation/123-0" for an outer ring
      name          the name tag or ""
      height_m      resolved height
      height_source "height tag" | "levels tag" | "default"
      outline       [[lat, lon], ...], closed ring
    Relations contribute their outer rings as independent outlines; holes
    are ignored (see DEFERRED.md).
    """
    half = side_m / 2.0 + BUILDING_FETCH_MARGIN_M
    south, west, _ = frame.enu_to_latlon(-half, -half)
    north, east, _ = frame.enu_to_latlon(half, half)
    query = (f"[out:json][timeout:{OVERPASS_TIMEOUT_S}];"
             f'(way["building"]({south},{west},{north},{east});'
             f'relation["building"]({south},{west},{north},{east}););'
             "out tags geom;")

    session = _session()
    reply = None
    last_error: Exception | None = None
    for url in OVERPASS_URLS:
        try:
            reply = session.post(url, data={"data": query}, timeout=OVERPASS_TIMEOUT_S + 15)
            reply.raise_for_status()
            break
        except Exception as error:  # noqa: BLE001 - try the next mirror
            last_error = error
            reply = None
    if reply is None:
        raise RuntimeError(f"every Overpass mirror failed: {last_error}")

    elements = reply.json().get("elements", [])
    buildings: list[dict] = []
    report = {"ways": 0, "relation_rings": 0, "skipped_open_rings": 0,
              "skipped_relations_holes": 0}

    def add_ring(ring_id: str, tags: dict, points: list[dict]) -> None:
        outline = [[p["lat"], p["lon"]] for p in points]
        if len(outline) >= 3 and outline[0] != outline[-1]:
            outline.append(outline[0])
        if len(outline) < 4:
            report["skipped_open_rings"] += 1
            return
        height, height_source = resolve_building_height(tags)
        buildings.append({"id": ring_id, "name": tags.get("name", ""),
                          "height_m": round(height, 2),
                          "height_source": height_source, "outline": outline})

    for element in elements:
        tags = element.get("tags", {})
        if element["type"] == "way" and "geometry" in element:
            add_ring(f"way/{element['id']}", tags, element["geometry"])
            report["ways"] += 1
        elif element["type"] == "relation":
            outer_index = 0
            for member in element.get("members", []):
                if member.get("role") != "outer" or "geometry" not in member:
                    if member.get("role") == "inner":
                        report["skipped_relations_holes"] += 1
                    continue
                add_ring(f"relation/{element['id']}-{outer_index}", tags,
                         member["geometry"])
                outer_index += 1
                report["relation_rings"] += 1
    return buildings, report


def write_buildings_geojson(buildings: list[dict], path: Path) -> None:
    """The raw footprints in lat/lon, for inspection in any GeoJSON viewer."""
    features = []
    for building in buildings:
        coordinates = [[[lon, lat] for lat, lon in building["outline"]]]
        features.append({"type": "Feature",
                         "properties": {key: building[key] for key in
                                        ("id", "name", "height_m", "height_source")},
                         "geometry": {"type": "Polygon", "coordinates": coordinates}})
    path.write_text(json.dumps({"type": "FeatureCollection", "features": features},
                               indent=1))
