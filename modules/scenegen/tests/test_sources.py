#!/usr/bin/env python3
"""Ground-truth tests for sources.py. These hit the network.

Run: python3 tests/test_sources.py [--cache DIR]

The expected values are physical facts, not fixtures:
  - Badwater Basin, Death Valley, sits about 85 m below sea level.
  - Lake Tahoe's surface is at 1897 m, and a lake is flat.
  - The Washington Monument's OSM footprint carries height=169.29 and its
    square base is about 16.8 m on a side.
  - H.J. Patterson Hall at UMD (way 23585339) is tagged height=21.58.
"""

from __future__ import annotations

import argparse
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import geo
import scene_model
import sources

CHECKS = []


def check(name: str, condition: bool, detail: str = "") -> None:
    CHECKS.append((name, condition, detail))
    mark = "ok  " if condition else "FAIL"
    print(f"  {mark} {name}" + (f"  [{detail}]" if detail else ""))


def test_elevation_known_points(cache: Path) -> None:
    print("elevation against known ground truth")
    badwater = sources.fetch_elevation_grid(geo.GeoFrame(36.2461, -116.8195), 200, 8,
                                            cache / "badwater")
    check("Badwater Basin is about 85 m below sea level",
          -110 < float(badwater.mean()) < -55, f"mean {badwater.mean():.1f} m")

    tahoe = sources.fetch_elevation_grid(geo.GeoFrame(39.0968, -120.0324), 400, 8,
                                         cache / "tahoe")
    check("Lake Tahoe's surface is at 1897 m",
          abs(float(tahoe.mean()) - 1897.0) < 15, f"mean {tahoe.mean():.1f} m")
    check("a lake is flat",
          float(tahoe.max() - tahoe.min()) < 5, f"relief {tahoe.max()-tahoe.min():.2f} m")


def test_buildings_known_footprints() -> None:
    print("OSM buildings against known footprints")
    monument_frame = geo.GeoFrame(38.8895, -77.0353)
    buildings, _ = sources.fetch_osm_buildings(monument_frame, 200)
    monuments = [b for b in buildings
                 if b["height_source"] == "height tag" and 160 < b["height_m"] < 175]
    check("the Washington Monument carries its height tag", len(monuments) == 1,
          f"{len(monuments)} candidates in {len(buildings)} buildings")
    if monuments:
        entries, _ = scene_model.buildings_from_osm(monuments, monument_frame, 200)
        base = entries[0]
        check("its base is a 16.8 m square",
              15 < base.length_m < 19 and 15 < base.width_m < 19,
              f"{base.length_m:.1f} x {base.width_m:.1f} m")
        check("it stands at the frame center",
              abs(base.east_m) < 25 and abs(base.north_m) < 25,
              f"({base.east_m:.1f}, {base.north_m:.1f})")

    umd_frame = geo.GeoFrame(38.9869, -76.9426)
    buildings, report = sources.fetch_osm_buildings(umd_frame, 600)
    patterson = [b for b in buildings if b["id"] == "way/23585339"]
    check("H.J. Patterson Hall is found", len(patterson) == 1,
          f"{len(buildings)} buildings, {report}")
    if patterson:
        check("its height tag reads 21.58",
              abs(patterson[0]["height_m"] - 21.58) < 0.01,
              f"{patterson[0]['height_m']}")
        ring = patterson[0]["outline"]
        check("its outline is a closed ring of nodes",
              len(ring) > 4 and ring[0] == ring[-1], f"{len(ring)} points")


def test_satellite_structure(cache: Path) -> None:
    print("satellite imagery structure")
    frame = geo.GeoFrame(38.9869, -76.9426)
    side, zoom = 150, 18
    image, georef = sources.fetch_satellite_image(frame, side, zoom, "google", cache / "sat")
    expected_px = side / geo.ground_resolution_m_per_px(38.9869, zoom)
    check("image spans the requested square",
          abs(image.width - expected_px) < 3 and abs(image.height - expected_px) < 4,
          f"{image.width}x{image.height} px, expected about {expected_px:.0f}")

    import numpy as np
    pixels = np.asarray(image.convert("L"), dtype=np.float64)
    check("image holds real content, not a blank tile", float(pixels.std()) > 10,
          f"std {pixels.std():.1f}")

    # The georef must place the frame center at the image center.
    x, y = georef.latlon_to_raster_px(38.9869, -76.9426)
    check("frame center lands at the image center",
          abs(x - image.width / 2) < 3 and abs(y - image.height / 2) < 3,
          f"({x:.1f}, {y:.1f}) in {image.width}x{image.height}")


def test_terrarium_decode() -> None:
    print("terrarium decoding")
    import numpy as np
    # 32768 encodes 0 m: R=128, G=0, B=0. One G step is 1 m, one B step 1/256.
    raster = np.array([[[128, 0, 0], [128, 100, 128]]], dtype=np.uint8)
    decoded = sources.decode_terrarium(raster)
    check("terrarium zero point decodes to 0 m", abs(decoded[0, 0]) < 1e-9,
          f"{decoded[0, 0]}")
    check("terrarium steps decode to 100.5 m", abs(decoded[0, 1] - 100.5) < 1e-9,
          f"{decoded[0, 1]}")
    check("height parser reads meters and feet",
          sources.parse_height_m("21.58") == 21.58
          and abs(sources.parse_height_m("72'") - 21.9456) < 1e-6
          and sources.parse_height_m("22 m") == 22.0
          and sources.parse_height_m("tall") is None)


def test_multipolygon_assembly() -> None:
    print("multipolygon assembly, canned Overpass reply")

    def way_points(*latlon):
        return [{"lat": lat, "lon": lon} for lat, lon in latlon]

    # The outer square arrives as two open fragments, the second reversed,
    # the way Overpass hands relation members over. The inner square is a
    # courtyard.
    a, b, c, d = (10.0, 10.0), (10.0, 11.0), (11.0, 11.0), (11.0, 10.0)
    hole = [(10.4, 10.4), (10.4, 10.6), (10.6, 10.6), (10.6, 10.4), (10.4, 10.4)]
    elements = [{
        "type": "relation", "id": 7,
        "tags": {"building": "yes", "building:levels": "2"},
        "members": [
            {"role": "outer", "geometry": way_points(a, b, c)},
            {"role": "outer", "geometry": way_points(a, d, c)},
            {"role": "inner", "geometry": way_points(*hole)},
        ],
    }]
    buildings, report = sources.buildings_from_overpass(elements)
    check("two open fragments close into one outer ring",
          len(buildings) == 1 and report["relation_rings"] == 1, str(report))
    building = buildings[0]
    outline = building["outline"]
    check("the assembled outline is a closed ring",
          len(outline) >= 5 and outline[0] == outline[-1], f"{len(outline)} points")
    check("the courtyard lands in the outer ring that contains it",
          len(building["holes"]) == 1 and report["holes"] == 1)
    check("levels resolve to meters",
          building["height_m"] == 2 * sources.METERS_PER_BUILDING_LEVEL)

    orphan = [{"type": "relation", "id": 8, "tags": {"building": "yes"},
               "members": [{"role": "outer",
                            "geometry": way_points(a, b, c)}]}]
    _, report = sources.buildings_from_overpass(orphan)
    check("a fragment with no partner counts as skipped",
          report["skipped_open_rings"] == 1 and report["relation_rings"] == 0,
          str(report))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", type=Path,
                    default=Path(tempfile.gettempdir()) / "scenegen-test-tiles")
    args = ap.parse_args()

    test_terrarium_decode()
    test_multipolygon_assembly()
    test_elevation_known_points(args.cache)
    test_satellite_structure(args.cache)
    test_buildings_known_footprints()

    failed = [name for name, ok, _ in CHECKS if not ok]
    print(f"\n{len(CHECKS) - len(failed)} of {len(CHECKS)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
