#!/usr/bin/env python3
"""Ground-truth tests for geo.py. Standard library only, no network.

Run: python3 tests/test_geo.py

The expected values come from published references, not from the code under
test:
  - WGS84 semi-minor axis b = 6356752.314245 m.
  - Length of one degree of latitude and longitude, from the standard
    truncated series (Snyder, "Map Projections", and every geodesy text):
      lat: 111132.954 - 559.822 cos(2p) + 1.175 cos(4p)  meters/degree
      lon: 111412.84 cos(p) - 93.5 cos(3p) + 0.118 cos(5p)
  - Web mercator ground resolution at the equator, zoom 0, 256 px tiles:
    156543.03392804097 m/px.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import geo

CHECKS = []


def check(name: str, condition: bool, detail: str = "") -> None:
    CHECKS.append((name, condition, detail))
    mark = "ok  " if condition else "FAIL"
    print(f"  {mark} {name}{'  ' + detail if detail and not condition else ''}")


def test_ecef_reference_points() -> None:
    x, y, z = geo.geodetic_to_ecef(0.0, 0.0, 0.0)
    check("ecef at (0,0,0) is (a,0,0)",
          abs(x - 6378137.0) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6,
          f"got {x} {y} {z}")
    x, y, z = geo.geodetic_to_ecef(90.0, 0.0, 0.0)
    check("ecef at the pole is (0,0,b)",
          abs(z - 6356752.314245) < 1e-4 and math.hypot(x, y) < 1e-3,
          f"got {x} {y} {z}")


def test_ecef_round_trip() -> None:
    worst = 0.0
    for lat, lon, alt in [(38.9869, -76.9426, 20.0), (-33.9, 151.2, 1200.0),
                          (61.2, -149.9, -30.0), (0.001, 179.999, 8848.0)]:
        x, y, z = geo.geodetic_to_ecef(lat, lon, alt)
        lat2, lon2, alt2 = geo.ecef_to_geodetic(x, y, z)
        worst = max(worst, abs(lat2 - lat) * 111e3, abs(lon2 - lon) * 111e3,
                    abs(alt2 - alt))
    check("geodetic <-> ecef round trip under 1 mm", worst < 1e-3, f"worst {worst:.2e} m")


def test_enu_against_degree_lengths() -> None:
    lat = 38.9869
    p = math.radians(lat)
    meters_per_deg_lat = 111132.954 - 559.822 * math.cos(2 * p) + 1.175 * math.cos(4 * p)
    meters_per_deg_lon = (111412.84 * math.cos(p) - 93.5 * math.cos(3 * p)
                          + 0.118 * math.cos(5 * p))
    frame = geo.GeoFrame(lat, -76.9426, 20.0)

    _, north, _ = frame.latlon_to_enu(lat + 0.01, -76.9426)
    expected = meters_per_deg_lat * 0.01
    check("0.01 deg of latitude matches the published arc length",
          abs(north - expected) < 1.0, f"got {north:.2f}, expected {expected:.2f}")

    east, _, _ = frame.latlon_to_enu(lat, -76.9426 + 0.01)
    expected = meters_per_deg_lon * 0.01
    check("0.01 deg of longitude matches the published arc length",
          abs(east - expected) < 1.0, f"got {east:.2f}, expected {expected:.2f}")


def test_enu_round_trip_and_curvature() -> None:
    frame = geo.GeoFrame(38.9869, -76.9426, 20.0)
    worst = 0.0
    for east, north in [(500.0, -300.0), (-1200.0, 950.0), (3.0, 4.0)]:
        lat, lon, alt = frame.enu_to_latlon(east, north, 0.0)
        e2, n2, u2 = frame.latlon_to_enu(lat, lon, alt)
        worst = max(worst, abs(e2 - east), abs(n2 - north), abs(u2))
    check("enu <-> lat/lon round trip under 1 mm", worst < 1e-3, f"worst {worst:.2e} m")

    # A point 1 km east at the origin's altitude sits below the tangent
    # plane by d^2/2R, about 78 mm. Flat-earth code returns exactly zero
    # here, so this check separates the two.
    lat, lon, _ = frame.enu_to_latlon(1000.0, 0.0, 0.0)
    _, _, up = frame.latlon_to_enu(lat, lon, 20.0)
    check("earth curvature appears at 1 km", -0.12 < up < -0.05, f"up {up:.4f} m")


def test_mercator_reference_points() -> None:
    x, y = geo.latlon_to_mercator_px(0.0, 0.0, 1)
    check("lat/lon origin maps to the center of the zoom-1 world",
          abs(x - 256.0) < 1e-9 and abs(y - 256.0) < 1e-9, f"got {x} {y}")

    _, y = geo.latlon_to_mercator_px(geo.MERCATOR_LAT_LIMIT_DEG, 0.0, 3)
    check("the projection limit maps to the top edge", abs(y) < 1e-6, f"y {y}")

    resolution = geo.ground_resolution_m_per_px(0.0, 0)
    check("equator zoom-0 resolution is the published constant",
          abs(resolution - 156543.03392804097) < 1e-6, f"got {resolution}")

    # (-76.9426 + 180) / 360 * 4 tiles = 1.145 at zoom 2, so tile x is 1.
    x, _ = geo.latlon_to_mercator_px(38.9869, -76.9426, 2)
    check("zoom-2 tile column for 76.94 W", geo.tile_index(x) == 1,
          f"tile {geo.tile_index(x)}")


def test_mercator_round_trip_and_scale() -> None:
    worst = 0.0
    for lat, lon in [(38.9869, -76.9426), (-45.0, 170.0), (84.0, -179.5)]:
        x, y = geo.latlon_to_mercator_px(lat, lon, 19)
        lat2, lon2 = geo.mercator_px_to_latlon(x, y, 19)
        worst = max(worst, abs(lat2 - lat), abs(lon2 - lon))
    check("mercator round trip under 1e-9 deg", worst < 1e-9, f"worst {worst:.2e}")

    # Two points 100 m apart on the ground must land 100 / resolution
    # pixels apart, within the fraction mercator distorts over 100 m.
    frame = geo.GeoFrame(38.9869, -76.9426)
    lat_b, lon_b, _ = frame.enu_to_latlon(100.0, 0.0)
    ax, ay = geo.latlon_to_mercator_px(38.9869, -76.9426, 19)
    bx, by = geo.latlon_to_mercator_px(lat_b, lon_b, 19)
    px_distance = math.hypot(bx - ax, by - ay)
    expected = 100.0 / geo.ground_resolution_m_per_px(38.9869, 19)
    check("100 m east spans the expected pixel count at zoom 19",
          abs(px_distance - expected) / expected < 3e-3,
          f"got {px_distance:.2f}, expected {expected:.2f}")


def main() -> int:
    for test in [test_ecef_reference_points, test_ecef_round_trip,
                 test_enu_against_degree_lengths, test_enu_round_trip_and_curvature,
                 test_mercator_reference_points, test_mercator_round_trip_and_scale]:
        print(test.__name__)
        test()
    failed = [name for name, ok, _ in CHECKS if not ok]
    print(f"\n{len(CHECKS) - len(failed)} of {len(CHECKS)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
