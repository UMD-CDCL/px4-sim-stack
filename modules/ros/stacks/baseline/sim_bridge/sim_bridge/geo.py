"""Where local (0, 0) is on the Earth, for the Map panel publishers.

Ground truth, the scorers, the footprints and the gimbal ROI all publish
GeoJSON for the Map panel, and they must agree on the origin: truth from one
origin and estimates from another would show a bias that is not there.

PX4 does not send GPS_GLOBAL_ORIGIN unless something asks for it, so the
fallback pairs the drone's own fix with its local position: if the aircraft
sits at local (x, y) and reports a latitude and longitude, then local (0, 0)
is that fix walked back by (x, y). gp_origin is still preferred when it does
arrive, because it is the origin rather than an estimate of it.

The projection is flat earth about the origin. Over the hundreds of meters a
scene covers, the error is far below the localization uncertainty.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

try:
    from geographic_msgs.msg import GeoPointStamped
    HAVE_GEO = True
except ImportError:          # geographic_msgs is optional in some distros
    HAVE_GEO = False

EARTH_R = 6378137.0


def lla_to_enu(lat: float, lon: float, alt: float,
               ref_lat: float, ref_lon: float,
               ref_alt: float) -> tuple[float, float, float]:
    """Flat-earth local ENU meters about a reference. Good over a few km,
    the inverse of MapOrigin.to_lla."""
    east = math.radians(lon - ref_lon) * EARTH_R * math.cos(math.radians(ref_lat))
    north = math.radians(lat - ref_lat) * EARTH_R
    return east, north, alt - ref_alt


class MapOrigin:
    """Tracks the WGS84 position of local (0, 0) for the node that owns it.

    Attach one to a node, then call `to_lla` to convert. `ready` is False
    until the vehicle has a fix, and every caller must handle that: the
    drone publishes a local position long before it publishes a usable one.

    By default the origin subscribes to the pose and fix topics itself. An
    owner that already subscribes to them passes external_updates=True and
    forwards the messages to on_local and on_fix, so the process holds one
    subscription per topic. Once gp_origin has spoken, on_local and on_fix
    decide nothing, and the default-mode subscriptions are destroyed.
    """

    def __init__(self, node, *, external_updates: bool = False) -> None:
        self.node = node
        self.lat: float | None = None
        self.lon: float | None = None
        self.alt: float | None = None
        self.from_fix = False
        self._local: tuple[float, float, float] | None = None
        self._local_sub = self._fix_sub = None
        self._drop_timer = None

        if not external_updates:
            self._local_sub = node.create_subscription(
                PoseStamped, "/mavros/local_position/pose",
                self.on_local, qos_profile_sensor_data)
            self._fix_sub = node.create_subscription(
                NavSatFix, "/mavros/global_position/global",
                self.on_fix, qos_profile_sensor_data)
        if HAVE_GEO:
            node.create_subscription(GeoPointStamped,
                                     "/mavros/global_position/gp_origin",
                                     self._on_origin, qos_profile_sensor_data)

    @property
    def ready(self) -> bool:
        return self.lat is not None

    @property
    def lla(self) -> tuple[float, float, float] | None:
        """The origin as (latitude, longitude, altitude), or None before a
        fix. Altitude is the AMSL of local z zero."""
        if self.lat is None or self.alt is None:
            return None
        return (self.lat, self.lon, self.alt)

    def _on_origin(self, msg) -> None:
        # Zero means PX4 has no origin yet, not a point off west Africa.
        if abs(msg.position.latitude) > 1e-9 or abs(msg.position.longitude) > 1e-9:
            self.lat = msg.position.latitude
            self.lon = msg.position.longitude
            self.alt = msg.position.altitude
            self.from_fix = False
            self._drop_feed_subscriptions()

    def _drop_feed_subscriptions(self) -> None:
        """Once gp_origin has spoken, on_local and on_fix decide nothing,
        so their subscriptions go. A one-shot timer does the destroying:
        a timer callback is the safe place to destroy entities, a
        subscription callback is not. No-op with external_updates."""
        if self._local_sub is None or self._drop_timer is not None:
            return
        self._drop_timer = self.node.create_timer(0.0, self._on_drop_timer)

    def _on_drop_timer(self) -> None:
        self.node.destroy_subscription(self._local_sub)
        self.node.destroy_subscription(self._fix_sub)
        self._local_sub = self._fix_sub = None
        self._drop_timer.cancel()

    def on_local(self, msg) -> None:
        p = msg.pose.position
        self._local = (p.x, p.y, p.z)

    def on_fix(self, msg) -> None:
        # Stop once gp_origin has spoken: it is the real origin, not a guess.
        if self.lat is not None and not self.from_fix:
            return
        if self._local is None or msg.status.status < 0:
            return
        x, y, z = self._local
        first = self.lat is None
        self.lat = msg.latitude - math.degrees(y / EARTH_R)
        self.lon = msg.longitude - math.degrees(
            x / (EARTH_R * math.cos(math.radians(msg.latitude))))
        self.alt = msg.altitude - z
        self.from_fix = True
        if first:
            self.node.get_logger().info(
                f"map origin from the vehicle fix: {self.lat:.7f}, {self.lon:.7f}")

    def to_lla(self, x: float, y: float) -> tuple[float, float] | None:
        """Local ENU meters to (latitude, longitude). None before a fix."""
        if self.lat is None:
            return None
        return (self.lat + math.degrees(y / EARTH_R),
                self.lon + math.degrees(
                    x / (EARTH_R * math.cos(math.radians(self.lat)))))

    def geojson_ring(self, points_xy) -> list | None:
        """The local (x, y) points as a closed GeoJSON ring of
        [longitude, latitude] pairs. None before the origin is known."""
        if self.lat is None:
            return None
        ring = [[lon, lat]
                for lat, lon in (self.to_lla(x, y) for x, y in points_xy)]
        ring.append(ring[0])
        return ring


class GroundPlane:
    """The flat plane localization projects onto, latched at takeoff.

    Every projection node used to recompute pose.z - rel_alt on each frame.
    That difference is the local height of the home point, so the plane was
    already anchored to the takeoff location, but it moved with every step
    of barometer drift between the two altitude sources. This helper takes
    the same difference once, the first time both values exist, which is on
    the ground before takeoff in any normal start, and holds it for the
    flight.

    Attach one to a node and call `z(pose_z)` with the current vehicle
    height wherever a ground height is needed. The `use_rel_alt` and
    `ground_z` parameters keep their old names and meanings: use_rel_alt
    false pins the plane to ground_z and subscribes to nothing. `rel_alt`
    stays readable for owners that need the raw value.
    """

    def __init__(self, node) -> None:
        node.declare_parameter("use_rel_alt", True)
        node.declare_parameter("ground_z", 0.0)
        self.node = node
        self.rel_alt: float | None = None
        self._pinned = float(node.get_parameter("ground_z").value)
        self._latched: float | None = None
        if node.get_parameter("use_rel_alt").value:
            node.create_subscription(Float64, "/mavros/global_position/rel_alt",
                                     self._on_rel_alt, qos_profile_sensor_data)

    def _on_rel_alt(self, msg: Float64) -> None:
        self.rel_alt = float(msg.data)

    def z(self, pose_z: float | None = None) -> float:
        """The plane height. Latches on the first call that has both the
        pose and rel_alt; before that, the ground_z parameter answers."""
        if self._latched is None and self.rel_alt is not None \
                and pose_z is not None:
            self._latched = pose_z - self.rel_alt
            self.node.get_logger().info(
                f"ground plane latched at z={self._latched:.2f}, "
                f"the takeoff altitude")
        return self._latched if self._latched is not None else self._pinned
