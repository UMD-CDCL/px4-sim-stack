"""Where local (0, 0) is on the Earth, for the Map panel publishers.

Ground truth and the scorers publish NavSatFix for the Map panel, and they
must agree on the origin: truth from one origin and estimates from another
would show a bias that is not there.

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
from sensor_msgs.msg import NavSatFix, NavSatStatus

try:
    from geographic_msgs.msg import GeoPointStamped
    HAVE_GEO = True
except ImportError:          # geographic_msgs is optional in some distros
    HAVE_GEO = False

EARTH_R = 6378137.0


class MapOrigin:
    """Tracks the WGS84 position of local (0, 0) for the node that owns it.

    Attach one to a node, then call `to_lla` to convert. `ready` is False
    until the vehicle has a fix, and every caller must handle that: the
    drone publishes a local position long before it publishes a usable one.
    """

    def __init__(self, node) -> None:
        self.node = node
        self.lat: float | None = None
        self.lon: float | None = None
        self.from_fix = False
        self._local: tuple[float, float] | None = None

        node.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_local, qos_profile_sensor_data)
        node.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._on_fix, qos_profile_sensor_data)
        if HAVE_GEO:
            node.create_subscription(GeoPointStamped,
                                     "/mavros/global_position/gp_origin",
                                     self._on_origin, qos_profile_sensor_data)

    @property
    def ready(self) -> bool:
        return self.lat is not None

    def _on_origin(self, msg) -> None:
        # Zero means PX4 has no origin yet, not a point off west Africa.
        if abs(msg.position.latitude) > 1e-9 or abs(msg.position.longitude) > 1e-9:
            self.lat = msg.position.latitude
            self.lon = msg.position.longitude
            self.from_fix = False

    def _on_local(self, msg) -> None:
        self._local = (msg.pose.position.x, msg.pose.position.y)

    def _on_fix(self, msg) -> None:
        # Stop once gp_origin has spoken: it is the real origin, not a guess.
        if self.lat is not None and not self.from_fix:
            return
        if self._local is None or msg.status.status < 0:
            return
        x, y = self._local
        first = self.lat is None
        self.lat = msg.latitude - math.degrees(y / EARTH_R)
        self.lon = msg.longitude - math.degrees(
            x / (EARTH_R * math.cos(math.radians(msg.latitude))))
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

    def navsat_fix(self, x: float, y: float, frame_id: str, stamp,
                   xy_std: float = 0.0) -> NavSatFix | None:
        """A NavSatFix at local (x, y), or None before the origin is known.

        xy_std becomes the position covariance, which the Map panel draws as
        an accuracy ring of that radius. Altitude is NaN: a point on the
        ground plane has no height worth plotting.
        """
        ll = self.to_lla(x, y)
        if ll is None:
            return None
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = frame_id
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude = ll
        fix.altitude = float("nan")
        if xy_std > 0.0:
            variance = xy_std * xy_std
            fix.position_covariance = [variance, 0.0, 0.0,
                                       0.0, variance, 0.0,
                                       0.0, 0.0, 0.0]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return fix
