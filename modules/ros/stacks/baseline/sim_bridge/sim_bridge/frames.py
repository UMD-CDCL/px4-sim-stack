"""The read side of the frame tree.

scene_tf publishes the tree and every node that casts a ray reads it through
here, so they all ask one question one way: where was this camera when the
frame was taken.

Why there is no velocity correction here
----------------------------------------
A pose is interpolated, never extrapolated. tf2 already interpolates between
the two transforms bracketing the requested time, and that is exact as long as
each transform is stamped with the instant its data was valid. Correcting a
pose afterwards by linear and angular velocity times a delay is the
first-order form of the same answer, and it adds the velocity estimate's noise
and fails under acceleration. So the correction belongs at the publisher, as
one delay per chain: see vehicle_delay and gimbal_delay in scene_tf.

That leaves one rule here, and no per-node exceptions to it: ask for the time
you mean. A lookup that lands outside the recorded span returns None rather
than the nearest pose, because during motion the nearest pose is the one
furthest from the truth.
"""

from __future__ import annotations

from typing import NamedTuple

from rclpy.duration import Duration
from rclpy.time import Time

from sim_bridge.runtime import tf_buffer

# How much history the buffer keeps. It has to cover the longest delay between
# a frame being captured and its detections arriving, with room to spare.
CACHE_S = 10.0


class CameraPose(NamedTuple):
    position: tuple
    rotation: tuple
    stamp: object    # the transform's own stamp, for messages built from it


class CameraFrame:
    """One camera's pose in the reference frame, over time.

    Attach one to a node. `misses` counts lookups that found no pose, for the
    node's own report line.
    """

    def __init__(self, node, optical_frame: str, reference_frame: str = "map",
                 cache_s: float = CACHE_S) -> None:
        self.node = node
        self.optical = optical_frame
        self.reference = reference_frame
        self.buffer = tf_buffer(node, cache_time=Duration(seconds=cache_s))
        self.hits = 0
        self.misses = 0
        self.fallbacks = 0

    def at(self, stamp, timeout_s: float = 0.0,
           or_latest: bool = False) -> CameraPose | None:
        """The pose at one instant, or None.

        or_latest falls back to the newest pose on record. Only a display
        path should ask for that: it trades a wrong answer for a missing one,
        which is the right trade for a picture and the wrong one for a
        measurement.
        """
        pose = self._lookup(_as_time(stamp), timeout_s)
        if pose is None and or_latest:
            pose = self._lookup(Time(), 0.0)
            if pose is not None:
                self.fallbacks += 1
        self._count(pose)
        return pose

    def latest(self, timeout_s: float = 0.0) -> CameraPose | None:
        """The newest pose on record. For a consumer that asks about now,
        rather than about a frame that was captured earlier."""
        pose = self._lookup(Time(), timeout_s)
        self._count(pose)
        return pose

    def reset_counts(self) -> None:
        self.hits = self.misses = self.fallbacks = 0

    def _count(self, pose) -> None:
        if pose is None:
            self.misses += 1
        else:
            self.hits += 1

    def _lookup(self, when: Time, timeout_s: float) -> CameraPose | None:
        try:
            tf = self.buffer.lookup_transform(
                self.reference, self.optical, when,
                timeout=Duration(seconds=timeout_s))
        except Exception:    # noqa: BLE001 - lookup raises several types
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return CameraPose((t.x, t.y, t.z), (r.x, r.y, r.z, r.w),
                          tf.header.stamp)


def _as_time(stamp) -> Time:
    """Take either a message stamp or an rclpy Time."""
    return stamp if isinstance(stamp, Time) else Time.from_msg(stamp)
