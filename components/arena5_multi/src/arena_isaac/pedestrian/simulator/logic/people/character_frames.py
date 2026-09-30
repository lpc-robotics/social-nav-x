"""Quaternion conversion between ROS and Isaac People character frames.

ROS planar poses use local ``+X`` as the body-forward axis.  Isaac People
characters use local ``-Y`` as their visual forward axis.  The character root
therefore needs a positive 90 degree yaw offset when a ROS orientation is
rendered, and the inverse offset when the graph transform is reported to ROS.
"""

import math
from collections.abc import Sequence


ROS_FORWARD_AXIS = (1.0, 0.0, 0.0)
CHARACTER_FORWARD_AXIS = (0.0, -1.0, 0.0)

_HALF_SQRT_TWO = math.sqrt(0.5)
_ROS_TO_CHARACTER_OFFSET = (0.0, 0.0, _HALF_SQRT_TWO, _HALF_SQRT_TWO)
_CHARACTER_TO_ROS_OFFSET = (0.0, 0.0, -_HALF_SQRT_TWO, _HALF_SQRT_TWO)


def normalize_quaternion(quaternion: Sequence[float]) -> tuple[float, ...]:
    """Return a finite unit quaternion in ``(x, y, z, w)`` order."""

    if len(quaternion) != 4:
        raise ValueError("a quaternion must have exactly four components")
    values = tuple(float(value) for value in quaternion)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion components must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError("a quaternion must have non-zero norm")
    return tuple(value / norm for value in values)


def _multiply_quaternions(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    """Compose two normalized ``(x, y, z, w)`` quaternions."""

    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def ros_to_character_quaternion(
    quaternion: Sequence[float],
) -> tuple[float, ...]:
    """Convert a ROS ``+X``-forward orientation to Isaac's ``-Y`` asset frame."""

    logical = normalize_quaternion(quaternion)
    return normalize_quaternion(
        _multiply_quaternions(logical, _ROS_TO_CHARACTER_OFFSET)
    )


def character_to_ros_quaternion(
    quaternion: Sequence[float],
) -> tuple[float, ...]:
    """Convert an Isaac ``-Y``-forward graph orientation back to ROS semantics."""

    rendered = normalize_quaternion(quaternion)
    return normalize_quaternion(
        _multiply_quaternions(rendered, _CHARACTER_TO_ROS_OFFSET)
    )
