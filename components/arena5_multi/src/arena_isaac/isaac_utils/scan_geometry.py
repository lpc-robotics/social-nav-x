"""Pure geometry and REP-117 semantics for normalized planar laser scans."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ScanLayout:
    angles: np.ndarray
    angle_min: float
    angle_max: float
    angle_increment: float


@dataclass(frozen=True)
class ScanObservation:
    """Finite control input plus masks that preserve the source semantics."""

    ranges: np.ndarray
    hit: np.ndarray
    no_return: np.ndarray
    too_close: np.ndarray
    unknown: np.ndarray
    usable_ray: np.ndarray
    obstacle: np.ndarray


def normalized_scan_layout(samples: int, angle_min: float, angle_max: float) -> ScanLayout:
    """Build an equally spaced layout without duplicating a full-circle endpoint."""
    samples = int(samples)
    angle_min = float(angle_min)
    angle_max = float(angle_max)
    if samples < 1:
        raise ValueError("samples must be at least one")
    if not np.isfinite([angle_min, angle_max]).all() or angle_max <= angle_min:
        raise ValueError("angle_min and angle_max must be finite and increasing")

    span = angle_max - angle_min
    full_circle = math.isclose(span, 2.0 * math.pi, rel_tol=0.0, abs_tol=1e-3)
    if samples == 1:
        increment = 0.0
    elif full_circle:
        # Treat near-2pi URDF decimals (for example 6.28318) as the
        # canonical full circle.  Using exactly 2pi/N avoids a tiny seam and
        # still omits the duplicated +pi endpoint.
        increment = 2.0 * math.pi / samples
    else:
        increment = span / (samples - 1)

    angles = angle_min + np.arange(samples, dtype=np.float64) * increment
    return ScanLayout(
        angles=angles,
        angle_min=angle_min,
        angle_max=float(angles[-1]),
        angle_increment=float(increment),
    )


def camera_sample_map(angles: np.ndarray, width: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Map ROS scan angles to nearest pixel-centre rays in four 90 degree views."""
    angles = np.asarray(angles, dtype=np.float64)
    width = int(width)
    if angles.ndim != 1 or not np.isfinite(angles).all():
        raise ValueError("angles must be a finite one-dimensional array")
    if width < 3:
        raise ValueError("camera width must be at least three")

    quarter_turn = math.pi / 2.0
    view_index = np.floor(angles / quarter_turn + 0.5).astype(np.int64) % 4
    view_yaw = view_index.astype(np.float64) * quarter_turn
    delta = (angles - view_yaw + math.pi) % (2.0 * math.pi) - math.pi
    if np.any(np.abs(delta) > math.pi / 4.0 + 1e-9):
        raise ValueError("an angle could not be assigned to a 90 degree view")

    lateral = np.tan(delta)
    pixel_float = (width * (1.0 - lateral) - 1.0) / 2.0
    pixel_index = np.clip(np.rint(pixel_float), 0, width - 1).astype(np.int64)

    pixel_lateral = 1.0 - 2.0 * (pixel_index.astype(np.float64) + 0.5) / width
    sampled_angles = view_yaw + np.arctan(pixel_lateral)
    sampled_angles = (sampled_angles + math.pi) % (2.0 * math.pi) - math.pi
    return view_index, pixel_index, sampled_angles


def depth_views_to_ranges(
    depth_views: np.ndarray,
    view_index: np.ndarray,
    pixel_index: np.ndarray,
    range_min: float,
    range_max: float,
    *,
    noise_mean: float = 0.0,
    noise_stddev: float = 0.0,
    range_resolution: float = 0.0,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """
    Convert renderer depth into REP-117 hit/no-return/too-close/unknown values.

    The installed ``distance_to_camera`` contract defines zero as no rendered
    object, while this Isaac 5.1 installation normally emits positive infinity.
    Negative values, NaN and negative infinity remain unknown.
    """
    depth_views = np.asarray(depth_views, dtype=np.float64)
    view_index = np.asarray(view_index, dtype=np.int64)
    pixel_index = np.asarray(pixel_index, dtype=np.int64)
    range_min = float(range_min)
    range_max = float(range_max)
    noise_mean = float(noise_mean)
    noise_stddev = float(noise_stddev)
    range_resolution = float(range_resolution)

    if depth_views.ndim != 2 or depth_views.shape[0] != 4:
        raise ValueError("depth_views must contain four centre rows")
    if view_index.shape != pixel_index.shape or view_index.ndim != 1:
        raise ValueError("view and pixel maps must be equal one-dimensional arrays")
    if np.any((view_index < 0) | (view_index >= 4)):
        raise ValueError("view index outside [0, 3]")
    if np.any((pixel_index < 0) | (pixel_index >= depth_views.shape[1])):
        raise ValueError("pixel index outside the depth row")
    if not 0.0 < range_min < range_max:
        raise ValueError("expected 0 < range_min < range_max")
    if not np.isfinite(noise_mean):
        raise ValueError("noise mean must be finite")
    if noise_stddev < 0.0 or range_resolution < 0.0:
        raise ValueError("noise and resolution cannot be negative")

    depth = depth_views[view_index, pixel_index]
    ranges = np.full(depth.shape, np.nan, dtype=np.float64)

    no_return = (
        np.isposinf(depth)
        | (depth == 0.0)
        | (np.isfinite(depth) & (depth > range_max))
    )
    too_close = np.isfinite(depth) & (depth > 0.0) & (depth < range_min)
    hit = np.isfinite(depth) & (depth >= range_min) & (depth <= range_max)
    ranges[no_return] = np.inf
    ranges[too_close] = -np.inf

    hit_ranges = depth[hit].copy()
    if noise_mean != 0.0 and hit_ranges.size:
        hit_ranges += noise_mean
    if noise_stddev > 0.0 and hit_ranges.size:
        rng = rng if rng is not None else np.random.default_rng(0)
        hit_ranges += rng.normal(0.0, noise_stddev, hit_ranges.size)
    if range_resolution > 0.0 and hit_ranges.size:
        hit_ranges = np.round(hit_ranges / range_resolution) * range_resolution

    hit_indices = np.flatnonzero(hit)
    below = hit_ranges < range_min
    above = hit_ranges > range_max
    in_range = ~(below | above)
    ranges[hit_indices[below]] = -np.inf
    ranges[hit_indices[above]] = np.inf
    ranges[hit_indices[in_range]] = hit_ranges[in_range]
    return ranges.astype(np.float32)


def scan_state_masks(
    ranges: np.ndarray, range_min: float, range_max: float
) -> dict[str, np.ndarray]:
    """Classify normalized scan values and reject unexpected finite sentinels."""
    ranges = np.asarray(ranges, dtype=np.float32)
    hit = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    no_return = np.isposinf(ranges)
    too_close = np.isneginf(ranges)
    unknown = ~(hit | no_return | too_close)
    return {
        "hit": hit,
        "no_return": no_return,
        "too_close": too_close,
        "unknown": unknown,
    }


def adapt_scan_for_control(
    ranges: np.ndarray,
    range_min: float,
    range_max: float,
    *,
    unknown_fill: float = 0.0,
) -> ScanObservation:
    """Produce finite DRL/MPC input while retaining a mask for every state."""
    if (
        not np.isfinite([range_min, range_max, unknown_fill]).all()
        or not 0.0 < range_min < range_max
    ):
        raise ValueError("control adapter requires finite 0 < range_min < range_max and fill")
    ranges = np.asarray(ranges, dtype=np.float32)
    masks = scan_state_masks(ranges, range_min, range_max)
    finite = np.full(ranges.shape, float(unknown_fill), dtype=np.float32)
    finite[masks["hit"]] = ranges[masks["hit"]]
    finite[masks["no_return"]] = float(range_max)
    finite[masks["too_close"]] = 0.0
    return ScanObservation(
        ranges=finite,
        hit=masks["hit"],
        no_return=masks["no_return"],
        too_close=masks["too_close"],
        unknown=masks["unknown"],
        usable_ray=masks["hit"] | masks["no_return"],
        obstacle=masks["hit"] | masks["too_close"],
    )
