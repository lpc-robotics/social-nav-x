"""Conservative endpoints from rendered radial depth, never from lidar sentinels."""
import numpy as np


def horizontal_directions(width, yaw):
    """Pixel-centre rays for a 90 degree pinhole camera, ROS x-forward/z-up."""
    lateral = 1.0 - 2.0 * (np.arange(width) + 0.5) / width
    angle = yaw + np.arctan(lateral)
    return np.column_stack((np.cos(angle), np.sin(angle), np.zeros(width)))


def clearing_endpoints(depth, directions, range_min, range_max, margin=0.05):
    """Only positive finite depth and explicit +Inf from the depth renderer.

    Zero, negative values, NaN and -Inf remain unobserved. Finite rays stop
    before the measured surface; +Inf means no rendered surface inside the
    camera clipping volume and ends at range_max. These points MUST only be
    used by an observation source with marking=false.
    """
    depth = np.asarray(depth, dtype=np.float64)
    directions = np.asarray(directions, dtype=np.float64)
    if depth.ndim != 1 or directions.shape != (depth.size, 3):
        raise ValueError("Expected one radial depth per 3D unit direction")
    valid = (np.isfinite(depth) & (depth > range_min + margin)) | np.isposinf(depth)
    distance = np.minimum(depth[valid] - margin, range_max)
    return (directions[valid] * distance[:, None]).astype(np.float32)
