"""Map-plane peer geometry and finite social-space intersection."""

import math

from ..event_extractor import normalize_angle
from .config import PeerThresholds
from .model import PairGeometry, PairMemory, SceneSnapshot


def low_latch(value, previous, enter, exit_):
    return value < exit_ if previous else value <= enter


def point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    norm = dx * dx + dy * dy
    fraction = 0.0 if norm <= 1e-24 else max(0.0, min(1.0,
        ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / norm))
    return math.hypot(point[0] - start[0] - fraction * dx,
                      point[1] - start[1] - fraction * dy)


def segment_distance(a, b, c, d):
    """Minimum distance of two finite 2D segments, including degenerate ones."""
    ux, uy = b[0] - a[0], b[1] - a[1]
    vx, vy = d[0] - c[0], d[1] - c[1]
    denominator = ux * vy - uy * vx
    if abs(denominator) > 1e-12:
        wx, wy = c[0] - a[0], c[1] - a[1]
        t = (wx * vy - wy * vx) / denominator
        s = (wx * uy - wy * ux) / denominator
        if 0 <= t <= 1 and 0 <= s <= 1:
            return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def evaluate_geometry(snapshot: SceneSnapshot, memory: PairMemory,
                      config: PeerThresholds) -> PairGeometry:
    (_, a), (_, b) = snapshot.humans
    distance = math.hypot(a.x - b.x, a.y - b.y)
    distinct = distance > a.radius + b.radius and distance > 1e-12
    errors = (abs(math.degrees(normalize_angle(math.atan2(b.y-a.y, b.x-a.x)-a.yaw))),
              abs(math.degrees(normalize_angle(math.atan2(a.y-b.y, a.x-b.x)-b.yaw))))
    visible = tuple(distinct and (
        distance < config.visible_exit_distance and error < config.visible_exit_half_fov_deg
        if previous else distance <= config.visible_enter_distance and error <= config.visible_enter_half_fov_deg
    ) for error, previous in zip(errors, memory.visible))
    near = low_latch(distance, memory.near, config.near_enter_distance, config.near_exit_distance)
    gaze = all(visible) and low_latch(max(errors), memory.gaze,
                                     config.gaze_enter_deg, config.gaze_exit_deg)
    speed = max(math.hypot(a.vx, a.vy), math.hypot(b.vx, b.vy))
    stationary = low_latch(speed, memory.stationary, config.stationary_enter_speed,
                           config.stationary_exit_speed)
    anchors = memory.anchors or ((a.x, a.y), (b.x, b.y))
    robot = (snapshot.robot.x, snapshot.robot.y)
    space_distance = point_segment_distance(robot, *anchors)
    swept_distance = (segment_distance(memory.previous_robot, robot, *anchors)
                      if memory.active and memory.previous_robot is not None else space_distance)
    intrusion = low_latch(space_distance, memory.intrusion_active,
                          snapshot.robot.radius + config.intrusion_enter_margin,
                          snapshot.robot.radius + config.intrusion_exit_margin)
    return PairGeometry(distance, errors, visible, near, gaze, stationary,
                        distinct and near and gaze and stationary,
                        space_distance, swept_distance, intrusion)
