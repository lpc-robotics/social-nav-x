"""Timestamped odometry interpolation, independent of ROS and wall-clock timers."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float


class Timeline:
    def __init__(self):
        self.samples = deque()
        self.wall_seen = None

    def append(self, sample: Sample, wall: float):
        if sample.stamp_ns < 0 or not all(math.isfinite(v) for v in
                (sample.x, sample.y, sample.yaw, sample.vx, sample.vy, sample.wz)):
            raise ValueError("invalid odometry")
        if self.samples and sample.stamp_ns < self.samples[-1].stamp_ns:
            raise ValueError("odometry time rewound")
        if self.samples and sample.stamp_ns == self.samples[-1].stamp_ns:
            return  # duplicate samples must not keep a dead source healthy
        self.samples.append(sample)
        self.wall_seen = wall
        while len(self.samples) > 2 and self.samples[1].stamp_ns < sample.stamp_ns - 5_000_000_000:
            self.samples.popleft()

    def at(self, stamp_ns):
        if not self.samples or not self.samples[0].stamp_ns <= stamp_ns <= self.samples[-1].stamp_ns:
            raise ValueError("snapshot outside odometry history")
        for sample in self.samples:
            if sample.stamp_ns == stamp_ns:
                return sample
        left = self.samples[0]
        for right in self.samples:
            if right.stamp_ns > stamp_ns:
                t = (stamp_ns - left.stamp_ns) / (right.stamp_ns - left.stamp_ns)
                yaw_delta = math.atan2(math.sin(right.yaw - left.yaw), math.cos(right.yaw - left.yaw))
                lerp = lambda a, b: a + t * (b - a)
                return Sample(stamp_ns, lerp(left.x, right.x), lerp(left.y, right.y),
                              left.yaw + t * yaw_delta, lerp(left.vx, right.vx),
                              lerp(left.vy, right.vy), lerp(left.wz, right.wz))
            left = right
        raise ValueError("no interpolation bracket")
