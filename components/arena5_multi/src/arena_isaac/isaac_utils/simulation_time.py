"""Use the ROS graph clock for new-mode observations and command extrapolation.

World.reset() resets World.current_time, while IsaacReadSimulationTime can keep
the accumulated simulation time used by /clock, odometry, and TF. Mixing those
epochs labels rendered poses in the past and underestimates command latency.
Legacy scenes retain their original World clock unless explicitly opted in.
"""
import math
import os


def observation_time(world_time):
    if os.environ.get("ARENA_MULTI_CLOCK_ALIGNMENT", "false") != "true":
        return world_time
    import omni.graph.core as og
    stamp = float(og.Controller.attribute(
        "/World/publish_time/read_simulation_time.outputs:simulationTime").get())
    if not math.isfinite(stamp) or stamp < 0:
        raise ValueError("invalid ROS simulation clock")
    return stamp


def command_age(world_time, stamp):
    return max(0.0, observation_time(world_time) - stamp) if stamp > 0 else 0.0
