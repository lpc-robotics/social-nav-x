"""Verify a real HuNav pedestrian is marked, moves, and leaves a clear master cell."""
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Odometry
from people_msgs.msg import People
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

root = Path(__file__).resolve().parents[1]
rclpy.init()
node = Node("audit_real_pedestrian_costmap", use_global_arguments=False)
latest = {"people": None, "odom": None, "clock": None}
mode = os.environ.get("COSTMAP_AUDIT_MODE", "fixed")
assert mode in ("fixed", "baseline")
node.create_subscription(People, "/people", lambda m: latest.__setitem__("people", m), 10)
node.create_subscription(Odometry, "/odom", lambda m: latest.__setitem__("odom", m), 10)
node.create_subscription(Clock, "/clock", lambda m: latest.__setitem__("clock", m), 10)
client = node.create_client(GetCostmap, "/local_costmap/get_costmap")
assert client.wait_for_service(timeout_sec=20), "local master service unavailable"


def now():
    msg = latest["clock"]
    return None if msg is None else msg.clock.sec + msg.clock.nanosec / 1e9


def get_map():
    future = client.call_async(GetCostmap.Request())
    deadline = time.monotonic() + 10
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
    assert future.done(), "GetCostmap timeout"
    msg = future.result().map
    grid = np.asarray(msg.data, dtype=np.uint8).reshape(msg.metadata.size_y, msg.metadata.size_x)
    resolution = msg.metadata.resolution
    ox = msg.metadata.origin.position.x
    oy = msg.metadata.origin.position.y
    return msg, grid, resolution, ox, oy


def region(grid, resolution, ox, oy, x, y, radius):
    yy, xx = np.indices(grid.shape)
    cx = ox + (xx + .5) * resolution
    cy = oy + (yy + .5) * resolution
    return grid[(cx - x) ** 2 + (cy - y) ** 2 <= radius ** 2]


deadline = time.monotonic() + 240
marked = None
cleared = None
baseline_final = None
departure_time = None
post_departure = []
records = []
initial_odom = None
last_sample = -math.inf
while time.monotonic() < deadline and cleared is None and baseline_final is None:
    rclpy.spin_once(node, timeout_sec=.1)
    sim_time = now()
    people = latest["people"]
    odom = latest["odom"]
    if sim_time is None or people is None or odom is None or sim_time - last_sample < .25:
        continue
    if initial_odom is None:
        initial_odom = [odom.pose.pose.position.x, odom.pose.pose.position.y]
    _, grid, resolution, ox, oy = get_map()
    last_sample = sim_time
    current = {person.name: [person.position.x, person.position.y] for person in people.people}
    target = current.get("threatening")
    record = {"sim_time": sim_time, "target": target,
              "robot": [odom.pose.pose.position.x, odom.pose.pose.position.y]}
    if target is not None:
        cells = region(grid, resolution, ox, oy, *target, .55)
        record.update(target_lethal=int(np.sum(cells == 254)), target_max=int(cells.max()) if cells.size else -1)
    records.append(record)
    if target is None:
        continue
    if marked is None:
        cells = region(grid, resolution, ox, oy, *target, .55)
        if cells.size and np.any(cells == 254):
            marked = {"sim_time": sim_time, "position": target,
                      "lethal_cells": int(np.sum(cells == 254)), "max_cost": int(cells.max())}
            np.save(root / f"audit/real_pedestrian_{mode}_marked_master.npy", grid)
            print("REAL_PEDESTRIAN_MARKED", json.dumps(marked), flush=True)
    else:
        movement = math.dist(target, marked["position"])
        if movement < 1.5 or sim_time - marked["sim_time"] < .5:
            continue
        cells = region(grid, resolution, ox, oy, *marked["position"], .35)
        candidate = {"sim_time": sim_time, "position": target, "movement": movement,
                     "old_lethal_cells": int(np.sum(cells == 254)),
                     "old_max_cost": int(cells.max()) if cells.size else -1}
        if mode == "fixed" and cells.size and candidate["old_max_cost"] == 0:
            cleared = candidate
            np.save(root / "audit/real_pedestrian_cleared_master.npy", grid)
            print("REAL_PEDESTRIAN_CLEARED", json.dumps(cleared), flush=True)
        elif mode == "baseline":
            if departure_time is None:
                departure_time = sim_time
            candidate["since_departure"] = sim_time - departure_time
            post_departure.append(candidate)
            if candidate["since_departure"] >= 5.0:
                baseline_final = candidate
                np.save(root / "audit/real_pedestrian_baseline_final_master.npy", grid)
                print("REAL_PEDESTRIAN_BASELINE_FINAL", json.dumps(baseline_final), flush=True)

assert marked is not None, "No real pedestrian marking observed"
final_odom = records[-1]["robot"]
robot_displacement = math.dist(initial_odom, final_odom)
assert robot_displacement < .01, f"Robot moved {robot_displacement} m"
if mode == "fixed":
    assert cleared is not None, "Real pedestrian old position did not clear"
    result = {"marked": marked, "cleared": cleared,
              "clearing_latency_sim_seconds": cleared["sim_time"] - marked["sim_time"],
              "robot_initial": initial_odom, "robot_final": final_odom,
              "robot_displacement": robot_displacement, "samples": records}
    output = root / "audit/real_pedestrian_costmap.json"
    label = "REAL_PEDESTRIAN_COSTMAP_PASS"
else:
    assert baseline_final is not None, "Baseline departure window not completed"
    result = {"marked": marked, "final": baseline_final,
              "robot_initial": initial_odom, "robot_final": final_odom,
              "robot_displacement": robot_displacement,
              "post_departure": post_departure, "samples": records}
    output = root / "audit/real_pedestrian_baseline.json"
    label = "REAL_PEDESTRIAN_BASELINE_RECORDED"
output.write_text(json.dumps(result, indent=2))
print(label, json.dumps({k: v for k, v in result.items()
                         if k not in ("samples", "post_departure")}), flush=True)
node.destroy_node()
rclpy.shutdown()
