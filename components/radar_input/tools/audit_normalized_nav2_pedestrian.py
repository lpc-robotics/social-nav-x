"""Observe a real moving HuNav person in the normalized-only local costmap."""

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Odometry
from people_msgs.msg import People
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(".workspaces/laserscan-v1/log/normalized_nav2_pedestrian.json"),
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("audit_normalized_nav2_pedestrian", use_global_arguments=False)
    latest = {"people": None, "odom": None, "clock": None}
    node.create_subscription(People, "/people", lambda m: latest.__setitem__("people", m), 10)
    node.create_subscription(Odometry, "/odom", lambda m: latest.__setitem__("odom", m), 10)
    node.create_subscription(Clock, "/clock", lambda m: latest.__setitem__("clock", m), 10)
    client = node.create_client(GetCostmap, "/local_costmap/get_costmap")
    assert client.wait_for_service(timeout_sec=20), "local costmap service unavailable"

    def spin_map():
        future = client.call_async(GetCostmap.Request())
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert future.done(), "GetCostmap timed out"
        message = future.result().map
        grid = np.asarray(message.data, dtype=np.uint8).reshape(
            message.metadata.size_y, message.metadata.size_x
        )
        return grid, message.metadata

    def cells_near(grid, metadata, position, radius):
        yy, xx = np.indices(grid.shape)
        x = metadata.origin.position.x + (xx + 0.5) * metadata.resolution
        y = metadata.origin.position.y + (yy + 0.5) * metadata.resolution
        mask = (x - position[0]) ** 2 + (y - position[1]) ** 2 <= radius**2
        return grid[mask]

    records = []
    marked = None
    cleared = None
    initial_robot = None
    last_sample = -math.inf
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and cleared is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            clock, people, odom = (latest[key] for key in ("clock", "people", "odom"))
            if clock is None or people is None or odom is None:
                continue
            sim_time = clock.clock.sec + clock.clock.nanosec / 1e9
            if sim_time - last_sample < 0.25:
                continue
            last_sample = sim_time
            robot = [odom.pose.pose.position.x, odom.pose.pose.position.y]
            if initial_robot is None:
                initial_robot = robot
            target = next((p for p in people.people if p.name == "threatening"), None)
            if target is None:
                continue
            position = [target.position.x, target.position.y]
            grid, metadata = spin_map()
            region = cells_near(grid, metadata, position, 0.55)
            record = {
                "sim_time": sim_time,
                "position": position,
                "robot": robot,
                "target_lethal": int(np.sum(region == 254)),
            }
            records.append(record)
            if marked is None and record["target_lethal"] > 0:
                marked = record.copy()
                print("NORMALIZED_PERSON_MARKED", json.dumps(marked), flush=True)
            if marked is None or math.dist(position, marked["position"]) < 1.5:
                continue
            old = cells_near(grid, metadata, marked["position"], 0.35)
            if old.size and int(old.max()) == 0:
                cleared = {
                    "sim_time": sim_time,
                    "position": position,
                    "old_lethal": int(np.sum(old == 254)),
                    "old_max": int(old.max()),
                    "movement": math.dist(position, marked["position"]),
                }
                print("NORMALIZED_PERSON_CLEARED", json.dumps(cleared), flush=True)

        assert marked is not None, "no real pedestrian marking observed"
        assert cleared is not None, "old pedestrian position did not clear"
        displacement = math.dist(initial_robot, records[-1]["robot"])
        assert displacement < 0.01, f"robot moved {displacement} m"
        result = {
            "passed": True,
            "marked": marked,
            "cleared": cleared,
            "clearing_latency_sim_seconds": cleared["sim_time"] - marked["sim_time"],
            "robot_displacement": displacement,
            "records": records,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2))
        print("NORMALIZED_NAV2_PEDESTRIAN_PASS", json.dumps({
            key: value for key, value in result.items() if key != "records"
        }), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
