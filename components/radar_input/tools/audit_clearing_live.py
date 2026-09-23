"""Record the real scene's clearing stream without changing runtime state."""
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs_py.point_cloud2 import read_points_numpy

root = Path(__file__).resolve().parents[1]
rclpy.init()
node = Node("audit_depth_clearing", use_global_arguments=False)
samples = []
lidar_samples = []


def callback(message):
    xyz = read_points_numpy(message, field_names=("x", "y", "z"), skip_nans=False)
    distance = np.linalg.norm(xyz, axis=1)
    samples.append({
        "arrival_monotonic": time.monotonic(),
        "stamp": message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
        "frame_id": message.header.frame_id,
        "points": int(xyz.shape[0]),
        "finite_points": int(np.isfinite(xyz).all(axis=1).sum()),
        "min_distance": float(np.nanmin(distance)),
        "max_distance": float(np.nanmax(distance)),
    })


def lidar_callback(message):
    lidar_samples.append({
        "arrival_monotonic": time.monotonic(),
        "stamp": message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
    })


node.create_subscription(PointCloud2, "/lidar_clearing", callback, qos_profile_sensor_data)
node.create_subscription(LaserScan, "/lidar", lidar_callback, qos_profile_sensor_data)
deadline = time.monotonic() + 30
while (len(samples) < 25 or len(lidar_samples) < 25) and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
assert len(samples) >= 10, f"Only received {len(samples)} clearing frames"
delta = np.diff([sample["stamp"] for sample in samples])
wall_delta = np.diff([sample["arrival_monotonic"] for sample in samples])
lidar_delta = np.diff([sample["stamp"] for sample in lidar_samples])
lidar_wall_delta = np.diff([sample["arrival_monotonic"] for sample in lidar_samples])
summary = {
    "samples": samples,
    "sim_period_median": float(np.median(delta)),
    "sim_frequency": float(1 / np.median(delta)),
    "wall_frequency": float(1 / np.median(wall_delta)),
    "lidar_sim_frequency": float(1 / np.median(lidar_delta)),
    "lidar_wall_frequency": float(1 / np.median(lidar_wall_delta)),
    "all_frames_lidar_link": all(s["frame_id"] == "lidar_link" for s in samples),
    "all_points_finite": all(s["points"] == s["finite_points"] for s in samples),
    "all_ranges_safe": all(s["min_distance"] > 0.08 and
                            s["max_distance"] <= 12.0001 for s in samples),
}
assert math.isclose(summary["sim_frequency"], 10.0, rel_tol=0.03), summary
assert summary["all_frames_lidar_link"] and summary["all_points_finite"]
assert summary["all_ranges_safe"]
(root / "audit/live_clearing_stream.json").write_text(json.dumps(summary, indent=2))
print(json.dumps({key: value for key, value in summary.items() if key != "samples"}, indent=2))
