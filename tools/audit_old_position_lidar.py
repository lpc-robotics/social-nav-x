"""Check whether LaserScan itself supplies a ray through one recorded old pedestrian cell."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

root = Path(__file__).resolve().parents[1]
pedestrian = json.loads((root / "audit/real_pedestrian_costmap.json").read_text())
x, y = pedestrian["marked"]["position"]
robot_x, robot_y = pedestrian["robot_initial"]
angle = math.atan2(y - robot_y, x - robot_x)
rclpy.init()
node = Node("audit_old_position_lidar", use_global_arguments=False)
samples = []


def callback(message):
    center = round((angle - message.angle_min) / message.angle_increment)
    samples.extend(message.ranges[max(0, center - 3):center + 4])


node.create_subscription(LaserScan, "/lidar", callback, qos_profile_sensor_data)
deadline = time.monotonic() + 20
while len(samples) < 140 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.2)
node.destroy_node()
rclpy.shutdown()
valid = [value for value in samples if math.isfinite(value) and value > 0]
invalid = [value for value in samples if not (math.isfinite(value) and value > 0)]
result = {
    "old_position": [x, y], "bearing_rad": angle,
    "beam_samples": len(samples), "valid_samples": len(valid),
    "invalid_samples": len(invalid), "valid_ranges": valid,
    "invalid_values": sorted(set(invalid)),
}
(root / "audit/real_old_position_lidar.json").write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
