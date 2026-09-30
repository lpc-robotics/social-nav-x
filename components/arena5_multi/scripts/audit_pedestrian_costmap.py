#!/usr/bin/env python3
"""Read-only audit: compare HuNav positions with nearby local lethal cells.

This is evidence of marking near people, not proof of obstacle identity or clearing.
Run while robots are stationary; samples with >0.5 sim-second skew are excluded.
"""
import argparse
import json
import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from hunav_msgs.msg import Agents
from nav_msgs.msg import OccupancyGrid

parser = argparse.ArgumentParser()
parser.add_argument('--seconds', type=float, default=45)
parser.add_argument('--output', required=True)
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('audit_pedestrian_costmap')
data = {}
subs = []
for topic, typ in [('/multirobot/hunav/agents', Agents)] + [(f'/robot_{i}/local_costmap/costmap', OccupancyGrid) for i in (1, 2)]:
    subs.append(node.create_subscription(typ, topic, lambda msg, t=topic: data.__setitem__(t, msg), qos_profile_sensor_data))
results = {str(i): {'samples': 0, 'marked_ids': [], 'latest': []} for i in (1, 2)}
def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
end = time.monotonic() + args.seconds
last = 0
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.2)
    if time.monotonic() - last < .5 or '/multirobot/hunav/agents' not in data:
        continue
    last = time.monotonic()
    agents = data['/multirobot/hunav/agents']
    for i in (1, 2):
        grid = data.get(f'/robot_{i}/local_costmap/costmap')
        if grid is None or abs(stamp(grid) - stamp(agents)) > .5:
            continue
        result = results[str(i)]
        result['samples'] += 1
        result['latest'] = []
        for agent in agents.agents:
            x, y = agent.position.position.x, agent.position.position.y
            ix = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
            iy = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
            radius = math.ceil(.6 / grid.info.resolution)
            values = [grid.data[yy * grid.info.width + xx] for yy in range(iy-radius, iy+radius+1) for xx in range(ix-radius, ix+radius+1) if 0 <= xx < grid.info.width and 0 <= yy < grid.info.height]
            maximum = max(values) if values else None
            result['latest'].append({'id': agent.id, 'xy': [x, y], 'nearby_max': maximum})
            if maximum == 100 and agent.id not in result['marked_ids']:
                result['marked_ids'].append(agent.id)
report = {'robots': results, 'both_observed_marking': all(v['marked_ids'] for v in results.values()), 'scope': 'nearby cell marking; not clearance/navigation acceptance'}
with open(args.output, 'w') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps(report, indent=2))
node.destroy_node()
rclpy.shutdown()
