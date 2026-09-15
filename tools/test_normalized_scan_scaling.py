"""Measure isolated normalized scan sources in a fresh Isaac process."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT_SOURCE = ROOT / ".workspaces/laserscan-v1/src/arena-isaac/arena_isaac"
sys.path.insert(0, str(EXPERIMENT_SOURCE))
parser = argparse.ArgumentParser()
parser.add_argument("--sources", type=int, choices=(1, 2, 4), required=True)
ARGS = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402


app = SimulationApp(
    {
        "headless": True,
        "active_gpu": 2,
        "physics_gpu": 0,
        "multi_gpu": False,
        "extra_args": [
            f"--/log/file={ROOT}/.workspaces/laserscan-v1/log/normalized_scan_scaling_kit.log"
        ],
    }
)

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import rclpy  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from isaac_utils.graphs.sensors.normalized_scan import NormalizedScan  # noqa: E402
from isaac_utils.utils.geom import Rotation, Translation  # noqa: E402


stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)


def cube(name, position, scale):
    primitive = UsdGeom.Cube.Define(stage, "/World/" + name)
    primitive.CreateSizeAttr(1.0)
    primitive.AddTranslateOp().Set(Gf.Vec3d(*position))
    primitive.AddScaleOp().Set(Gf.Vec3d(*scale))


cube("central_wall", (0.0, 0.0, 0.5), (0.2, 8.0, 1.0))
positions = [(-3.0, -2.0, 0.5), (3.0, -2.0, 0.5), (-3.0, 2.0, 0.5), (3.0, 2.0, 0.5)]


def gpu_memory_mib():
    try:
        output = subprocess.check_output(
            [
                "nvidia-smi",
                "--query-gpu=index,memory.used",
                "--format=csv,noheader,nounits",
            ],
            text=True,
        )
        return {
            int(line.split(",")[0]): int(line.split(",")[1])
            for line in output.strip().splitlines()
        }
    except Exception as error:
        return {"error": str(error)}


rclpy.init()
node = Node("normalized_scan_scaling_test", use_global_arguments=False)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
results = []

try:
    for count in (ARGS.sources,):
        received = [[] for _ in range(count)]
        scans = []
        subscriptions = []
        for index in range(count):
            topic = f"/robot_{index}/lidar_normalized"
            prim_path = f"/World/scaling_{count}/robot_{index}/sensor"
            sensor = UsdGeom.Xform.Define(stage, prim_path)
            sensor.AddTranslateOp().Set(Gf.Vec3d(*positions[index]))
            subscriptions.append(
                node.create_subscription(
                    LaserScan,
                    topic,
                    received[index].append,
                    qos_profile_sensor_data,
                )
            )
            position = positions[index]
            scans.append(
                NormalizedScan(
                    prim_path,
                    f"robot_{index}/lidar_link",
                    f"robot_{index}/lidar_link_normalized",
                    topic,
                    Translation(*position),
                    Rotation.parse([0.0, 0.0, 0.0]),
                    640,
                    -math.pi,
                    math.pi,
                    0.08,
                    12.0,
                    10.0,
                    noise_stddev=0.0,
                    range_resolution=0.0,
                    seed=100,
                )
            )

        for _ in range(30):
            app.update()
            NormalizedScan.update_all(timeline.get_current_time())
            rclpy.spin_once(node, timeout_sec=0.0)

        memory_before = gpu_memory_mib()
        sim_start = timeline.get_current_time()
        wall_start = time.monotonic()
        for _ in range(180):
            app.update()
            NormalizedScan.update_all(timeline.get_current_time())
            rclpy.spin_once(node, timeout_sec=0.0)
        wall_seconds = time.monotonic() - wall_start
        sim_seconds = timeline.get_current_time() - sim_start
        memory_after = gpu_memory_mib()

        per_robot = []
        for index, messages in enumerate(received):
            assert len(messages) >= 20, (count, index, len(messages))
            assert all(
                message.header.frame_id == f"robot_{index}/lidar_link_normalized"
                for message in messages
            )
            assert all(len(message.ranges) == 640 for message in messages)
            stamps = np.asarray(
                [
                    message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                    for message in messages
                ]
            )
            periods = np.diff(stamps)
            per_robot.append(
                {
                    "topic": f"/robot_{index}/lidar_normalized",
                    "frame_id": messages[-1].header.frame_id,
                    "messages": len(messages),
                    "sim_frequency_hz": float(1.0 / np.median(periods)),
                }
            )

        results.append(
            {
                "sources": count,
                "sim_seconds": sim_seconds,
                "wall_seconds": wall_seconds,
                "real_time_factor": sim_seconds / wall_seconds,
                "render_updates_per_wall_second": 180.0 / wall_seconds,
                "gpu_memory_mib_before": memory_before,
                "gpu_memory_mib_after": memory_after,
                "robots": per_robot,
            }
        )

        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        for scan in scans:
            scan.destroy()
        for _ in range(10):
            app.update()

    output = ROOT / f".workspaces/laserscan-v1/log/normalized_scan_scaling_{ARGS.sources}.json"
    payload = {"passed": True, "results": results}
    output.write_text(json.dumps(payload, indent=2))
    print("NORMALIZED_SCAN_SCALING_PASS", json.dumps(payload), flush=True)
except Exception:
    traceback.print_exc()
    print("NORMALIZED_SCAN_SCALING_FAILED", flush=True)
    raise
finally:
    for scan in list(NormalizedScan._instances):
        scan.destroy()
    node.destroy_node()
    rclpy.shutdown()
    app.close()
