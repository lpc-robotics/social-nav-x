"""Compare real render LaserScan clearing with the frozen PointCloud2 path."""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / ".workspaces/laserscan-v1/log"
LOG.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(ROOT / ".workspaces/laserscan-v1/src/arena-isaac/arena_isaac"))

from isaacsim import SimulationApp  # noqa: E402


app = SimulationApp(
    {
        "headless": True,
        "active_gpu": int(os.environ.get("GPU_ID", "0")),
        "physics_gpu": 0,
        "multi_gpu": False,
        "extra_args": [f"--/log/file={LOG}/normalized_nav2_costmap_kit.log"],
    }
)

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from lifecycle_msgs.srv import ChangeState  # noqa: E402
from nav2_msgs.srv import GetCostmap  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster  # noqa: E402
import yaml  # noqa: E402

from isaac_utils.graphs.sensors.depth_clearing import DepthClearing  # noqa: E402
from isaac_utils.graphs.sensors.normalized_scan import NormalizedScan  # noqa: E402
from isaac_utils.utils.geom import Rotation, Translation  # noqa: E402


stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
sensor = UsdGeom.Xform.Define(stage, "/World/sensor")
sensor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.2))


def cube(name, position, scale):
    primitive = UsdGeom.Cube.Define(stage, "/World/" + name)
    primitive.CreateSizeAttr(1.0)
    translate = primitive.AddTranslateOp()
    translate.Set(Gf.Vec3d(*position))
    primitive.AddScaleOp().Set(Gf.Vec3d(*scale))
    return translate


cube("wall", (2.4, 0.0, 0.5), (0.2, 2.0, 1.0))
person = cube("moving_person", (0.0, 2.0, 0.8), (0.4, 0.4, 1.6))
cube("hidden_wall", (-1.0, 0.0, 0.5), (0.2, 0.2, 1.0))
near = cube("near_occluder", (-0.04, 6.0, 0.2), (0.02, 0.03, 0.1))

rclpy.init()
node = Node("normalized_nav2_costmap_test", use_global_arguments=False)
scan_pub = node.create_publisher(LaserScan, "/lidar", qos_profile_sensor_data)
clock_pub = node.create_publisher(Clock, "/clock", 10)
scan_states = {"too_close": 0}


def count_too_close(message):
    scan_states["too_close"] = int(np.isneginf(message.ranges).sum())


node.create_subscription(
    LaserScan, "/lidar_normalized", count_too_close, qos_profile_sensor_data
)
tf = StaticTransformBroadcaster(node)
transform = TransformStamped()
transform.header.frame_id = "odom"
transform.child_frame_id = "lidar_link"
transform.transform.translation.z = 0.2
transform.transform.rotation.w = 1.0
tf.sendTransform(transform)

clearing = DepthClearing("/World/sensor", "lidar_link", "/lidar_clearing", 0.08, 12.0, 10.0)
normalized = NormalizedScan(
    "/World/sensor",
    "odom",
    "lidar_link_normalized",
    "/lidar_normalized",
    Translation(0.0, 0.0, 0.2),
    Rotation.parse([0.0, 0.0, 0.0]),
    640,
    -math.pi,
    math.pi,
    0.08,
    12.0,
    10.0,
    noise_stddev=0.0,
    range_resolution=0.0,
    seed=0,
)


def params(kind):
    base = {
        "use_sim_time": True,
        "global_frame": "odom",
        "robot_base_frame": "lidar_link",
        "rolling_window": False,
        "width": 8,
        "height": 8,
        "origin_x": -4.0,
        "origin_y": -4.0,
        "resolution": 0.1,
        "update_frequency": 10.0,
        "publish_frequency": 10.0,
        "robot_radius": 0.1,
        "always_send_full_costmap": True,
        "plugins": ["voxel_layer", "inflation_layer"],
        "voxel_layer": {
            "plugin": "nav2_costmap_2d::VoxelLayer",
            "enabled": True,
            "origin_z": 0.0,
            "z_resolution": 0.05,
            "z_voxels": 16,
            "mark_threshold": 0,
            "observation_sources": "lidar depth_clearing" if kind == "stable" else "normalized",
        },
        "inflation_layer": {
            "plugin": "nav2_costmap_2d::InflationLayer",
            "inflation_radius": 0.55,
            "cost_scaling_factor": 3.0,
        },
    }
    if kind == "stable":
        base["voxel_layer"].update(
            {
                "lidar": {
                    "topic": "/lidar",
                    "data_type": "LaserScan",
                    "marking": True,
                    "clearing": True,
                    "max_obstacle_height": 2.0,
                    "obstacle_max_range": 2.5,
                    "raytrace_max_range": 3.0,
                    "observation_persistence": 0.0,
                    "inf_is_valid": False,
                },
                "depth_clearing": {
                    "topic": "/lidar_clearing",
                    "data_type": "PointCloud2",
                    "marking": False,
                    "clearing": True,
                    "max_obstacle_height": 2.0,
                    "raytrace_max_range": 3.0,
                    "observation_persistence": 0.0,
                },
            }
        )
    else:
        base["voxel_layer"]["normalized"] = {
            "topic": "/lidar_normalized",
            "data_type": "LaserScan",
            "marking": True,
            "clearing": True,
            "max_obstacle_height": 2.0,
            "obstacle_max_range": 2.5,
            "raytrace_max_range": 3.0,
            "observation_persistence": 0.0,
            "inf_is_valid": kind == "normalized_valid_inf",
        }
    return base


processes = []
logs = []
clients = {}


def wait_future(future, timeout=20.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    assert future.done(), "ROS service timeout"
    return future.result()


def sample(client):
    message = wait_future(client.call_async(GetCostmap.Request())).map
    grid = np.asarray(message.data).reshape(message.metadata.size_y, message.metadata.size_x)
    origin = message.metadata.origin.position
    yy, xx = np.indices(grid.shape)
    x = origin.x + (xx + 0.5) * message.metadata.resolution
    y = origin.y + (yy + 0.5) * message.metadata.resolution
    former_person = (np.abs(x) < 0.3) & (np.abs(y - 1.8) < 0.3)
    wall = (np.abs(x - 2.3) < 0.2) & (np.abs(y) < 0.8)
    hidden = (np.abs(x + 1.0) < 0.2) & (np.abs(y) < 0.2)
    return {
        "former_person_lethal": int(np.sum(grid[former_person] == 254)),
        "former_person_max": int(grid[former_person].max()),
        "wall_lethal": int(np.sum(grid[wall] == 254)),
        "occluded_lethal": int(np.sum(grid[hidden] == 254)),
    }


try:
    variants = ["stable", "normalized_invalid_inf", "normalized_valid_inf"]
    for name in variants:
        config = LOG / f"{name}_replacement_costmap.yaml"
        config.write_text(yaml.safe_dump({"/**": {"ros__parameters": params(name)}}))
        stream = (LOG / f"{name}_replacement_costmap.log").open("w")
        logs.append(stream)
        command = [
            str(ROOT / "build/costmap_harness/costmap_harness"),
            "--ros-args",
            "-r",
            f"__ns:=/{name}",
            "--params-file",
            str(config),
        ]
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   cwd=ROOT, start_new_session=True)
        processes.append(process)
        lifecycle = node.create_client(ChangeState, f"/{name}/costmap/change_state")
        assert lifecycle.wait_for_service(timeout_sec=20), name
        clock_pub.publish(Clock())
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            assert wait_future(lifecycle.call_async(request)).success
        clients[name] = node.create_client(GetCostmap, f"/{name}/get_costmap")
        assert clients[name].wait_for_service(timeout_sec=10)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    results = []
    for tick in range(660):
        if tick == 300:
            person.Set(Gf.Vec3d(0.0, 6.0, 0.8))
            near.Set(Gf.Vec3d(-0.04, 0.0, 0.2))
        app.update()
        now = timeline.get_current_time()
        nanoseconds = round(now * 1e9)
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(nanoseconds, 10**9)
        clock_pub.publish(clock)
        if tick % 6 == 0:
            scan = LaserScan()
            scan.header.frame_id = "lidar_link"
            scan.header.stamp = clock.clock
            scan.angle_min = -math.pi
            scan.angle_increment = 2.0 * math.pi / 640
            scan.angle_max = scan.angle_min + 639 * scan.angle_increment
            scan.range_min, scan.range_max, scan.scan_time = 0.08, 12.0, 0.1
            angles = scan.angle_min + np.arange(640) * scan.angle_increment
            ranges = np.full(640, -1.0)
            wall = (np.cos(angles) > 0) & (np.abs(np.tan(angles)) < 1.0 / 2.3)
            ranges[wall] = 2.3 / np.cos(angles[wall])
            if tick < 300:
                hidden = np.abs(np.abs(angles) - math.pi) < 0.08
                ranges[hidden] = 1.0
            if tick < 300:
                human = np.abs(angles - math.pi / 2.0) < math.atan(0.2 / 1.8)
                ranges[human] = 1.8 / np.sin(angles[human])
            scan.ranges = ranges.astype(np.float32).tolist()
            scan_pub.publish(scan)
        clearing.update(now)
        normalized.update(now)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.004)
        if tick in (270, 650):
            record = {"tick": tick, "sim_time": now, "variants": {}}
            for name in variants:
                record["variants"][name] = sample(clients[name])
            record["normalized_too_close_beams"] = scan_states["too_close"]
            results.append(record)
            print("COSTMAP_RESULT", json.dumps(record), flush=True)

    before, after = (entry["variants"] for entry in results)
    assert before["stable"]["former_person_lethal"] > 0
    assert before["normalized_valid_inf"]["former_person_lethal"] > 0
    assert before["normalized_invalid_inf"]["former_person_lethal"] > 0
    assert after["stable"]["former_person_max"] == 0
    assert after["normalized_valid_inf"]["former_person_max"] == 0
    assert after["normalized_invalid_inf"]["former_person_lethal"] > 0
    assert after["stable"]["wall_lethal"] > 0
    assert after["normalized_valid_inf"]["wall_lethal"] > 0
    assert before["stable"]["occluded_lethal"] > 0
    assert before["normalized_valid_inf"]["occluded_lethal"] > 0
    assert after["stable"]["occluded_lethal"] > 0
    assert after["normalized_valid_inf"]["occluded_lethal"] > 0
    assert results[-1]["normalized_too_close_beams"] > 0
    output = LOG / "normalized_nav2_costmap_comparison.json"
    output.write_text(json.dumps({"passed": True, "results": results}, indent=2))
    print("NORMALIZED_NAV2_COSTMAP_PASS", flush=True)
except Exception:
    traceback.print_exc()
    print("NORMALIZED_NAV2_COSTMAP_FAILED", flush=True)
    raise
finally:
    normalized.destroy()
    clearing.destroy()
    node.destroy_node()
    rclpy.shutdown()
    for process in processes:
        os.killpg(process.pid, signal.SIGTERM)
    for process in processes:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
    for stream in logs:
        stream.close()
    app.close()
