"""RTX + two real Humble VoxelLayer masters, fixed sensor and moving occluder."""
import json
import math
import os
import signal
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "active_gpu": 2, "physics_gpu": 0,
                     "multi_gpu": False, "extra_args": [
                         f"--/log/file={ROOT}/logs/depth_test_kit.log"]})
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.sensors.rtx")
import omni.kit.commands
import omni.replicator.core as rep
import omni.timeline
import omni.usd
import numpy as np
from pxr import Gf, UsdGeom
import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import GetCostmap
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml
sys.path.insert(0, str(ROOT / "src/arena-isaac/arena_isaac"))
from isaac_utils.graphs.sensors.depth_clearing import DepthClearing

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
sensor = UsdGeom.Xform.Define(stage, "/World/sensor")
sensor.AddTranslateOp().Set(Gf.Vec3d(0, 0, .2))

def cube(name, pos, scale):
    cube = UsdGeom.Cube.Define(stage, "/World/" + name)
    cube.CreateSizeAttr(1.0)
    translate = cube.AddTranslateOp()
    translate.Set(Gf.Vec3d(*pos))
    cube.AddScaleOp().Set(Gf.Vec3d(*scale))
    return translate

cube("wall", (2.4, 0, .5), (.2, 2, 1))
person = cube("moving_occluder", (0, 2, .8), (.4, .4, 1.6))
# A close occluder must NOT turn into a free ray through a hidden obstacle.
cube("near", (-.04, 0, .2), (.02, .03, .1))
rclpy.init()
node = Node("clearing_scene_test", use_global_arguments=False)
scan_pub = node.create_publisher(LaserScan, "/lidar", qos_profile_sensor_data)
clock_pub = node.create_publisher(Clock, "/clock", 10)
tf = StaticTransformBroadcaster(node)
transform = TransformStamped()
transform.header.frame_id = "odom"
transform.child_frame_id = "lidar_link"
transform.transform.translation.z = .2
transform.transform.rotation.w = 1.0
tf.sendTransform(transform)
clearing = DepthClearing("/World/sensor", "lidar_link", "/lidar_clearing", .08, 12, 10)

params = {
    "use_sim_time": True, "global_frame": "odom", "robot_base_frame": "lidar_link",
    "rolling_window": False, "width": 8, "height": 8, "origin_x": -4.0,
    "origin_y": -4.0, "resolution": .1, "update_frequency": 10.0,
    "publish_frequency": 10.0, "robot_radius": .1, "always_send_full_costmap": True,
    "plugins": ["voxel_layer", "inflation_layer"],
    "voxel_layer": {"plugin": "nav2_costmap_2d::VoxelLayer", "enabled": True,
        "origin_z": 0.0, "z_resolution": .05, "z_voxels": 16,
        "mark_threshold": 0, "observation_sources": "lidar",
        "lidar": {"topic": "/lidar", "data_type": "LaserScan", "marking": True,
            "max_obstacle_height": 2.0,
            "clearing": True, "obstacle_max_range": 2.5, "raytrace_max_range": 3.0,
            "observation_persistence": 0.0, "inf_is_valid": False}},
    "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer",
                        "inflation_radius": .55, "cost_scaling_factor": 3.0},
}
processes = []
clients = {}
logs = []

def wait_future(future, timeout=20):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)
    assert future.done(), "ROS service timeout"
    return future.result()

try:
    for name in ["baseline", "fixed"]:
        p = json.loads(json.dumps(params))
        if name == "fixed":
            p["voxel_layer"]["observation_sources"] += " depth_clearing"
            p["voxel_layer"]["depth_clearing"] = {
                "topic": "/lidar_clearing", "data_type": "PointCloud2",
                "marking": False, "clearing": True, "raytrace_max_range": 3.0,
                "observation_persistence": 0.0, "max_obstacle_height": 2.0}
        config = ROOT / f"logs/{name}_costmap.yaml"
        config.write_text(yaml.safe_dump({"/**": {"ros__parameters": p}}))
        log = (ROOT / f"logs/{name}_costmap.log").open("w")
        logs.append(log)
        command = [str(ROOT / "build/costmap_harness/costmap_harness"),
                   "--ros-args", "-r", f"__ns:=/{name}", "--params-file", str(config)]
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   cwd=ROOT, start_new_session=True)
        processes.append(process)
        client = node.create_client(ChangeState, f"/{name}/costmap/change_state")
        assert client.wait_for_service(timeout_sec=20), f"Missing lifecycle {name}"
        clock_pub.publish(Clock())
        for transition in [1, 3]:
            request = ChangeState.Request()
            request.transition.id = transition
            assert wait_future(client.call_async(request)).success
        clients[name] = node.create_client(GetCostmap, f"/{name}/get_costmap")
        assert clients[name].wait_for_service(timeout_sec=10)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    results = []
    for tick in range(660):
        if tick == 300:
            person.Set(Gf.Vec3d(0, 6, .8))
        app.update()
        now = timeline.get_current_time()
        ns = round(now * 1e9)
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(ns, 10**9)
        clock_pub.publish(clock)
        # Identical marking observations for both masters. The missing returns
        # reproduce the proven RTX FlatScan sentinel; geometry supplies clearing.
        if tick % 6 == 0:
            scan = LaserScan()
            scan.header.frame_id = "lidar_link"
            scan.header.stamp = clock.clock
            scan.angle_min = -math.pi
            scan.angle_increment = 2 * math.pi / 640
            scan.angle_max = scan.angle_min + 639 * scan.angle_increment
            scan.range_min, scan.range_max, scan.scan_time = .08, 12.0, .1
            angles = scan.angle_min + np.arange(640) * scan.angle_increment
            ranges = np.full(640, -1.0)
            wall = (np.cos(angles) > 0) & (np.abs(np.tan(angles)) < 1 / 2.3)
            if tick < 480:
                ranges[wall] = 2.3 / np.cos(angles[wall])
            # Seed a remembered obstacle behind the close occluder. It must
            # remain occupied because the new depth source cannot see through it.
            if tick < 30:
                hidden = np.abs(np.abs(angles) - math.pi) < .05
                ranges[hidden] = 1.0
            if tick < 300:
                human = np.abs(angles - math.pi / 2) < math.atan(.2 / 1.8)
                ranges[human] = 1.8 / np.sin(angles[human])
            scan.ranges = ranges.astype(np.float32).tolist()
            scan_pub.publish(scan)
        clearing.update(now)
        rclpy.spin_once(node, timeout_sec=0)
        # Allow the external costmap update threads to run even at high RTF.
        time.sleep(.004)
        if tick in [270, 330, 390, 510, 650]:
            print("SCAN_SUBSCRIBERS", scan_pub.get_subscription_count(), flush=True)
            record = {"tick": tick, "sim_time": now, "masters": {}}
            for name, client in clients.items():
                msg = wait_future(client.call_async(GetCostmap.Request())).map
                grid = np.array(msg.data).reshape(msg.metadata.size_y, msg.metadata.size_x)
                origin = msg.metadata.origin.position
                yy, xx = np.indices(grid.shape)
                x = origin.x + (xx + .5) * msg.metadata.resolution
                y = origin.y + (yy + .5) * msg.metadata.resolution
                old = (abs(x) < .3) & (abs(y - 1.8) < .3)
                inflation = (abs(x) < .8) & (abs(y - 1.8) < .8)
                wall = (abs(x - 2.3) < .2) & (abs(y) < .8)
                hidden = (abs(x + 1) < .2) & (abs(y) < .2)
                record["masters"][name] = {
                    "old_lethal_cells": int(np.sum(grid[old] == 254)),
                    "old_max_cost": int(grid[old].max()),
                    "inflated_cells": int(np.sum((grid[inflation] > 0) & (grid[inflation] < 254))),
                    "wall_lethal_cells": int(np.sum(grid[wall] == 254)),
                    "occluded_lethal_cells": int(np.sum(grid[hidden] == 254)),
                }
                np.save(ROOT / f"audit/{name}_master_{tick}.npy", grid)
            results.append(record)
            print("MASTER_RESULT", json.dumps(record), flush=True)
    (ROOT / "audit/depth_clearing_comparison.json").write_text(json.dumps(results, indent=2))
    before, after = results[0]["masters"], results[-1]["masters"]
    assert before["fixed"]["old_lethal_cells"] > 0
    assert after["baseline"]["old_lethal_cells"] > 0
    assert after["fixed"]["old_max_cost"] == 0
    assert after["fixed"]["inflated_cells"] == 0
    assert after["fixed"]["wall_lethal_cells"] >= before["fixed"]["wall_lethal_cells"]
    assert after["fixed"]["occluded_lethal_cells"] > 0
    assert after["fixed"]["occluded_lethal_cells"] == before["fixed"]["occluded_lethal_cells"]
    print("DEPTH_CLEARING_TEST_PASS", flush=True)
except Exception:
    traceback.print_exc()
    print("DEPTH_CLEARING_TEST_FAILED", flush=True)
    raise
finally:
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
    for log in logs:
        log.close()
    app.close()
