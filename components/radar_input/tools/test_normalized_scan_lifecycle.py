"""Validate normalized scan teardown and same-path robot respawn in Isaac."""

import json
from pathlib import Path
import sys
import traceback


ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT_SOURCE = ROOT / ".workspaces/laserscan-v1/src/arena-isaac/arena_isaac"
sys.path.insert(0, str(EXPERIMENT_SOURCE))
KIT_LOG = ROOT / ".workspaces/laserscan-v1/log/normalized_scan_lifecycle_kit.log"
KIT_LOG.unlink(missing_ok=True)

from isaacsim import SimulationApp  # noqa: E402


app = SimulationApp(
    {
        "headless": True,
        "active_gpu": 2,
        "physics_gpu": 0,
        "multi_gpu": False,
        "extra_args": [f"--/log/file={KIT_LOG}"],
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
wall = UsdGeom.Cube.Define(stage, "/World/wall")
wall.CreateSizeAttr(1.0)
wall.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 0.5))
wall.AddScaleOp().Set(Gf.Vec3d(0.2, 5.0, 1.0))


def make_scan(index):
    prim_path = f"/World/robot_{index}/lidar"
    sensor = UsdGeom.Xform.Define(stage, prim_path)
    sensor.AddTranslateOp().Set(Gf.Vec3d(0.0, index * 1.5, 0.5))
    return NormalizedScan(
        prim_path,
        f"robot_{index}/lidar_link",
        f"robot_{index}/lidar_link_normalized",
        f"/robot_{index}/lidar_normalized",
        Translation(0.0, 0.0, 0.5),
        Rotation.parse([0.0, 0.0, 0.0]),
        640,
        -3.14159,
        3.14159,
        0.08,
        12.0,
        10.0,
        noise_stddev=0.0,
        range_resolution=0.0,
        seed=100,
    )


def advance(steps):
    for _ in range(steps):
        app.update()
        NormalizedScan.update_all(timeline.get_current_time())
        rclpy.spin_once(node, timeout_sec=0.0)


def destroy_scan(scan, prim_path):
    render_paths = [product.path for product, _ in scan.cameras]
    scan.destroy()
    stage.RemovePrim(prim_path)
    advance(5)
    assert all(not stage.GetPrimAtPath(path).IsValid() for path in render_paths), render_paths


rclpy.init()
node = Node("normalized_scan_lifecycle_test", use_global_arguments=False)
received = [[], []]
subscriptions = [
    node.create_subscription(
        LaserScan,
        f"/robot_{index}/lidar_normalized",
        received[index].append,
        qos_profile_sensor_data,
    )
    for index in range(2)
]
timeline = omni.timeline.get_timeline_interface()
timeline.play()
scans = []
payload = None

try:
    scans = [make_scan(0), make_scan(1)]
    advance(90)
    assert len(received[0]) >= 10 and len(received[1]) >= 10

    # Respawn one robot while another remains alive.
    robot_1_before = len(received[1])
    destroy_scan(scans[0], "/World/robot_0")
    received[0].clear()
    scans[0] = make_scan(0)
    advance(90)
    assert len(received[0]) >= 10
    assert len(received[1]) > robot_1_before

    # Also cover the single-robot case after all normalized sources went away.
    for index, scan in enumerate(scans):
        destroy_scan(scan, f"/World/robot_{index}")
    scans.clear()
    assert len(list(NormalizedScan._instances)) == 0
    received[0].clear()
    scans = [make_scan(0)]
    advance(90)
    assert len(received[0]) >= 10
    message = received[0][-1]
    assert message.header.frame_id == "robot_0/lidar_link_normalized"
    assert len(message.ranges) == 640
    stamps = np.asarray(
        [item.header.stamp.sec + item.header.stamp.nanosec / 1e9 for item in received[0]]
    )
    frequency = float(1.0 / np.median(np.diff(stamps)))
    assert abs(frequency - 10.0) < 1e-3
    payload = {
        "passed": True,
        "same_path_respawn_messages": len(received[0]),
        "frequency_hz": frequency,
        "active_instances_before_final_cleanup": len(list(NormalizedScan._instances)),
    }

    destroy_scan(scans[0], "/World/robot_0")
    scans.clear()
    payload["active_instances_after_final_cleanup"] = len(
        list(NormalizedScan._instances)
    )
    assert payload["active_instances_after_final_cleanup"] == 0
    log_text = KIT_LOG.read_text(errors="replace") if KIT_LOG.exists() else ""
    payload["teardown_warning_count"] = log_text.count(
        "Normalized scan annotator teardown"
    )
    payload["render_product_warning_count"] = log_text.count(
        "Normalized scan render-product teardown"
    )
    assert payload["teardown_warning_count"] == 0, payload
    assert payload["render_product_warning_count"] == 0, payload
    output = ROOT / ".workspaces/laserscan-v1/log/normalized_scan_lifecycle.json"
    output.write_text(json.dumps(payload, indent=2))
    print("NORMALIZED_SCAN_LIFECYCLE_PASS", json.dumps(payload), flush=True)
except Exception:
    traceback.print_exc()
    print("NORMALIZED_SCAN_LIFECYCLE_FAILED", flush=True)
    raise
finally:
    for scan in list(NormalizedScan._instances):
        scan.destroy()
    node.destroy_node()
    rclpy.shutdown()
    app.close()
