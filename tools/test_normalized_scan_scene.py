"""Controlled Isaac render test for normalized LaserScan geometry and semantics."""

import json
import math
from pathlib import Path
import sys
import traceback


ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT_SOURCE = ROOT / ".workspaces/laserscan-v1/src/arena-isaac/arena_isaac"
sys.path.insert(0, str(EXPERIMENT_SOURCE))

from isaacsim import SimulationApp  # noqa: E402


app = SimulationApp(
    {
        "headless": True,
        "active_gpu": 2,
        "physics_gpu": 0,
        "multi_gpu": False,
        "extra_args": [
            f"--/log/file={ROOT}/.workspaces/laserscan-v1/log/normalized_scan_scene_kit.log"
        ],
    }
)

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import rclpy  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

from isaac_utils.graphs.sensors.normalized_scan import NormalizedScan  # noqa: E402
from isaac_utils.utils.geom import Rotation, Translation  # noqa: E402


stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
sensor = UsdGeom.Xform.Define(stage, "/World/sensor")
sensor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.5))


def cube(name, position, scale):
    primitive = UsdGeom.Cube.Define(stage, "/World/" + name)
    primitive.CreateSizeAttr(1.0)
    translate = primitive.AddTranslateOp()
    translate.Set(Gf.Vec3d(*position))
    primitive.AddScaleOp().Set(Gf.Vec3d(*scale))
    return translate


cube("front_wall", (3.0, 0.0, 0.5), (0.2, 2.0, 1.0))
moving = cube("moving_person", (0.0, 2.0, 0.5), (0.4, 0.4, 1.0))
cube("too_close", (-0.04, 0.0, 0.5), (0.02, 0.04, 0.1))

rclpy.init()
node = Node("normalized_scan_scene_test", use_global_arguments=False)
scans = []
static_transforms = []
node.create_subscription(
    LaserScan, "/lidar_normalized_test", scans.append, qos_profile_sensor_data
)
tf_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
node.create_subscription(TFMessage, "/tf_static", static_transforms.append, tf_qos)

normalized = NormalizedScan(
    "/World/sensor",
    "odom",
    "lidar_normalized",
    "/lidar_normalized_test",
    Translation(0.0, 0.0, 0.142),
    Rotation.parse([0.0, 0.0, 0.0]),
    640,
    -math.pi,
    math.pi,
    0.08,
    12.0,
    10.0,
    noise_stddev=0.0,
    range_resolution=0.0,
    seed=7,
)


def nearest(scan, angle):
    index = int(round((angle - scan.angle_min) / scan.angle_increment)) % len(scan.ranges)
    return float(scan.ranges[index])


try:
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    moved_at = None
    for tick in range(240):
        if len(scans) >= 5 and moved_at is None:
            moving.Set(Gf.Vec3d(0.0, 20.0, 0.5))
            moved_at = len(scans)
        app.update()
        normalized.update(timeline.get_current_time())
        rclpy.spin_once(node, timeout_sec=0.0)
        if moved_at is not None and len(scans) >= moved_at + 5:
            break

    assert moved_at is not None and len(scans) >= moved_at + 5
    before = scans[moved_at - 1]
    after = scans[-1]
    ranges = np.asarray(after.ranges, dtype=np.float32)
    assert len(ranges) == 640
    assert after.header.frame_id == "lidar_normalized"
    assert math.isclose(
        after.angle_max,
        after.angle_min + 639 * after.angle_increment,
        abs_tol=1e-6,
    )
    assert after.angle_max < math.pi
    assert after.time_increment == 0.0
    assert math.isclose(after.scan_time, 0.1, abs_tol=1e-6)
    assert not np.any(np.isfinite(ranges) & ((ranges < 0.08) | (ranges > 12.0)))
    assert not np.any(np.isfinite(ranges) & ((ranges == -1.0) | (ranges == 0.0)))
    assert math.isclose(nearest(before, 0.0), 2.9, abs_tol=0.03)
    assert math.isclose(nearest(after, 0.0), 2.9, abs_tol=0.03)
    assert math.isclose(nearest(before, math.pi / 2.0), 1.8, abs_tol=0.03)
    assert math.isinf(nearest(after, math.pi / 2.0)) and nearest(after, math.pi / 2.0) > 0
    assert math.isinf(nearest(after, -math.pi)) and nearest(after, -math.pi) < 0

    transforms = [item for message in static_transforms for item in message.transforms]
    transform = next(item for item in transforms if item.child_frame_id == "lidar_normalized")
    assert transform.header.frame_id == "odom"
    assert math.isclose(transform.transform.translation.z, 0.142, abs_tol=1e-9)

    stamps = [scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9 for scan in scans]
    periods = np.diff(stamps)
    result = {
        "passed": True,
        "messages": len(scans),
        "sim_frequency_hz": float(1.0 / np.median(periods)),
        "before_front_m": nearest(before, 0.0),
        "after_front_m": nearest(after, 0.0),
        "before_person_m": nearest(before, math.pi / 2.0),
        "after_person": "+Inf",
        "too_close": "-Inf",
        "unknown_count": int(np.isnan(ranges).sum()),
        "hit_count": int(np.isfinite(ranges).sum()),
        "no_return_count": int(np.isposinf(ranges).sum()),
        "too_close_count": int(np.isneginf(ranges).sum()),
        "frame_id": after.header.frame_id,
        "parent_frame_id": transform.header.frame_id,
        "sensor_offset_z": transform.transform.translation.z,
    }
    output = ROOT / ".workspaces/laserscan-v1/log/normalized_scan_scene.json"
    output.write_text(json.dumps(result, indent=2))
    print("NORMALIZED_SCAN_SCENE_PASS", json.dumps(result), flush=True)
except Exception:
    traceback.print_exc()
    print("NORMALIZED_SCAN_SCENE_FAILED", flush=True)
    raise
finally:
    normalized.destroy()
    node.destroy_node()
    rclpy.shutdown()
    app.close()
