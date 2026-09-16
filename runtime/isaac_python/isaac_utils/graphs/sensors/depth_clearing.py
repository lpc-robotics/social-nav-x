"""Optional render-geometry clearing source for Isaac 5.1 RTX lidar gaps.

Four co-located narrow depth cameras see render geometry, including animated
people without PhysX colliders. The RTX LaserScan remains the marking source.
"""
import math
import weakref

import carb
import numpy as np
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, UsdGeom
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

from isaac_utils.clearing_geometry import clearing_endpoints, horizontal_directions


class DepthClearing:
    _instances = weakref.WeakSet()
    WIDTH = 321
    HEIGHT = 3

    def __init__(self, prim_path, frame_id, topic, range_min, range_max, rate):
        self.frame_id = frame_id
        self.topic = topic
        self.range_min = range_min
        self.range_max = range_max
        self.period = 1.0 / rate
        self.last_time = None
        self.frames = 0
        self.published_once = False
        self.cameras = []
        self.node = Node("depth_clearing_" + str(id(self)), use_global_arguments=False)
        self.publisher = self.node.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
        stage = omni.usd.get_context().get_stage()
        try:
            for quadrant in range(4):
                yaw = quadrant * math.pi / 2
                camera = UsdGeom.Camera.Define(stage, f"{prim_path}/clearing_{quadrant}")
                camera.CreateFocalLengthAttr(10.0)
                camera.CreateHorizontalApertureAttr(20.0)
                camera.CreateVerticalApertureAttr(20.0 * self.HEIGHT / self.WIDTH)
                camera.CreateClippingRangeAttr(Gf.Vec2f(0.001, range_max))
                direction = Gf.Vec3d(math.cos(yaw), math.sin(yaw), 0)
                pose = Gf.Matrix4d().SetLookAt(Gf.Vec3d(0), direction, Gf.Vec3d(0, 0, 1)).GetInverse()
                camera.AddTransformOp().Set(pose)
                product = rep.create.render_product(camera.GetPath(), [self.WIDTH, self.HEIGHT])
                annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
                annotator.attach([product.path])
                self.cameras.append((product, annotator, horizontal_directions(self.WIDTH, yaw)))
        except Exception:
            self.destroy()
            raise
        self._instances.add(self)
        carb.log_info(
            f"Depth clearing ready: topic={topic} frame={frame_id} "
            f"rate={rate:.3f} range=[{range_min:.3f}, {range_max:.3f}]"
        )

    @classmethod
    def update_all(cls, simulation_time):
        # Called after world.step(render=True), so depth and timestamp refer to
        # the completed frame. Paused renders never generate clearing messages.
        for instance in list(cls._instances):
            instance.update(simulation_time)

    def update(self, simulation_time):
        self.frames += 1
        if self.last_time is not None and simulation_time < self.last_time:
            self.frames = 0
            self.last_time = None
        if self.frames < 6 or (self.last_time is not None and
                              simulation_time - self.last_time < self.period - 1e-6):
            return
        points = []
        for _, annotator, directions in self.cameras:
            depth = np.asarray(annotator.get_data())
            if depth.shape != (self.HEIGHT, self.WIDTH):
                return  # incomplete render is not free space
            points.append(clearing_endpoints(depth[self.HEIGHT // 2], directions,
                                             self.range_min, self.range_max))
        header = Header(frame_id=self.frame_id)
        ns = round(simulation_time * 1e9)
        header.stamp.sec, header.stamp.nanosec = divmod(ns, 10**9)
        cloud = np.concatenate(points)
        self.publisher.publish(create_cloud_xyz32(header, cloud))
        if not self.published_once:
            carb.log_info(
                f"Depth clearing first frame: topic={self.topic} "
                f"points={len(cloud)} stamp={header.stamp.sec}.{header.stamp.nanosec:09d}"
            )
            self.published_once = True
        self.last_time = simulation_time

    def destroy(self):
        self._instances.discard(self)
        for product, annotator, _ in self.cameras:
            try:
                annotator.detach([product.path])
            except Exception as error:
                carb.log_warn(f"Depth clearing annotator teardown: {error}")
        for product, _, _ in self.cameras:
            try:
                product.destroy()
            except Exception as error:
                carb.log_warn(f"Depth clearing render-product teardown: {error}")
        self.cameras.clear()
        self.node.destroy_node()
