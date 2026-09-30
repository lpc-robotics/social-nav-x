"""Optional REP-117 LaserScan reconstructed from independent render-depth views."""

import itertools
import math
import weakref
import zlib

import carb
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
import numpy as np
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, UsdGeom
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from isaac_utils.scan_geometry import (
    camera_sample_map,
    depth_views_to_ranges,
    normalized_scan_layout,
    scan_state_masks,
)


class NormalizedScan:
    """Publish a geometry-based planar scan without interpreting RTX sentinels."""

    _instances = weakref.WeakSet()
    _serials = itertools.count(1)
    WIDTH = 1025
    HEIGHT = 3

    def __init__(
        self,
        prim_path,
        parent_frame_id,
        frame_id,
        topic,
        translation,
        rotation,
        samples,
        angle_min,
        angle_max,
        range_min,
        range_max,
        rate,
        noise_mean=0.0,
        noise_stddev=0.0,
        range_resolution=0.0,
        seed=0,
    ):
        self.parent_frame_id = parent_frame_id
        self.frame_id = frame_id
        self.topic = topic
        self.range_min = float(range_min)
        self.range_max = float(range_max)
        self.period = 1.0 / float(rate)
        self.noise_mean = float(noise_mean)
        self.noise_stddev = float(noise_stddev)
        self.range_resolution = float(range_resolution)
        self.last_time = None
        self.last_diagnostic_time = None
        self.frames = 0
        self.published_once = False
        self.cameras = []
        self._destroyed = False

        self.layout = normalized_scan_layout(samples, angle_min, angle_max)
        self.view_index, self.pixel_index, sampled_angles = camera_sample_map(
            self.layout.angles, self.WIDTH
        )
        angular_error = np.abs(
            (sampled_angles - self.layout.angles + math.pi) % (2.0 * math.pi) - math.pi
        )
        self.max_angular_error = float(angular_error.max(initial=0.0))
        topic_seed = zlib.crc32(topic.encode("utf-8"))
        self.rng = np.random.default_rng((int(seed) + topic_seed) % (2**32))

        node_suffix = str(next(self._serials))
        self.node = Node("normalized_scan_" + node_suffix, use_global_arguments=False)
        self.publisher = self.node.create_publisher(LaserScan, topic, qos_profile_sensor_data)
        self.diagnostic_publisher = self.node.create_publisher(
            DiagnosticArray, topic + "/diagnostics", 10
        )
        self.tf_broadcaster = StaticTransformBroadcaster(self.node)
        self._publish_static_transform(translation, rotation)

        stage = omni.usd.get_context().get_stage()
        try:
            for quadrant in range(4):
                yaw = quadrant * math.pi / 2.0
                camera = UsdGeom.Camera.Define(stage, f"{prim_path}/normalized_scan_{quadrant}")
                camera.CreateFocalLengthAttr(10.0)
                camera.CreateHorizontalApertureAttr(20.0)
                camera.CreateVerticalApertureAttr(20.0 * self.HEIGHT / self.WIDTH)
                camera.CreateClippingRangeAttr(Gf.Vec2f(0.001, self.range_max))
                direction = Gf.Vec3d(math.cos(yaw), math.sin(yaw), 0.0)
                pose = Gf.Matrix4d().SetLookAt(
                    Gf.Vec3d(0.0), direction, Gf.Vec3d(0.0, 0.0, 1.0)
                ).GetInverse()
                camera.AddTransformOp().Set(pose)
                # A robot can be deleted and respawned at the same USD path.  Do
                # not let Replicator hand the new sensor a cached render product
                # whose Hydra texture was destroyed with the previous robot.
                product = rep.create.render_product(
                    camera.GetPath(),
                    [self.WIDTH, self.HEIGHT],
                    force_new=True,
                    name=f"NormalizedScan_{node_suffix}_{quadrant}",
                )
                annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
                annotator.attach([product.path])
                self.cameras.append((product, annotator))
        except Exception:
            self.destroy()
            raise

        self._instances.add(self)
        carb.log_info(
            f"Normalized scan ready: topic={topic} frame={frame_id} "
            f"beams={len(self.layout.angles)} rate={rate:.3f} "
            f"range=[{self.range_min:.3f}, {self.range_max:.3f}] "
            f"max_angular_error={self.max_angular_error:.8f}"
        )

    def _publish_static_transform(self, translation, rotation):
        transform = TransformStamped()
        transform.header.frame_id = self.parent_frame_id
        transform.child_frame_id = self.frame_id
        transform.transform.translation.x = float(translation.x)
        transform.transform.translation.y = float(translation.y)
        transform.transform.translation.z = float(translation.z)
        transform.transform.rotation.w = float(rotation.w)
        transform.transform.rotation.x = float(rotation.x)
        transform.transform.rotation.y = float(rotation.y)
        transform.transform.rotation.z = float(rotation.z)
        self.tf_broadcaster.sendTransform(transform)

    @classmethod
    def update_all(cls, simulation_time):
        for instance in list(cls._instances):
            instance.update(simulation_time)

    @staticmethod
    def _stamp(message, simulation_time):
        nanoseconds = round(simulation_time * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(nanoseconds, 10**9)

    def _publish_diagnostic(self, simulation_time, level, reason, masks=None):
        if (
            self.last_diagnostic_time is not None
            and simulation_time - self.last_diagnostic_time < self.period - 1e-6
        ):
            return
        array = DiagnosticArray()
        self._stamp(array, simulation_time)
        status = DiagnosticStatus()
        status.level = level
        status.name = self.topic
        status.hardware_id = "isaac-render-depth-normalized-scan"
        status.message = reason
        values = {
            "source": "distance_to_camera",
            "frame_id": self.frame_id,
            "beams": str(len(self.layout.angles)),
            "range_min": str(self.range_min),
            "range_max": str(self.range_max),
            "scan_age_s": (
                "inf"
                if self.last_time is None
                else f"{max(0.0, simulation_time - self.last_time):.9f}"
            ),
            "max_angular_error_rad": f"{self.max_angular_error:.9f}",
        }
        if masks is not None:
            values.update({name: str(int(mask.sum())) for name, mask in masks.items()})
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        array.status = [status]
        self.diagnostic_publisher.publish(array)
        self.last_diagnostic_time = simulation_time

    def update(self, simulation_time):
        self.frames += 1
        if self.last_time is not None and simulation_time < self.last_time:
            self.frames = 0
            self.last_time = None
            self.last_diagnostic_time = None
        if self.frames < 6:
            return
        if self.last_time is not None and simulation_time - self.last_time < self.period - 1e-6:
            return

        rows = []
        try:
            for _, annotator in self.cameras:
                depth = np.asarray(annotator.get_data(do_array_copy=True))
                if depth.shape != (self.HEIGHT, self.WIDTH):
                    self._publish_diagnostic(
                        simulation_time,
                        DiagnosticStatus.WARN,
                        f"incomplete render shape {depth.shape}",
                    )
                    return
                rows.append(depth[self.HEIGHT // 2])
        except Exception as error:
            self._publish_diagnostic(
                simulation_time, DiagnosticStatus.ERROR, f"render read failed: {error}"
            )
            return

        try:
            ranges = depth_views_to_ranges(
                np.stack(rows),
                self.view_index,
                self.pixel_index,
                self.range_min,
                self.range_max,
                noise_mean=self.noise_mean,
                noise_stddev=self.noise_stddev,
                range_resolution=self.range_resolution,
                rng=self.rng,
            )
        except Exception as error:
            self._publish_diagnostic(
                simulation_time, DiagnosticStatus.ERROR, f"conversion failed: {error}"
            )
            return

        message = LaserScan()
        message.header.frame_id = self.frame_id
        self._stamp(message, simulation_time)
        message.angle_min = self.layout.angle_min
        message.angle_max = self.layout.angle_max
        message.angle_increment = self.layout.angle_increment
        message.time_increment = 0.0
        message.scan_time = self.period
        message.range_min = self.range_min
        message.range_max = self.range_max
        message.ranges = ranges.tolist()
        message.intensities = []
        self.publisher.publish(message)

        masks = scan_state_masks(ranges, self.range_min, self.range_max)
        self.last_time = simulation_time
        self._publish_diagnostic(simulation_time, DiagnosticStatus.OK, "scan published", masks)
        if not self.published_once:
            carb.log_info(
                f"Normalized scan first frame: topic={self.topic} "
                f"hit={int(masks['hit'].sum())} no_return={int(masks['no_return'].sum())} "
                f"too_close={int(masks['too_close'].sum())} "
                f"unknown={int(masks['unknown'].sum())}"
            )
            self.published_once = True

    def destroy(self):
        if self._destroyed:
            return
        self._destroyed = True
        self._instances.discard(self)
        for _, annotator in reversed(self.cameras):
            try:
                annotator.detach()
            except Exception as error:
                carb.log_warn(f"Normalized scan annotator teardown: {error}")
        for product, _ in reversed(self.cameras):
            try:
                product.destroy()
            except Exception as error:
                carb.log_warn(f"Normalized scan render-product teardown: {error}")
        self.cameras.clear()
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
