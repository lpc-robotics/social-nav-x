"""Wall-clock command and observation watchdog for one robot."""

from __future__ import annotations

import copy
import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from std_msgs.msg import Bool


class CommandGuard(Node):
    def __init__(self) -> None:
        super().__init__("command_guard")
        self.declare_parameter("robot_name", "")
        self.declare_parameter("control_mode", "nav2")
        # Humble rclpy cannot represent a typed, initialized empty array here:
        # [] becomes BYTE_ARRAY, while a type-only declaration is uninitialized.
        # Use one filtered sentinel so a single-robot launch has a valid default
        # and multi-robot string-array overrides keep the same declared type.
        self.declare_parameter("peer_names", [""])
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("observation_timeout", 0.6)
        # RTX lidar publication can lag /clock in simulated time under multi-sensor
        # load even while messages continue arriving. Keep that bound distinct
        # from the wall-clock loss-of-signal watchdog.
        self.declare_parameter("simulation_stamp_timeout", 1.0)
        self.declare_parameter("startup_grace", 20.0)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("max_linear", 1.0)
        self.declare_parameter("max_angular", 1.2)
        self.declare_parameter("require_pedestrian_health", False)
        self.require_pedestrian_health = bool(self.get_parameter("require_pedestrian_health").value)
        self.robot_name = str(self.get_parameter("robot_name").value).strip("/")
        self.mode = str(self.get_parameter("control_mode").value)
        self.peer_names = [
            name
            for item in self.get_parameter("peer_names").value
            if (name := str(item).strip("/"))
        ]
        if not self.robot_name or self.mode not in ("nav2", "external"):
            raise RuntimeError("robot_name and valid control_mode are required")
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.observation_timeout = float(self.get_parameter("observation_timeout").value)
        self.simulation_stamp_timeout = float(
            self.get_parameter("simulation_stamp_timeout").value
        )
        self.startup_grace = float(self.get_parameter("startup_grace").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        rate = float(self.get_parameter("publish_rate").value)
        if min(
            self.command_timeout, self.observation_timeout,
            self.simulation_stamp_timeout, rate, self.max_linear, self.max_angular,
        ) <= 0.0:
            raise RuntimeError("timeouts and publish_rate must be positive")

        prefix = f"/{self.robot_name}"
        self.publisher = self.create_publisher(Twist, prefix + "/cmd_vel", 10)
        self.diagnostic = self.create_publisher(
            DiagnosticArray, "/multirobot/diagnostics", 10
        )
        self.create_subscription(Twist, prefix + "/cmd_vel_nav", self._nav, 10)
        self.create_subscription(Twist, prefix + "/cmd_vel_external", self._external, 10)
        self.create_subscription(
            LaserScan, prefix + "/lidar_normalized", self._lidar, qos_profile_sensor_data
        )
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.peer_seen = {peer: 0.0 for peer in self.peer_names}
        self.peer_stamps_ns = {peer: None for peer in self.peer_names}
        self.peer_subscriptions = [
            self.create_subscription(
                Odometry,
                f"/{peer}/odom",
                lambda message, name=peer: self._peer(name, message),
                qos_profile_sensor_data,
            )
            for peer in self.peer_names
        ]
        self.create_service(SetBool, prefix + "/emergency_stop", self._emergency)

        self._lock = threading.RLock()
        self._pedestrian_healthy = False
        self._pedestrian_health_seen = 0.0
        if self.require_pedestrian_health:
            self.create_subscription(Bool, "/multirobot/hunav/health", self._pedestrian_health, 10)
        self._command = Twist()
        self._command_valid = True
        self._command_seen = 0.0
        self._lidar_seen = 0.0
        self._lidar_stamp_ns = None
        self._clock_ns = None
        self._emergency_latched = False
        self._clock_rewind_latched = False
        self._started = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, args=(rate,), daemon=True)
        self._thread.start()

    def _pedestrian_health(self, message: Bool) -> None:
        with self._lock:
            self._pedestrian_healthy = message.data
            self._pedestrian_health_seen = time.monotonic()

    def _accept(self, source: str, message: Twist) -> None:
        if source != self.mode:
            return
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        with self._lock:
            self._command_valid = all(math.isfinite(value) for value in values)
            command = Twist()
            if self._command_valid:
                command.linear.x = max(-self.max_linear, min(self.max_linear, message.linear.x))
                command.angular.z = max(-self.max_angular, min(self.max_angular, message.angular.z))
            self._command = command
            self._command_seen = time.monotonic()

    def _nav(self, message: Twist) -> None:
        self._accept("nav2", message)

    def _external(self, message: Twist) -> None:
        self._accept("external", message)

    @staticmethod
    def _stamp_ns(message) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _lidar(self, message: LaserScan) -> None:
        with self._lock:
            self._lidar_seen = time.monotonic()
            self._lidar_stamp_ns = self._stamp_ns(message)

    def _peer(self, name: str, message: Odometry) -> None:
        with self._lock:
            self.peer_seen[name] = time.monotonic()
            self.peer_stamps_ns[name] = self._stamp_ns(message)

    def _on_clock(self, message: Clock) -> None:
        stamp = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        with self._lock:
            if self._clock_ns is not None and stamp < self._clock_ns:
                self._clock_rewind_latched = True
            self._clock_ns = stamp

    def _emergency(self, request: SetBool.Request, response: SetBool.Response):
        with self._lock:
            if not request.data and self._clock_rewind_latched:
                response.success = False
                response.message = "clock rewind is latched; restart the experiment"
                return response
            self._emergency_latched = bool(request.data)
            if not request.data:
                self._command = Twist()
                self._command_seen = 0.0
        response.success = True
        response.message = "emergency stop enabled" if request.data else "cleared; new command required"
        return response

    def _decision(self, now: float) -> tuple[Twist, str]:
        with self._lock:
            if self._clock_rewind_latched:
                return Twist(), "clock_rewind"
            if self._emergency_latched:
                return Twist(), "emergency_stop"
            if self.require_pedestrian_health and (
                not self._pedestrian_healthy or now - self._pedestrian_health_seen > 0.5
            ):
                return Twist(), "pedestrian_unhealthy"
            if now - self._started <= self.startup_grace:
                observations_ready = self._lidar_seen > 0 and all(self.peer_seen.values())
            else:
                observations_ready = True
            if not observations_ready:
                return Twist(), "startup_wait"
            if self._lidar_seen <= 0 or now - self._lidar_seen > self.observation_timeout:
                return Twist(), "lidar_stale"
            if self._clock_ns is None or self._lidar_stamp_ns is None:
                return Twist(), "simulation_time_unavailable"
            lidar_sim_age = (self._clock_ns - self._lidar_stamp_ns) / 1e9
            if lidar_sim_age < -0.050001 or lidar_sim_age > self.simulation_stamp_timeout:
                return Twist(), "lidar_stamp_stale"
            stale_peers = [
                peer for peer, seen in self.peer_seen.items()
                if seen <= 0 or now - seen > self.observation_timeout
            ]
            if stale_peers:
                return Twist(), "peer_stale:" + ",".join(stale_peers)
            stale_peer_stamps = [
                peer for peer, stamp in self.peer_stamps_ns.items()
                if stamp is None
                or (self._clock_ns - stamp) / 1e9 < -0.050001
                or (self._clock_ns - stamp) / 1e9 > self.simulation_stamp_timeout
            ]
            if stale_peer_stamps:
                return Twist(), "peer_stamp_stale:" + ",".join(stale_peer_stamps)
            if self._command_seen <= 0 or now - self._command_seen > self.command_timeout:
                return Twist(), "command_stale"
            if not self._command_valid:
                return Twist(), "invalid_command"
            return copy.deepcopy(self._command), "ok"

    def _publish_diagnostic(self, now: float, reason: str) -> None:
        array = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = f"multirobot/{self.robot_name}/command_guard"
        status.hardware_id = "arena-isaac"
        status.level = DiagnosticStatus.OK if reason == "ok" else DiagnosticStatus.WARN
        status.message = reason
        with self._lock:
            lidar_wall_age = (
                math.inf if self._lidar_seen <= 0 else now - self._lidar_seen
            )
            lidar_sim_age = (
                math.inf
                if self._clock_ns is None or self._lidar_stamp_ns is None
                else (self._clock_ns - self._lidar_stamp_ns) / 1e9
            )
            peer_wall_ages = [
                math.inf if seen <= 0 else now - seen
                for seen in self.peer_seen.values()
            ]
            peer_sim_ages = [
                math.inf if self._clock_ns is None or stamp is None
                else (self._clock_ns - stamp) / 1e9
                for stamp in self.peer_stamps_ns.values()
            ]
        status.values = [
            KeyValue(key="mode", value=self.mode),
            KeyValue(key="wall_time", value=f"{now:.6f}"),
            KeyValue(key="lidar_wall_age_s", value=f"{lidar_wall_age:.6f}"),
            KeyValue(key="lidar_sim_age_s", value=f"{lidar_sim_age:.6f}"),
            KeyValue(
                key="maximum_peer_wall_age_s",
                value=f"{max(peer_wall_ages, default=0.0):.6f}",
            ),
            KeyValue(
                key="maximum_peer_sim_age_s",
                value=f"{max(peer_sim_ages, default=0.0):.6f}",
            ),
        ]
        array.status = [status]
        self.diagnostic.publish(array)

    def _loop(self, rate: float) -> None:
        period = 1.0 / rate
        next_tick = time.monotonic()
        diagnostic_tick = next_tick
        while not self._stop.is_set():
            now = time.monotonic()
            command, reason = self._decision(now)
            self.publisher.publish(command)
            if now >= diagnostic_tick:
                self._publish_diagnostic(now, reason)
                diagnostic_tick = now + 1.0
            next_tick += period
            self._stop.wait(max(0.0, next_tick - time.monotonic()))

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
