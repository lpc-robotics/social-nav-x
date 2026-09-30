"""One authoritative pedestrian world with a stateless multi-robot SFM backend."""
from __future__ import annotations

import copy
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String
from rosgraph_msgs.msg import Clock as ClockMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from hunav_msgs.msg import Agent, Agents
from arena_people_msgs.msg import Pedestrian, SpawnPedestrian
from arena_people_msgs.srv import SpawnPedestrians, UpdatePedestrians
from arena_multi_hunav_msgs.msg import BehaviorModifiers, InteractionForces
from arena_multi_hunav_msgs.srv import ComputeMultiAgents
import yaml

from .psychology import Entity, MentalState, MODELS, make_context, prepare_step
from .timeline import Timeline, Sample


def load_people(path):
    document = yaml.safe_load(Path(path).read_text())
    spec = document["hunav_loader"]["ros__parameters"]
    models = document["arena_multi_hunav"]["character_models"]
    if len(models) != len(spec["agents"]):
        raise ValueError("one character model required per pedestrian")
    result = Agents(); result.header.frame_id = "map"
    ids, names = set(), set()
    for name in spec["agents"]:
        item = spec[name]
        a = Agent(); a.id = int(item["id"]); a.name = name; a.type = Agent.PERSON
        if a.id <= 0 or a.id in ids or name in names:
            raise ValueError("invalid pedestrian identity")
        ids.add(a.id); names.add(name)
        a.group_id = -1; a.skin = int(item.get("skin", 0)); a.radius = float(item["radius"])
        a.desired_velocity = float(item["max_vel"]); a.goal_radius = float(item["goal_radius"])
        a.cyclic_goals = bool(item["cyclic_goals"])
        if int(item["behavior"]["type"]) != 1 or int(item.get("group_id", -1)) != -1:
            raise ValueError("multi_sfm v1 supports only ungrouped regular people")
        a.behavior.type = 1
        for key in ("goal_force_factor", "obstacle_force_factor", "social_force_factor"):
            setattr(a.behavior, key, float(item["behavior"][key]))
        p = item["init_pose"]
        a.position.position.x = float(p["x"]); a.position.position.y = float(p["y"])
        a.yaw = float(p["h"])
        a.position.orientation.z = math.sin(a.yaw / 2); a.position.orientation.w = math.cos(a.yaw / 2)
        for key in item["goals"]:
            goal = Pose(); goal.position.x = float(item[key]["x"]); goal.position.y = float(item[key]["y"])
            goal.orientation.w = 1.0; a.goals.append(goal)
        values = [a.position.position.x, a.position.position.y, a.yaw, a.radius,
                  a.desired_velocity, a.goal_radius] + [v for g in a.goals for v in (g.position.x, g.position.y)]
        if not all(math.isfinite(v) for v in values) or min(a.radius, a.goal_radius) <= 0 or not 0 < a.desired_velocity <= 1.0:
            raise ValueError("invalid pedestrian geometry, speed, or goals")
        result.agents.append(a)
    if not result.agents:
        raise ValueError("no pedestrians")
    return result, dict(zip(spec["agents"], models))


def entity(a):
    return Entity(a.id, a.name, a.position.position.x, a.position.position.y,
                  a.velocity.linear.x, a.velocity.linear.y, a.radius,
                  tuple((g.position.x, g.position.y) for g in a.goals))


def validate_response(request, response):
    if not response or not response.success:
        raise ValueError("compute_failed:" + (response.error if response else "empty response"))
    start = request.current_agents.header.stamp
    expected = start.sec*1_000_000_000 + start.nanosec + round(request.dt*1e9)
    stamp = response.updated_agents.header.stamp
    if (response.epoch, response.step) != (request.epoch, request.step) or stamp.sec*1_000_000_000+stamp.nanosec != expected:
        raise ValueError("late_or_invalid_compute_response")
    if response.updated_agents.header.frame_id != "map":
        raise ValueError("compute_frame_changed")
    if sorted((a.id,a.name) for a in response.updated_agents.agents) != sorted((a.id,a.name) for a in request.current_agents.agents):
        raise ValueError("compute_identity_changed")
    for a in response.updated_agents.agents:
        values = (a.position.position.x,a.position.position.y,a.yaw,
                  a.velocity.linear.x,a.velocity.linear.y,a.velocity.angular.z)
        if not all(math.isfinite(v) for v in values) or math.hypot(a.velocity.linear.x,a.velocity.linear.y)>1.00001:
            raise ValueError("invalid_compute_motion")
    expected_pairs={(a.id,r.name) for a in request.current_agents.agents for r in request.robots.agents}
    pairs={(v.agent_id,v.robot_name) for v in response.influences}
    if pairs!=expected_pairs or len(response.influences)!=len(expected_pairs):
        raise ValueError("invalid_compute_influences")
    return expected


class MultiSfmAdapter(Node):
    def __init__(self):
        super().__init__("multi_sfm_adapter")
        for name, value in {"robot_names": ["robot_1", "robot_2"], "agent_config_file": "",
                            "random_seed": 1, "psychology_model": "noop",
                            "observation_timeout": 0.6, "simulation_stamp_timeout": 1.0,
                            "service_timeout": 1.0}.items():
            self.declare_parameter(name, value)
        self.names = sorted(self.get_parameter("robot_names").value)
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError("robot_names must be nonempty and unique")
        self.robot_ids = {name: -i - 1 for i, name in enumerate(self.names)}
        self.history = {name: Timeline() for name in self.names}
        self.agents, self.models = load_people(self.get_parameter("agent_config_file").value)
        if set(self.models).intersection(self.names):
            raise ValueError("robot and pedestrian names overlap")
        self.epoch = uuid.uuid4().int & ((1 << 64) - 1); self.step = 0
        self.stamp_ns = None; self.clock_ns = None; self.scene_ready = False; self.spawned = False
        self.fault = ""; self.pending = None; self.update_pending = None
        self.display_stamp = None; self.last_display_wall = 0.0
        self.compute_count = 0; self.update_count = 0; self.max_compute_wall = 0.0
        self.model = MODELS[str(self.get_parameter("psychology_model").value)]()
        self.model.initialize({}, tuple(a.id for a in self.agents.agents), int(self.get_parameter("random_seed").value))
        self.mental = {a.id: MentalState() for a in self.agents.agents}
        self.agent_pub = self.create_publisher(Agents, "/multirobot/hunav/agents", 10)
        self.robot_pub = self.create_publisher(Agents, "/multirobot/hunav/robots", 10)
        self.force_pub = self.create_publisher(InteractionForces, "/multirobot/hunav/interactions", 10)
        self.health_pub = self.create_publisher(Bool, "/multirobot/hunav/health", 10)
        self.status_pub = self.create_publisher(String, "/multirobot/hunav/status", 10)
        self.psych_pub = self.create_publisher(String, "/multirobot/hunav/psychology", 10)
        self.create_subscription(String, "/multirobot/status", self._scene, 10)
        self.create_subscription(ClockMsg, "/clock", self._on_clock, qos_profile_sensor_data)
        for name in self.names:
            self.create_subscription(Odometry, f"/{name}/odom", lambda m, n=name: self._odom(n, m), qos_profile_sensor_data)
        self.compute = self.create_client(ComputeMultiAgents, "/multirobot/hunav/compute_agents")
        self.spawn = self.create_client(SpawnPedestrians, "/isaac/SpawnPedestrians")
        self.update = self.create_client(UpdatePedestrians, "/isaac/UpdatePedestrians")
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.01, self._tick, clock=self.wall_clock)
        self.report_timer = self.create_timer(0.1, self._report, clock=self.wall_clock)
        self.last_report = 0.0
        self.spawn_future = None
        self.warm_display_ready = False

    def _fail(self, reason):
        if not self.fault:
            self.fault = reason; self.epoch = (self.epoch + 1) % (1 << 64)
            self.get_logger().error(reason)
            self.health_pub.publish(Bool(data=False))
            for a in self.agents.agents:
                a.velocity = Twist()
            # Do not cancel an outstanding Isaac write: serialize a stop AFTER
            # its completion so a late update cannot overwrite the stop command.

    def _scene(self, msg):
        if msg.data.startswith("MULTIROBOT_SCENE_READY"):
            self.scene_ready = True

    def _on_clock(self, msg):
        stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self.clock_ns is not None and stamp < self.clock_ns and self.spawned:
            self._fail("clock_rewind")
        self.clock_ns = stamp

    def _odom(self, name, msg):
        if self.fault:
            return
        try:
            if msg.header.frame_id != f"{name}/odom" or msg.child_frame_id != f"{name}/base_link":
                raise ValueError("unexpected odometry frames")
            q = msg.pose.pose.orientation
            if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01:
                raise ValueError("invalid quaternion")
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            v = msg.twist.twist
            sample = Sample(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                            math.cos(yaw)*v.linear.x-math.sin(yaw)*v.linear.y,
                            math.sin(yaw)*v.linear.x+math.cos(yaw)*v.linear.y, v.angular.z)
            if not self.spawned and self.history[name].samples and sample.stamp_ns < self.history[name].samples[-1].stamp_ns:
                self.history[name] = Timeline()  # SpawnUrdf resets before scene readiness
            self.history[name].append(sample, time.monotonic())
        except Exception as error:
            self._fail(f"odom:{name}:{error}")

    def _robots(self, stamp_ns):
        result = Agents(); result.header.frame_id = "map"; result.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        for name in self.names:
            s = self.history[name].at(stamp_ns)
            a = Agent(); a.id = self.robot_ids[name]; a.name = name; a.type = Agent.ROBOT; a.group_id = -1
            a.radius = 0.35; a.position.position.x = s.x; a.position.position.y = s.y; a.yaw = s.yaw
            a.position.orientation.z = math.sin(s.yaw/2); a.position.orientation.w = math.cos(s.yaw/2)
            a.velocity.linear.x = s.vx; a.velocity.linear.y = s.vy; a.velocity.angular.z = s.wz
            a.linear_vel = math.hypot(s.vx, s.vy); a.angular_vel = s.wz
            result.agents.append(a)
        return result

    def _fresh(self):
        now = time.monotonic()
        for name, history in self.history.items():
            if not history.samples:
                return False
            if now - history.wall_seen > self.get_parameter("observation_timeout").value:
                raise ValueError(f"robot_stale:{name}")
            age = (self.clock_ns - history.samples[-1].stamp_ns) / 1e9
            # Isaac float time can encode a 50 ms frame as 50,000,001 ns.
            # Allow only numerical roundoff around the existing one-frame lead.
            if age < -0.050001 or age > self.get_parameter("simulation_stamp_timeout").value:
                raise ValueError(f"robot_stamp_stale:{name}:age={age:.9f}")
        return True

    def _dispatch_display(self, stop=False):
        req = UpdatePedestrians.Request()
        req.stamp = Time(nanoseconds=(self.clock_ns or 0) if stop else self.stamp_ns).to_msg()
        for a in self.agents.agents:
            p = Pedestrian(); p.name = a.name; p.pose = copy.deepcopy(a.position)
            p.twist = Twist() if stop else copy.deepcopy(a.velocity); req.pedestrians.append(p)
        self.update_pending = (self.update.call_async(req), time.monotonic(), self.stamp_ns, stop)
        self.last_display_wall = time.monotonic()

    def _tick(self):
        try:
            self._tick_impl()
        except Exception as error:
            self._fail(str(error))

    def _tick_impl(self):
        now = time.monotonic(); timeout = float(self.get_parameter("service_timeout").value)
        if self.update_pending:
            future, started, stamp, stopped = self.update_pending
            if future.done():
                self.update_pending = None
                response = future.result()
                if not response or list(response.results) != [0] * len(self.agents.agents):
                    self._fail("isaac_update_failed")
                elif not stopped:
                    self.display_stamp = stamp; self.update_count += 1
                elif self.stamp_ns is None:
                    self.warm_display_ready = True
                    self.warm_since = None
            elif now - started > (30.0 if self.stamp_ns is None else timeout):
                self._fail("isaac_update_timeout")
        if self.fault:
            if self.spawned and self.update_pending is None and self.update.service_is_ready() and now-self.last_display_wall >= 0.2:
                self._dispatch_display(stop=True)
            return
        if not self.scene_ready or not self.clock_ns or not all(h.samples for h in self.history.values()):
            return
        if not self.spawned:
            if self.spawn_future is None:
                if not (self.spawn.service_is_ready() and self.compute.service_is_ready() and self.update.service_is_ready()):
                    return
                req = SpawnPedestrians.Request()
                for a in self.agents.agents:
                    p = SpawnPedestrian(); p.model_ref = self.models[a.name]; p.pedestrian.name = a.name
                    p.pedestrian.pose = a.position; req.pedestrians.append(p)
                self.spawn_future = self.spawn.call_async(req); self.spawn_start = now
            elif self.spawn_future.done():
                res = self.spawn_future.result()
                if not res or list(res.results) != [0] * len(self.agents.agents):
                    raise ValueError("pedestrian_spawn_failed")
                self.spawned = True
                self.resume_wall = now
                self.warm_since = None
            elif now - self.spawn_start > 300:
                raise ValueError("pedestrian_spawn_timeout")
            return
        if self.stamp_ns is None:
            # Character creation blocks Isaac for seconds. Startup remains
            # unhealthy until every robot has a NEW sample after that operation.
            if now - self.resume_wall > 30:
                raise ValueError("post_spawn_odometry_timeout")
            if self.update_pending is None and now-self.last_display_wall >= 0.2:
                if not self.warm_display_ready:
                    self._dispatch_display(stop=True)
            if not self.warm_display_ready:
                return
            if not all(h.wall_seen >= self.resume_wall for h in self.history.values()):
                return
            try:
                fresh = self._fresh()
            except ValueError:
                fresh = False
            if not fresh:
                self.warm_since = None
                return
            if self.warm_since is None:
                self.warm_since = now
            if now-self.warm_since < 2.0:
                return
            self.stamp_ns = min(self.clock_ns, *(h.samples[-1].stamp_ns for h in self.history.values()))
            self.agents.header.stamp = Time(nanoseconds=self.stamp_ns).to_msg()
        if not self._fresh():
            return
        if self.pending:
            future, started, req, proposal, plugin_snapshot = self.pending
            if future.done():
                self.pending = None
                res = future.result()
                if (req.epoch, req.step) != (self.epoch, self.step):
                    raise ValueError("late_or_invalid_compute_response")
                expected_stamp = validate_response(req, res)
                self.agents = res.updated_agents; self.stamp_ns = expected_stamp
                self.mental = proposal.states; self.model.restore(plugin_snapshot)
                self.compute_count += 1; self.max_compute_wall = max(self.max_compute_wall, now-started)
                self.agent_pub.publish(self.agents)
                self.force_pub.publish(InteractionForces(header=self.agents.header, epoch=self.epoch, step=self.step, influences=res.influences))
            elif now-started > timeout:
                raise ValueError("compute_timeout")
        if self.update_pending is None and now-self.last_display_wall >= 0.1:
            self._dispatch_display()
        if (self.clock_ns - self.stamp_ns) / 1e9 > 1.0:
            raise ValueError("integration_lag")
        if self.pending is not None:
            return
        available = min(self.clock_ns, *(h.samples[-1].stamp_ns for h in self.history.values()))
        if available-self.stamp_ns < 25_000_000:
            return
        robots = self._robots(self.stamp_ns)
        context = make_context(self.stamp_ns, 0.025, [entity(a) for a in self.agents.agents], [entity(a) for a in robots.agents])
        proposal, proposed_plugin = prepare_step(self.model, context, self.mental)
        self.step += 1
        req = ComputeMultiAgents.Request(epoch=self.epoch, step=self.step, dt=0.025,
                                        current_agents=copy.deepcopy(self.agents), robots=robots)
        for agent_id, modifier in sorted(proposal.modifiers.items()):
            req.modifiers.append(BehaviorModifiers(agent_id=agent_id, **asdict(modifier)))
        self.robot_pub.publish(robots)
        self.pending = (self.compute.call_async(req), now, req, proposal, proposed_plugin)

    def _report(self):
        now = time.monotonic()
        healthy = self.spawned and not self.fault and self.update_count > 0
        self.health_pub.publish(Bool(data=healthy))
        if now-self.last_report < 1.0:
            return
        self.last_report = now
        payload = {"backend": "multi_sfm", "interaction_scope": "all_robots", "robots": self.names,
                   "healthy": healthy, "fault": self.fault, "epoch": self.epoch, "step": self.step,
                   "compute": self.compute_count, "updates": self.update_count, "max_dt": 0.025,
                   "max_compute_wall_s": self.max_compute_wall,
                   "integration_lag_s": None if self.stamp_ns is None else (self.clock_ns-self.stamp_ns)/1e9,
                   "display_lag_s": None if self.display_stamp is None else (self.clock_ns-self.display_stamp)/1e9}
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        self.psych_pub.publish(String(data=json.dumps({"schema_version": 1, "stamp_ns": self.stamp_ns,
            "plugin": self.model.snapshot(), "states": {str(i): asdict(s) for i, s in self.mental.items()}}, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = MultiSfmAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.health_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
