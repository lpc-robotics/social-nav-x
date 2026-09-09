"""Bounded Nav2-free, odometry-driven two-human GPU acceptance scenarios."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from hunav_msgs.msg import Agents
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from ..scenario_verifier import _angle_error, _quaternion_yaw


SCENARIOS = ("formation", "near", "crossing", "asymmetric", "fast", "disabled")


class MultiScenarioVerifier(Node):
    def __init__(self):
        super().__init__("verify_formal_social_multi_scenario")
        self.declare_parameter("scenario", "formation")
        self.declare_parameter("round", 1)
        self.scenario = self.get_parameter("scenario").value
        self.round = self.get_parameter("round").value
        if self.scenario not in SCENARIOS or self.round < 1:
            raise ValueError("invalid scenario/round")
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/formal_social_behavior/multi/states", self.on_frame, 10)
        self.create_subscription(Agents, "/human_states", self.on_humans, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.frame = None
        self.frames = []
        self.humans = {}
        self.robot = None
        self.stamp = None
        self.errors = []
        self.max_outward = {1: -math.inf, 2: -math.inf}
        self.motion_evidence = {}

    def on_frame(self, message):
        try:
            frame = json.loads(message.data)
            if frame["schema_version"] != 2 or [a["agent_id"] for a in frame["agents"]] != [1, 2]:
                raise ValueError("invalid schema or IDs")
            social = sum(a["state"] == "SOCIAL" for a in frame["agents"])
            if social not in (0, 2) or frame["pair"]["active"] != (social == 2):
                raise ValueError("partial pair commit")
            if self.frame and frame["commit_seq"] <= self.frame["commit_seq"]:
                raise ValueError("non-increasing commit sequence")
            # A short SCARED response can recover before the lower-rate
            # /human_states publisher exposes its type. The next committed
            # input still contains that actual HuNav result, with robot and
            # human positions/velocities matched to the same snapshot.
            scared_ids = {a["agent_id"] for a in frame["agents"] if a["state"] == "SCARED"}
            scared_ids.update(t["agent_id"] for t in frame["transitions"] if t["old_state"] == "SCARED")
            robot = frame["input"]["robot"]
            for human in frame["input"]["humans"]:
                if human["agent_id"] in scared_ids:
                    dx, dy = human["x"] - robot["x"], human["y"] - robot["y"]
                    distance = math.hypot(dx, dy)
                    if distance > 1e-9:
                        outward = (human["vx"] * dx + human["vy"] * dy) / distance
                        agent_id = human["agent_id"]
                        self.max_outward[agent_id] = max(self.max_outward[agent_id], outward)
            self.frame = frame
            self.frames.append(frame)
        except Exception as exc:
            self.errors.append(str(exc))

    def on_humans(self, message):
        if {a.id for a in message.agents} != {1, 2}:
            self.errors.append("human IDs changed")
            return
        self.humans = {a.id: a for a in message.agents}
        for agent in message.agents:
            values = (agent.position.position.x, agent.position.position.y, agent.yaw,
                      agent.velocity.linear.x, agent.velocity.linear.y)
            if not all(math.isfinite(v) for v in values):
                self.errors.append("non-finite human state")
            if agent.behavior.type == 4 and self.robot:
                dx = agent.position.position.x - self.robot[0]
                dy = agent.position.position.y - self.robot[1]
                distance = math.hypot(dx, dy)
                if distance > 1e-9:
                    outward = (agent.velocity.linear.x * dx + agent.velocity.linear.y * dy) / distance
                    self.max_outward[agent.id] = max(self.max_outward[agent.id], outward)

    def on_odom(self, message):
        pose = message.pose.pose
        self.robot = (pose.position.x, pose.position.y, _quaternion_yaw(pose.orientation))
        self.stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not all(math.isfinite(v) for v in self.robot):
            self.errors.append("non-finite odometry")

    def velocity(self, linear=0., angular=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd.publish(msg)

    def until(self, predicate, *, timeout=90., linear=0., controller=None):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.errors:
                raise AssertionError(self.errors)
            if self.count_publishers("/cmd_vel") != 1:
                raise AssertionError("acceptance requires exactly one cmd_vel publisher")
            if controller:
                controller()
            else:
                self.velocity(linear)
            rclpy.spin_once(self, timeout_sec=.02)
            if predicate():
                self.velocity()
                return
        raise AssertionError(f"scenario timeout: states={self.states()} robot={self.robot}")

    def states(self):
        return tuple(a["state"] for a in self.frame["agents"]) if self.frame else ()

    def hold(self, seconds, states=None):
        start = self.stamp
        def done():
            if states is not None and self.states() != states:
                raise AssertionError(f"hold changed states: {self.states()}")
            return self.stamp - start >= seconds
        self.until(done)

    def drive_x(self, target, speed=.15):
        direction = 1 if target > self.robot[0] else -1
        self.until(lambda: direction * (self.robot[0] - target) >= 0,
                   linear=direction * speed)

    def regular_stopped(self):
        assert len(self.humans) == 2
        for agent in self.humans.values():
            assert agent.behavior.type == 1
            assert math.hypot(agent.velocity.linear.x, agent.velocity.linear.y) <= .02

    def surprised_facing(self, ids):
        def ready():
            evidence = {}
            for agent_id in ids:
                agent = self.humans[agent_id]
                desired = math.atan2(self.robot[1] - agent.position.position.y,
                                     self.robot[0] - agent.position.position.x)
                yaw = _quaternion_yaw(agent.position.orientation)
                speed = math.hypot(agent.velocity.linear.x, agent.velocity.linear.y)
                error = math.degrees(_angle_error(yaw, desired))
                if agent.behavior.type != 3 or speed > .02 or error > 3 or _angle_error(yaw, agent.yaw) > math.radians(.25):
                    return False
                evidence[agent_id] = {"speed": speed, "facing_error_degrees": error}
            self.motion_evidence.update(evidence)
            return True
        self.until(ready, timeout=30.)

    def run(self):
        # Discovery precedes the exclusive-publisher gate.
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and (not self.robot or len(self.humans) != 2):
            rclpy.spin_once(self, timeout_sec=.1)
        assert self.robot and len(self.humans) == 2, "missing odometry/human observations"
        if self.scenario == "disabled":
            self.hold(10.)
            self.regular_stopped()
            assert not self.frames, "disabled proxy published formal states"
        else:
            self.until(lambda: self.states() == ("SOCIAL", "SOCIAL"))
            self.hold(10., ("SOCIAL", "SOCIAL"))
            self.regular_stopped()
            assert self.frame["reset_count"] == 0
            if self.scenario == "near":
                self.drive_x(4.9)
                self.hold(5., ("SOCIAL", "SOCIAL"))
                assert any("ROBOT_NEAR" in a["events"] for a in self.frame["agents"])
                assert self.frame["reset_count"] == 0
            elif self.scenario in ("crossing", "asymmetric"):
                self.drive_x(5.24)
                if self.scenario == "asymmetric":
                    # Matrix starts this case at y=2.4: the same shared space
                    # entry is a personal-space violation only for A.
                    self.until(lambda: self.states() == ("SCARED", "SURPRISED"))
                    self.until(lambda: self.max_outward[1] > .01)
                    self.surprised_facing((2,))
                else:
                    self.until(lambda: self.states() == ("SURPRISED", "SURPRISED"))
                    self.surprised_facing((1, 2))
                    self.drive_x(7.0)
                    assert self.robot[0] > 6.8, "robot did not cross the frozen pair segment"
                    self.hold(1.)
            elif self.scenario == "fast":
                # Enter the existing Scared profile's 3 m activation range
                # slowly while remaining outside the social capsule. From
                # x=3 the initial distance is >3 m and braking variability can
                # leave a correct SCARED automaton with no active SFM action.
                self.drive_x(3.8)
                self.hold(.5, ("SOCIAL", "SOCIAL"))
                self.until(lambda: "SCARED" in self.states(), linear=.8)
                self.until(lambda: max(self.max_outward.values()) > .01)
                assert any(t["cause"] == "ROBOT_FAST_APPROACH" for f in self.frames for t in f["transitions"])
            if self.scenario in ("crossing", "asymmetric"):
                intrusions = [e for f in self.frames for e in f["shared_events"] if e["name"] == "ROBOT_INTRUSION"]
                assert len(intrusions) == 1, f"expected one intrusion edge, got {len(intrusions)}"
            if self.scenario in ("formation", "near", "fast"):
                assert not any(e["name"] == "ROBOT_INTRUSION" for f in self.frames for e in f["shared_events"])
        result = {"scenario": self.scenario, "round": self.round, "cmd_vel_publishers": 1,
                  "observed_commits": len(self.frames), "robot": self.robot,
                  "reset_count": self.frame["reset_count"] if self.frame else 0,
                  "motion": self.motion_evidence,
                  "outward_sources": ["stamp_matched_committed_input", "human_states"],
                  "max_outward": {str(k): v if math.isfinite(v) else None for k, v in self.max_outward.items()}}
        print("FORMAL_MULTI_SCENARIO_OK " + json.dumps(result, allow_nan=False, sort_keys=True), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = MultiScenarioVerifier()
    try:
        node.run()
    finally:
        node.velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
