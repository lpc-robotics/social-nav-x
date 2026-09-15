import importlib.util
import math
import os

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseStamped
from hunav_msgs.msg import Agent, Agents
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


def load_visualizer_module():
    path = os.environ["MPC_VISUALIZER_PATH"]
    spec = importlib.util.spec_from_file_location("mpc_visualizer", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def marker_by_namespace(markers, namespace):
    return next(marker for marker in markers if marker.ns == namespace)


def test_path_pose_frames_inherit_path_frame_without_mutating_input():
    visualizer = load_visualizer_module()
    message = Path()
    message.header.frame_id = "map"
    empty_frame_pose = PoseStamped()
    empty_frame_pose.pose.orientation.w = 1.0
    matching_frame_pose = PoseStamped()
    matching_frame_pose.header.frame_id = "map"
    matching_frame_pose.pose.orientation.w = 1.0
    message.poses = [empty_frame_pose, matching_frame_pose]

    result = visualizer.normalize_path_pose_frames(message)

    assert result.header.frame_id == "map"
    assert [pose.header.frame_id for pose in result.poses] == ["map", "map"]
    assert message.poses[0].header.frame_id == ""


def test_path_pose_frame_conflict_is_rejected():
    visualizer = load_visualizer_module()
    message = Path()
    message.header.frame_id = "map"
    pose = PoseStamped()
    pose.header.frame_id = "odom"
    message.poses = [pose]

    try:
        visualizer.normalize_path_pose_frames(message)
    except ValueError as error:
        assert "conflict" in str(error)
    else:
        raise AssertionError("conflicting Path frames were accepted")


def test_human_marker_geometry_and_semantics():
    visualizer = load_visualizer_module()
    message = Agents()
    message.header.frame_id = "map"
    human = Agent()
    human.id = 6
    human.name = "threatening_actor"
    human.position.position.x = 1.0
    human.position.position.y = 2.0
    human.velocity.linear.x = 0.4
    human.velocity.linear.y = -0.2
    human.radius = 0.3
    human.behavior.type = 6
    goal = Pose()
    goal.position.x = 4.0
    goal.position.y = 5.0
    human.goals = [goal]
    message.agents = [human]

    stamp = Time(sec=12, nanosec=34)
    result = visualizer.build_human_markers(
        message=message,
        marker_stamp=stamp,
        measurement_age=0.2,
        prediction_horizon=2.5,
        prediction_dt=0.1,
        robot_radius=0.326,
        safe_distance=0.35,
        geometry_uncertainty=0.05,
        body_height=1.7,
        marker_lifetime=0.6,
    )

    assert result.markers[0].action == Marker.DELETEALL
    assert len(result.markers) == 7
    assert all(marker.header.frame_id == "map" for marker in result.markers[1:])
    assert all(marker.header.stamp == stamp for marker in result.markers[1:])

    body = marker_by_namespace(result.markers, "human_body")
    assert body.type == Marker.CYLINDER
    assert math.isclose(body.pose.position.x, 1.08)
    assert math.isclose(body.pose.position.y, 1.96)
    assert math.isclose(body.scale.x, 0.6)
    assert body.color.r > body.color.g

    safety = marker_by_namespace(result.markers, "human_mpc_exclusion")
    expected_diameter = 2.0 * (0.3 + 0.326 + 0.05 + 0.35)
    assert math.isclose(safety.scale.x, expected_diameter)
    assert math.isclose(safety.scale.y, expected_diameter)

    prediction = marker_by_namespace(result.markers, "human_prediction")
    assert len(prediction.points) == 26
    assert math.isclose(prediction.points[0].x, 1.08)
    assert math.isclose(prediction.points[0].y, 1.96)
    assert math.isclose(prediction.points[-1].x, 2.08)
    assert math.isclose(prediction.points[-1].y, 1.46)

    velocity = marker_by_namespace(result.markers, "human_velocity")
    assert velocity.type == Marker.ARROW
    assert math.isclose(velocity.points[1].x - velocity.points[0].x, 0.4)
    assert math.isclose(velocity.points[1].y - velocity.points[0].y, -0.2)

    label = marker_by_namespace(result.markers, "human_label")
    assert label.text == "ID 6 | threatening | 0.45 m/s"
    goals = marker_by_namespace(result.markers, "human_goals")
    assert len(goals.points) == 1
    assert goals.points[0].x == 4.0
    assert goals.points[0].y == 5.0


def test_empty_snapshot_clears_old_human_markers():
    visualizer = load_visualizer_module()
    message = Agents()
    message.header.frame_id = "map"
    result = visualizer.build_human_markers(
        message=message,
        marker_stamp=Time(sec=1),
        measurement_age=0.0,
        prediction_horizon=2.5,
        prediction_dt=0.1,
        robot_radius=0.326,
        safe_distance=0.35,
        geometry_uncertainty=0.05,
        body_height=1.7,
        marker_lifetime=0.6,
    )
    assert len(result.markers) == 1
    assert result.markers[0].action == Marker.DELETEALL
