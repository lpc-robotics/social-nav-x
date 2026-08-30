"""ROS-independent validation of HuNav compute responses."""

from __future__ import annotations

import math
from typing import Any, Iterable


def _finite_values(label: str, values: Iterable[float]) -> None:
    for value in values:
        if not math.isfinite(float(value)):
            raise RuntimeError(f"{label} contains a non-finite value")


def _validate_pose(label: str, pose: Any) -> None:
    _finite_values(
        label,
        (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def _validate_agent(agent: Any) -> None:
    label = f"agent id={int(agent.id)}"
    _validate_pose(f"{label} pose", agent.position)
    _finite_values(
        f"{label} velocity",
        (
            agent.velocity.linear.x,
            agent.velocity.linear.y,
            agent.velocity.linear.z,
            agent.velocity.angular.x,
            agent.velocity.angular.y,
            agent.velocity.angular.z,
            agent.yaw,
            agent.desired_velocity,
            agent.radius,
            agent.linear_vel,
            agent.angular_vel,
        ),
    )
    for index, goal in enumerate(agent.goals):
        _validate_pose(f"{label} goal[{index}]", goal)


def validate_compute_response(response: Any, *, expected_agents: Any) -> None:
    """Reject missing, resized, reordered, or non-finite HuNav responses."""

    if response is None or not hasattr(response, "updated_agents"):
        raise RuntimeError("raw compute_agents returned no response")
    agents = response.updated_agents.agents
    expected = list(expected_agents)
    if len(agents) != len(expected):
        raise RuntimeError(
            "raw compute_agents changed agent count: "
            f"expected {len(expected)}, got {len(agents)}"
        )
    for index, (agent, expected_agent) in enumerate(zip(agents, expected)):
        if int(agent.id) != int(expected_agent.id):
            raise RuntimeError(
                f"raw compute_agents changed agent id at index {index}"
            )
        if str(agent.name) != str(expected_agent.name):
            raise RuntimeError(
                f"raw compute_agents changed agent name at index {index}"
            )
        if int(agent.behavior.type) != int(expected_agent.behavior.type):
            raise RuntimeError(
                f"raw compute_agents changed behavior type at index {index}"
            )
        if len(agent.goals) != len(expected_agent.goals):
            raise RuntimeError(
                f"raw compute_agents changed goal count at index {index}"
            )
        _validate_agent(agent)
