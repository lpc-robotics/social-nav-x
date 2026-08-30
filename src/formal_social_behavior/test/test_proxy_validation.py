import math
import types
import unittest

from formal_social_behavior.proxy_validation import validate_compute_response


def point(x=0.0, y=0.0, z=0.0):
    return types.SimpleNamespace(x=x, y=y, z=z)


def pose(x=0.0):
    return types.SimpleNamespace(
        position=point(x=x),
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def agent(agent_id=1):
    return types.SimpleNamespace(
        id=agent_id,
        name=f"agent_{agent_id}",
        position=pose(),
        velocity=types.SimpleNamespace(linear=point(), angular=point()),
        yaw=0.0,
        desired_velocity=0.6,
        radius=0.4,
        linear_vel=0.0,
        angular_vel=0.0,
        behavior=types.SimpleNamespace(type=1),
        goals=[pose(3.0)],
    )


def response(*agents):
    return types.SimpleNamespace(
        updated_agents=types.SimpleNamespace(agents=list(agents))
    )


class ComputeResponseValidationTests(unittest.TestCase):
    def test_accepts_finite_response_with_unchanged_count(self):
        expected = agent()
        validate_compute_response(response(agent()), expected_agents=[expected])

    def test_rejects_missing_or_resized_response(self):
        expected = agent()
        with self.assertRaises(RuntimeError):
            validate_compute_response(None, expected_agents=[expected])
        with self.assertRaises(RuntimeError):
            validate_compute_response(response(), expected_agents=[expected])

    def test_rejects_identity_type_and_goal_count_changes(self):
        mutations = (
            lambda value: setattr(value, "id", 2),
            lambda value: setattr(value, "name", "renamed"),
            lambda value: setattr(value.behavior, "type", 5),
            lambda value: value.goals.clear(),
        )
        for mutate in mutations:
            actual = agent()
            mutate(actual)
            with self.subTest(mutate=mutate), self.assertRaises(RuntimeError):
                validate_compute_response(
                    response(actual), expected_agents=[agent()]
                )

    def test_rejects_non_finite_pose_velocity_and_goal(self):
        for field in ("pose", "velocity", "goal"):
            candidate = agent()
            if field == "pose":
                candidate.position.position.x = math.nan
            elif field == "velocity":
                candidate.velocity.linear.x = math.inf
            else:
                candidate.goals[0].position.x = math.nan
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                validate_compute_response(
                    response(candidate), expected_agents=[agent()]
                )


if __name__ == "__main__":
    unittest.main()
