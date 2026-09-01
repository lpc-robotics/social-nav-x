import math
import types
import unittest

from formal_social_behavior.scenario_verifier import (
    FormalScenarioVerifier,
    SURPRISED_MAX_FACING_ERROR,
    _angle_error,
    _quaternion_yaw,
    _visual_hold_seconds,
)


def _quaternion(yaw):
    return types.SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def _human(*, yaw, reported_yaw=None, speed=0.0):
    return types.SimpleNamespace(
        yaw=yaw if reported_yaw is None else reported_yaw,
        position=types.SimpleNamespace(
            position=types.SimpleNamespace(x=1.0, y=0.0),
            orientation=_quaternion(yaw),
        ),
        velocity=types.SimpleNamespace(
            linear=types.SimpleNamespace(x=speed, y=0.0),
        ),
    )


class ScenarioGeometryTests(unittest.TestCase):
    def test_quaternion_yaw_and_wrapped_angle(self):
        for yaw in (-math.pi, -2.4, 0.0, 2.4, math.pi):
            with self.subTest(yaw=yaw):
                self.assertLessEqual(
                    _angle_error(_quaternion_yaw(_quaternion(yaw)), yaw),
                    1.0e-12,
                )
        self.assertAlmostEqual(
            _angle_error(math.pi - 0.1, -math.pi + 0.1),
            0.2,
        )

    def test_visual_hold_validation(self):
        self.assertEqual(_visual_hold_seconds(0), 0.0)
        self.assertEqual(_visual_hold_seconds(2.5), 2.5)
        for invalid in (True, "2", -0.1, 60.1, math.nan, math.inf):
            with self.subTest(value=invalid):
                with self.assertRaises(RuntimeError):
                    _visual_hold_seconds(invalid)

    def test_surprised_checks_render_quaternion_with_three_degree_limit(self):
        verifier = object.__new__(FormalScenarioVerifier)
        verifier.robot_xy = (0.0, 0.0)
        verifier.human = _human(yaw=math.pi - math.radians(2.0))

        self.assertTrue(verifier.surprised_response_observed(2.4))
        self.assertLess(
            verifier.surprised_facing_error,
            SURPRISED_MAX_FACING_ERROR,
        )
        self.assertLessEqual(verifier.surprised_yaw_field_error, 1.0e-12)

        verifier.human = _human(yaw=math.pi - math.radians(4.0))
        self.assertFalse(verifier.surprised_response_observed(2.4))
        verifier.human = _human(
            yaw=math.pi - math.radians(2.0),
            speed=0.021,
        )
        self.assertFalse(verifier.surprised_response_observed(2.4))
        verifier.human = _human(
            yaw=math.pi - math.radians(2.0),
            reported_yaw=math.pi - math.radians(3.0),
        )
        self.assertFalse(verifier.surprised_response_observed(2.4))


class RecoveryMotionTests(unittest.TestCase):
    @staticmethod
    def _verifier(samples):
        verifier = object.__new__(FormalScenarioVerifier)
        verifier.human_samples = samples
        verifier.regular_motion_result = None
        return verifier

    def test_stopped_regular_is_consistent(self):
        samples = [
            (index * 25_000_000, 5.85, 3.0, 0.0, 1)
            for index in range(8)
        ]
        verifier = self._verifier(samples)
        self.assertEqual(verifier.regular_recovery_motion(0), "stopped")

    def test_moving_regular_is_consistent(self):
        samples = [
            (index * 25_000_000, 5.80 + index * 0.005, 3.0, 0.2, 1)
            for index in range(8)
        ]
        verifier = self._verifier(samples)
        self.assertEqual(verifier.regular_recovery_motion(0), "moving")

    def test_nonzero_velocity_with_frozen_pose_is_rejected(self):
        samples = [
            (index * 25_000_000, 5.852177, 3.0, 0.209789, 1)
            for index in range(8)
        ]
        verifier = self._verifier(samples)
        self.assertIsNone(verifier.regular_recovery_motion(0))
        self.assertIsNone(verifier.regular_motion_result)


if __name__ == "__main__":
    unittest.main()
