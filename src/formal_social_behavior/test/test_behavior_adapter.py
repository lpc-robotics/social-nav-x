import math
import types
import unittest

from formal_social_behavior.behavior_adapter import (
    BehaviorProfile,
    BehaviorType,
    apply_profile_to_agent,
    default_behavior_profiles,
    profile_for_state,
    profile_signature,
    regular_goal_reached,
    stop_agent_motion,
)
from formal_social_behavior.config import (
    AutomatonTiming,
    EventThresholds,
    FormalSocialConfig,
)
from formal_social_behavior.model import FormalState


class BehaviorProfileTests(unittest.TestCase):
    @staticmethod
    def _kinematic_agent(*, x=5.85, goal_x=6.0, goal_radius=0.3):
        def point(x_value=0.0):
            return types.SimpleNamespace(
                x=x_value,
                y=0.0,
                z=0.0,
            )

        return types.SimpleNamespace(
            position=types.SimpleNamespace(position=point(x)),
            velocity=types.SimpleNamespace(
                linear=point(-0.21),
                angular=point(0.03),
            ),
            linear_vel=0.21,
            angular_vel=0.03,
            goal_radius=goal_radius,
            goals=[types.SimpleNamespace(position=point(goal_x))],
        )

    def test_verified_default_state_mapping(self):
        profiles = default_behavior_profiles()
        self.assertEqual(profiles[FormalState.NORMAL].type, 1)
        self.assertEqual(profiles[FormalState.ATTENTION].type, 1)
        self.assertEqual(profiles[FormalState.CURIOUS].type, 5)
        self.assertEqual(profiles[FormalState.CURIOUS].duration, 30.0)
        self.assertFalse(profiles[FormalState.CURIOUS].once)
        self.assertEqual(profiles[FormalState.CURIOUS].vel, 0.8)
        self.assertEqual(profiles[FormalState.CURIOUS].dist, 1.5)
        self.assertEqual(profiles[FormalState.SURPRISED].type, 3)
        self.assertEqual(profiles[FormalState.SURPRISED].dist, 4.0)
        self.assertEqual(profiles[FormalState.SCARED].type, 4)
        self.assertEqual(profiles[FormalState.SCARED].duration, 40.0)
        self.assertEqual(profiles[FormalState.SCARED].dist, 3.0)
        for profile in profiles.values():
            self.assertEqual(profile.configuration, 1)
            self.assertEqual(profile.goal_force_factor, 2.0)
            self.assertEqual(profile.obstacle_force_factor, 10.0)
            self.assertEqual(profile.social_force_factor, 5.0)
            self.assertEqual(profile.other_force_factor, 20.0)

    def test_profile_lookup_and_signature(self):
        normal = profile_for_state(FormalState.NORMAL)
        attention = profile_for_state(FormalState.ATTENTION)
        self.assertEqual(profile_signature(normal), profile_signature(attention))

    def test_apply_profile_deep_copies_and_does_not_mutate_input(self):
        behavior = types.SimpleNamespace(
            type=1,
            state=7,
            configuration=0,
            duration=1.0,
            once=True,
            vel=0.1,
            dist=0.2,
            goal_force_factor=1.0,
            obstacle_force_factor=1.0,
            social_force_factor=1.0,
            other_force_factor=1.0,
        )
        agent = types.SimpleNamespace(
            id=1,
            behavior=behavior,
            goals=[types.SimpleNamespace(x=3.0)],
        )
        profile = profile_for_state(FormalState.CURIOUS)
        candidate = apply_profile_to_agent(agent, profile)

        self.assertIsNot(candidate, agent)
        self.assertIsNot(candidate.behavior, agent.behavior)
        self.assertIsNot(candidate.goals, agent.goals)
        self.assertEqual(agent.behavior.type, 1)
        self.assertEqual(agent.behavior.state, 7)
        self.assertEqual(candidate.behavior.type, 5)
        self.assertEqual(candidate.behavior.state, 0)
        self.assertEqual(candidate.behavior.vel, 0.8)
        self.assertEqual(candidate.behavior.dist, 1.5)

    def test_apply_rejects_incomplete_behavior_object(self):
        agent = types.SimpleNamespace(behavior=types.SimpleNamespace(type=1))
        with self.assertRaises(TypeError):
            apply_profile_to_agent(
                agent, profile_for_state(FormalState.NORMAL)
            )

    def test_regular_goal_reached_matches_hunav_extra_tolerance(self):
        self.assertTrue(regular_goal_reached(self._kinematic_agent(x=5.61)))
        self.assertFalse(regular_goal_reached(self._kinematic_agent(x=5.59)))
        no_goals = self._kinematic_agent()
        no_goals.goals = []
        self.assertFalse(regular_goal_reached(no_goals))

    def test_stop_agent_motion_is_a_complete_deep_copy(self):
        agent = self._kinematic_agent()
        stopped = stop_agent_motion(agent)

        self.assertIsNot(stopped, agent)
        self.assertIsNot(stopped.velocity, agent.velocity)
        self.assertEqual(agent.velocity.linear.x, -0.21)
        self.assertEqual(agent.velocity.angular.x, 0.03)
        self.assertEqual(agent.linear_vel, 0.21)
        self.assertEqual(
            (
                stopped.velocity.linear.x,
                stopped.velocity.linear.y,
                stopped.velocity.linear.z,
                stopped.velocity.angular.x,
                stopped.velocity.angular.y,
                stopped.velocity.angular.z,
                stopped.linear_vel,
                stopped.angular_vel,
            ),
            (0.0,) * 8,
        )

    def test_profile_validation(self):
        with self.assertRaises(ValueError):
            BehaviorProfile(behavior_type=BehaviorType.CURIOUS, vel=-0.1)
        with self.assertRaises(ValueError):
            BehaviorProfile(behavior_type=99)
        with self.assertRaises(ValueError):
            BehaviorProfile(
                behavior_type=BehaviorType.CURIOUS,
                state=7,
            )
        with self.assertRaises(ValueError):
            BehaviorProfile(
                behavior_type=BehaviorType.CURIOUS,
                configuration=99,
            )

    def test_behavior_type_rejects_implicit_coercion(self):
        for invalid_type in (True, 1.0, "1"):
            with self.subTest(value=invalid_type, source="constructor"):
                with self.assertRaises(TypeError):
                    BehaviorProfile(behavior_type=invalid_type)
            with self.subTest(value=invalid_type, source="mapping"):
                with self.assertRaises(TypeError):
                    BehaviorProfile.from_mapping({"type": invalid_type})

        with self.assertRaisesRegex(ValueError, "cannot define both"):
            BehaviorProfile.from_mapping(
                {"type": 1, "behavior_type": 1}
            )

    def test_profile_numeric_fields_are_strict_finite_floats(self):
        numeric_fields = (
            "duration",
            "vel",
            "dist",
            "goal_force_factor",
            "obstacle_force_factor",
            "social_force_factor",
            "other_force_factor",
        )
        for name in numeric_fields:
            with self.subTest(field=name, value="integer"):
                profile = BehaviorProfile(
                    behavior_type=BehaviorType.REGULAR,
                    **{name: 1},
                )
                self.assertIs(type(getattr(profile, name)), float)
            for invalid_value in (True, "1.0"):
                with self.subTest(field=name, value=invalid_value):
                    with self.assertRaises(TypeError):
                        BehaviorProfile(
                            behavior_type=BehaviorType.REGULAR,
                            **{name: invalid_value},
                        )
            for invalid_value in (math.nan, math.inf, -math.inf):
                with self.subTest(field=name, value=invalid_value):
                    with self.assertRaisesRegex(ValueError, "finite"):
                        BehaviorProfile(
                            behavior_type=BehaviorType.REGULAR,
                            **{name: invalid_value},
                        )

    def test_integer_profile_fields_do_not_accept_coercion(self):
        for name, valid_value in (("state", 0), ("configuration", 1)):
            for invalid_value in (True, float(valid_value), str(valid_value)):
                with self.subTest(field=name, value=invalid_value):
                    with self.assertRaises(TypeError):
                        BehaviorProfile(
                            behavior_type=BehaviorType.REGULAR,
                            **{name: invalid_value},
                        )


class ConfigTests(unittest.TestCase):
    def test_parse_ros_parameter_document_and_profile_override(self):
        config = FormalSocialConfig.from_mapping(
            {
                "formal_social_behavior": {
                    "ros__parameters": {
                        "target_agent_id": 7,
                        "thresholds": {
                            "visible_enter_distance": 7.0,
                            "visible_exit_distance": 7.5,
                        },
                        "timing": {"attention_dwell_seconds": 0.75},
                        "behavior_profiles": {
                            "normal": {},
                            "attention": {},
                            "curious": {"vel": 0.9},
                            "surprised": {},
                            "scared": {},
                        },
                    }
                }
            }
        )
        self.assertEqual(config.target_agent_id, 7)
        self.assertEqual(config.thresholds.visible_enter_distance, 7.0)
        self.assertEqual(config.timing.attention_dwell_seconds, 0.75)
        self.assertEqual(
            config.behavior_profiles[FormalState.CURIOUS].type, 5
        )
        self.assertEqual(
            config.behavior_profiles[FormalState.CURIOUS].vel, 0.9
        )
        self.assertEqual(
            config.behavior_profiles[FormalState.CURIOUS].dist, 1.5
        )

    def test_invalid_hysteresis_bands_are_rejected(self):
        with self.assertRaises(ValueError):
            EventThresholds(
                visible_enter_distance=6.0,
                visible_exit_distance=5.0,
            )
        with self.assertRaises(ValueError):
            EventThresholds(
                fast_approach_enter_speed=0.3,
                fast_approach_exit_speed=0.35,
            )

    def test_threshold_numbers_are_strict_finite_floats(self):
        normalized = EventThresholds(visible_enter_distance=6)
        self.assertIs(type(normalized.visible_enter_distance), float)

        for name in EventThresholds.__dataclass_fields__:
            for invalid_value in (True, "1.0"):
                with self.subTest(field=name, value=invalid_value):
                    with self.assertRaises(TypeError):
                        EventThresholds(**{name: invalid_value})
            for invalid_value in (math.nan, math.inf, -math.inf):
                with self.subTest(field=name, value=invalid_value):
                    with self.assertRaisesRegex(ValueError, "finite"):
                        EventThresholds(**{name: invalid_value})

    def test_timing_numbers_are_strict_finite_floats(self):
        normalized = AutomatonTiming(
            attention_dwell_seconds=1,
            recovery_timeout_seconds=3,
            reentry_cooldown_seconds=1,
        )
        for name in AutomatonTiming.__dataclass_fields__:
            self.assertIs(type(getattr(normalized, name)), float)
            for invalid_value in (True, "1.0"):
                with self.subTest(field=name, value=invalid_value):
                    with self.assertRaises(TypeError):
                        AutomatonTiming(**{name: invalid_value})
            for invalid_value in (math.nan, math.inf, -math.inf):
                with self.subTest(field=name, value=invalid_value):
                    with self.assertRaisesRegex(ValueError, "finite"):
                        AutomatonTiming(**{name: invalid_value})

    def test_all_states_require_profiles(self):
        with self.assertRaises(ValueError):
            FormalSocialConfig(
                behavior_profiles={
                    FormalState.NORMAL: profile_for_state(FormalState.NORMAL)
                }
            )

        with self.assertRaisesRegex(ValueError, "missing states"):
            FormalSocialConfig.from_mapping(
                {"behavior_profiles": {"normal": {}}}
            )

    def test_config_rejects_typos_and_lossy_target_ids(self):
        with self.assertRaisesRegex(ValueError, "unknown event threshold"):
            FormalSocialConfig.from_mapping(
                {"thresholds": {"visible_enter_distnace": 7.0}}
            )
        with self.assertRaisesRegex(ValueError, "unknown automaton timing"):
            FormalSocialConfig.from_mapping(
                {"timing": {"attention_dwel_seconds": 0.75}}
            )
        with self.assertRaisesRegex(ValueError, "unknown formal social"):
            FormalSocialConfig.from_mapping({"target_agent_name": "human"})
        for invalid_id in (1.9, "1", True):
            with self.subTest(target_agent_id=invalid_id):
                with self.assertRaises(TypeError):
                    FormalSocialConfig.from_mapping(
                        {"target_agent_id": invalid_id}
                    )

    def test_config_rejects_wrapper_siblings_and_normalized_duplicates(self):
        with self.assertRaisesRegex(ValueError, "beside formal_social_behavior"):
            FormalSocialConfig.from_mapping(
                {
                    "formal_social_behavior": {"ros__parameters": {}},
                    "formal_social_behaviour": {},
                }
            )
        with self.assertRaisesRegex(ValueError, "beside ros__parameters"):
            FormalSocialConfig.from_mapping(
                {
                    "formal_social_behavior": {
                        "ros__parameters": {},
                        "target_agent_id": 1,
                    }
                }
            )
        complete_profiles = {
            state.value: {} for state in FormalState
        }
        complete_profiles["normal"] = {}
        with self.assertRaisesRegex(ValueError, "duplicate behavior profile"):
            FormalSocialConfig.from_mapping(
                {"behavior_profiles": complete_profiles}
            )


if __name__ == "__main__":
    unittest.main()
