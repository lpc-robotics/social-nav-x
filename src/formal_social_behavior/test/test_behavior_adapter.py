import types
import unittest

from formal_social_behavior.behavior_adapter import (
    BehaviorProfile,
    BehaviorType,
    apply_profile_to_agent,
    default_behavior_profiles,
    profile_for_state,
    profile_signature,
)
from formal_social_behavior.config import EventThresholds, FormalSocialConfig
from formal_social_behavior.model import FormalState


class BehaviorProfileTests(unittest.TestCase):
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

    def test_profile_validation(self):
        with self.assertRaises(ValueError):
            BehaviorProfile(behavior_type=BehaviorType.CURIOUS, vel=-0.1)
        with self.assertRaises(ValueError):
            BehaviorProfile(behavior_type=99)


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
                            "curious": {"vel": 0.9},
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

    def test_all_states_require_profiles(self):
        with self.assertRaises(ValueError):
            FormalSocialConfig(
                behavior_profiles={
                    FormalState.NORMAL: profile_for_state(FormalState.NORMAL)
                }
            )


if __name__ == "__main__":
    unittest.main()
