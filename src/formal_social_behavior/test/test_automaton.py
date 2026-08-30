import unittest

from formal_social_behavior.automaton import (
    TRANSITION_RULES,
    SocialAutomaton,
    transition_rules_for,
)
from formal_social_behavior.event_extractor import EventExtractor
from formal_social_behavior.model import (
    AutomatonContext,
    EventMemory,
    EventSnapshot,
    FormalState,
    InteractionMetrics,
    MotionSnapshot,
    PlanarKinematics,
    SocialEvent,
    TransitionCause,
)


SECOND = 1_000_000_000


def event_snapshot(stamp_ns, *events, closing_speed=0.0, ttc=100.0):
    return EventSnapshot(
        sim_time_ns=stamp_ns,
        events=frozenset(events),
        metrics=InteractionMetrics(
            distance=2.0,
            closing_speed=closing_speed,
            ttc_seconds=ttc,
            bearing_to_robot=0.0,
            relative_bearing=0.0,
        ),
    )


class AutomatonPathTests(unittest.TestCase):
    def setUp(self):
        self.automaton = SocialAutomaton()

    def test_slow_safe_approach_reaches_curious_after_dwell(self):
        context = self.automaton.initial_context()
        attention = self.automaton.step(
            context,
            event_snapshot(0, SocialEvent.ROBOT_VISIBLE),
        )
        before_dwell = self.automaton.step(
            attention.context,
            event_snapshot(
                int(0.49 * SECOND),
                SocialEvent.ROBOT_VISIBLE,
                SocialEvent.ROBOT_NEAR,
                SocialEvent.ROBOT_SAFE_APPROACH,
                closing_speed=0.15,
            ),
        )
        curious = self.automaton.step(
            before_dwell.context,
            event_snapshot(
                int(0.50 * SECOND),
                SocialEvent.ROBOT_VISIBLE,
                SocialEvent.ROBOT_NEAR,
                SocialEvent.ROBOT_SAFE_APPROACH,
                closing_speed=0.15,
            ),
        )
        self.assertEqual(attention.context.state, FormalState.ATTENTION)
        self.assertEqual(before_dwell.context.state, FormalState.ATTENTION)
        self.assertEqual(curious.context.state, FormalState.CURIOUS)
        self.assertEqual(curious.transition.cause, TransitionCause.ATTENTION_DWELL)

    def test_attention_does_not_accept_mid_speed_as_safe_approach(self):
        context = self.automaton.step(
            self.automaton.initial_context(),
            event_snapshot(0, SocialEvent.ROBOT_VISIBLE),
        ).context
        result = self.automaton.step(
            context,
            event_snapshot(
                SECOND,
                SocialEvent.ROBOT_VISIBLE,
                SocialEvent.ROBOT_NEAR,
                closing_speed=0.30,
            ),
        )
        self.assertEqual(result.context.state, FormalState.ATTENTION)

    def test_sudden_near_reaches_surprised_and_recovers_after_three_seconds(self):
        surprised = self.automaton.step(
            self.automaton.initial_context(),
            event_snapshot(
                SECOND,
                SocialEvent.ROBOT_VISIBLE,
                SocialEvent.ROBOT_NEAR,
                SocialEvent.SUDDEN_NEAR,
                closing_speed=0.3,
            ),
        )
        almost = self.automaton.step(
            surprised.context,
            event_snapshot(int(3.999 * SECOND), SocialEvent.ROBOT_VISIBLE),
        )
        recovered = self.automaton.step(
            almost.context,
            event_snapshot(4 * SECOND, SocialEvent.ROBOT_VISIBLE),
        )
        self.assertEqual(surprised.context.state, FormalState.SURPRISED)
        self.assertEqual(almost.context.state, FormalState.SURPRISED)
        self.assertEqual(recovered.context.state, FormalState.NORMAL)
        self.assertEqual(
            recovered.transition.cause, TransitionCause.RECOVERY_TIMEOUT
        )

    def test_danger_priority_is_personal_space_then_ttc_then_speed(self):
        all_dangers = self.automaton.step(
            self.automaton.initial_context(),
            event_snapshot(
                0,
                SocialEvent.PERSONAL_SPACE_VIOLATION,
                SocialEvent.TTC_LOW,
                SocialEvent.ROBOT_FAST_APPROACH,
            ),
        )
        ttc_and_speed = self.automaton.step(
            self.automaton.initial_context(),
            event_snapshot(
                1,
                SocialEvent.TTC_LOW,
                SocialEvent.ROBOT_FAST_APPROACH,
            ),
        )
        speed_only = self.automaton.step(
            self.automaton.initial_context(),
            event_snapshot(2, SocialEvent.ROBOT_FAST_APPROACH),
        )
        self.assertEqual(
            all_dangers.transition.cause,
            TransitionCause.PERSONAL_SPACE_VIOLATION,
        )
        self.assertEqual(ttc_and_speed.transition.cause, TransitionCause.TTC_LOW)
        self.assertEqual(
            speed_only.transition.cause, TransitionCause.ROBOT_FAST_APPROACH
        )

    def test_danger_escalates_each_non_scared_state(self):
        for state in (
            FormalState.NORMAL,
            FormalState.ATTENTION,
            FormalState.CURIOUS,
            FormalState.SURPRISED,
        ):
            with self.subTest(state=state):
                result = self.automaton.step(
                    AutomatonContext(state=state, state_entered_ns=0),
                    event_snapshot(SECOND, SocialEvent.TTC_LOW),
                )
                self.assertEqual(result.context.state, FormalState.SCARED)

    def test_scared_recovery_starts_on_first_safe_tick(self):
        scared = self.automaton.step(
            self.automaton.initial_context(),
            event_snapshot(0, SocialEvent.ROBOT_FAST_APPROACH),
        )
        first_safe = self.automaton.step(
            scared.context,
            event_snapshot(
                SECOND,
                SocialEvent.ROBOT_LEAVING,
                closing_speed=-0.2,
            ),
        )
        almost = self.automaton.step(
            first_safe.context,
            event_snapshot(
                int(3.999 * SECOND),
                SocialEvent.ROBOT_LEAVING,
                closing_speed=-0.2,
            ),
        )
        recovered = self.automaton.step(
            almost.context,
            event_snapshot(
                4 * SECOND,
                SocialEvent.ROBOT_LEAVING,
                closing_speed=-0.2,
            ),
        )
        self.assertIsNone(scared.context.safe_since_ns)
        self.assertEqual(first_safe.context.safe_since_ns, SECOND)
        self.assertEqual(almost.context.state, FormalState.SCARED)
        self.assertEqual(recovered.context.state, FormalState.NORMAL)

    def test_new_danger_restarts_scared_safe_timer(self):
        context = AutomatonContext(
            state=FormalState.SCARED,
            state_entered_ns=0,
            safe_since_ns=0,
            last_stamp_ns=2 * SECOND,
        )
        danger = self.automaton.step(
            context,
            event_snapshot(3 * SECOND, SocialEvent.TTC_LOW),
        )
        safe_again = self.automaton.step(
            danger.context,
            event_snapshot(
                4 * SECOND,
                SocialEvent.ROBOT_LEAVING,
                closing_speed=-0.2,
            ),
        )
        too_early = self.automaton.step(
            safe_again.context,
            event_snapshot(
                6 * SECOND,
                SocialEvent.ROBOT_LEAVING,
                closing_speed=-0.2,
            ),
        )
        self.assertIsNone(danger.context.safe_since_ns)
        self.assertEqual(safe_again.context.safe_since_ns, 4 * SECOND)
        self.assertEqual(too_early.context.state, FormalState.SCARED)

    def test_curious_requires_leaving_and_near_exit(self):
        context = AutomatonContext(
            state=FormalState.CURIOUS,
            state_entered_ns=0,
        )
        still_near = self.automaton.step(
            context,
            event_snapshot(
                SECOND,
                SocialEvent.ROBOT_LEAVING,
                SocialEvent.ROBOT_NEAR,
                closing_speed=-0.2,
            ),
        )
        outside = self.automaton.step(
            still_near.context,
            event_snapshot(
                2 * SECOND,
                SocialEvent.ROBOT_LEAVING,
                closing_speed=-0.2,
            ),
        )
        self.assertEqual(still_near.context.state, FormalState.CURIOUS)
        self.assertEqual(outside.context.state, FormalState.NORMAL)

    def test_robot_lost_immediately_recovers_non_normal_states(self):
        for state in (
            FormalState.ATTENTION,
            FormalState.CURIOUS,
            FormalState.SURPRISED,
            FormalState.SCARED,
        ):
            with self.subTest(state=state):
                result = self.automaton.step(
                    AutomatonContext(state=state, state_entered_ns=0),
                    event_snapshot(SECOND, SocialEvent.ROBOT_LOST),
                )
                self.assertEqual(result.context.state, FormalState.NORMAL)
                self.assertEqual(result.transition.cause, TransitionCause.ROBOT_LOST)


class AutomatonDeterminismTests(unittest.TestCase):
    def setUp(self):
        self.automaton = SocialAutomaton()

    def test_at_most_one_transition_per_timestamp(self):
        first = self.automaton.step(
            self.automaton.initial_context(),
            event_snapshot(10, SocialEvent.ROBOT_VISIBLE),
        )
        duplicate = self.automaton.step(
            first.context,
            event_snapshot(
                10,
                SocialEvent.ROBOT_NEAR,
                SocialEvent.ROBOT_SAFE_APPROACH,
            ),
        )
        self.assertEqual(first.context.state, FormalState.ATTENTION)
        self.assertEqual(duplicate.context.state, FormalState.ATTENTION)
        self.assertIsNone(duplicate.transition)

    def test_reentry_cooldown_and_danger_bypass(self):
        attention = self.automaton.step(
            self.automaton.initial_context(),
            event_snapshot(0, SocialEvent.ROBOT_VISIBLE),
        )
        normal = self.automaton.step(
            attention.context,
            event_snapshot(SECOND, SocialEvent.ROBOT_LOST),
        )
        cooldown = self.automaton.step(
            normal.context,
            event_snapshot(int(1.2 * SECOND), SocialEvent.ROBOT_VISIBLE),
        )
        after = self.automaton.step(
            cooldown.context,
            event_snapshot(int(1.5 * SECOND), SocialEvent.ROBOT_VISIBLE),
        )
        danger_during_cooldown = self.automaton.step(
            normal.context,
            event_snapshot(int(1.1 * SECOND), SocialEvent.TTC_LOW),
        )
        self.assertEqual(cooldown.context.state, FormalState.NORMAL)
        self.assertEqual(after.context.state, FormalState.ATTENTION)
        self.assertEqual(danger_during_cooldown.context.state, FormalState.SCARED)

    def test_clock_rollback_clears_all_timers_and_forces_normal(self):
        context = AutomatonContext(
            state=FormalState.SCARED,
            state_entered_ns=9,
            safe_since_ns=9,
            cooldown_until_ns=20,
            last_stamp_ns=10,
            last_transition_stamp_ns=9,
        )
        rollback_events = EventSnapshot(
            sim_time_ns=5,
            events=frozenset({SocialEvent.TIME_RESET}),
            metrics=event_snapshot(5).metrics,
            clock_rollback=True,
        )
        result = self.automaton.step(context, rollback_events)
        self.assertTrue(result.clock_reset)
        self.assertEqual(result.context.state, FormalState.NORMAL)
        self.assertIsNone(result.context.state_entered_ns)
        self.assertIsNone(result.context.safe_since_ns)
        self.assertIsNone(result.context.cooldown_until_ns)
        self.assertEqual(result.context.last_stamp_ns, 5)
        self.assertEqual(result.transition.cause, TransitionCause.TIME_RESET)

    def test_transition_table_is_closed_and_enumerable(self):
        self.assertTrue(TRANSITION_RULES)
        for rule in TRANSITION_RULES:
            self.assertTrue(rule.source_states)
            self.assertIsInstance(rule.destination, FormalState)
            for source in rule.source_states:
                self.assertIsInstance(source, FormalState)
        for state in FormalState:
            self.assertTrue(transition_rules_for(state))

    def test_replay_is_identical(self):
        observations = (
            MotionSnapshot(
                0,
                PlanarKinematics(5.0, 0.0, vx=-0.15, radius=0.0),
                PlanarKinematics(0.0, 0.0, radius=0.0),
            ),
            MotionSnapshot(
                int(0.5 * SECOND),
                PlanarKinematics(2.5, 0.0, vx=-0.15, radius=0.0),
                PlanarKinematics(0.0, 0.0, radius=0.0),
            ),
            MotionSnapshot(
                SECOND,
                PlanarKinematics(2.6, 0.0, vx=0.2, radius=0.0),
                PlanarKinematics(0.0, 0.0, radius=0.0),
            ),
            MotionSnapshot(
                2 * SECOND,
                PlanarKinematics(2.8, 0.0, vx=0.2, radius=0.0),
                PlanarKinematics(0.0, 0.0, radius=0.0),
            ),
        )

        def run_once():
            extractor = EventExtractor()
            automaton = SocialAutomaton()
            memory = EventMemory()
            context = automaton.initial_context()
            trace = []
            for observation in observations:
                event_result = extractor.evaluate(observation, memory)
                step = automaton.step(context, event_result.event_snapshot)
                memory = event_result.memory
                context = step.context
                if step.transition is not None:
                    trace.append(
                        (
                            step.transition.sim_time_ns,
                            step.transition.old_state.value,
                            step.transition.new_state.value,
                            step.transition.cause.value,
                        )
                    )
            return trace, memory, context

        self.assertEqual(run_once(), run_once())


if __name__ == "__main__":
    unittest.main()
