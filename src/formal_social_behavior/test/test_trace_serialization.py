import math
import unittest

from formal_social_behavior.automaton import SocialAutomaton
from formal_social_behavior.model import (
    EventSnapshot,
    FormalState,
    InteractionMetrics,
    SocialEvent,
)
from formal_social_behavior.trace_serialization import (
    json_text,
    observation_payload,
    transition_payload,
)


def visible_snapshot(stamp_ns):
    return EventSnapshot(
        sim_time_ns=stamp_ns,
        events=frozenset({SocialEvent.ROBOT_VISIBLE}),
        metrics=InteractionMetrics(
            distance=5.0,
            closing_speed=0.0,
            ttc_seconds=math.inf,
            bearing_to_robot=0.0,
            relative_bearing=0.0,
        ),
    )


class TraceSerializationTests(unittest.TestCase):
    def test_payload_contains_required_schema_fields_and_no_nan(self):
        snapshot = visible_snapshot(7)
        payload = observation_payload(
            event_snapshot=snapshot,
            state=FormalState.ATTENTION,
            behavior_type=1,
            agent_id=1,
            agent_name="formal_human",
            config_sha256="abc",
            reset_count=0,
        )
        text = json_text(payload)
        self.assertIn('"schema_version":1', text)
        self.assertIn('"state":"ATTENTION"', text)
        self.assertIn('"events":["ROBOT_VISIBLE"]', text)
        self.assertIn('"ttc_seconds":null', text)
        self.assertNotIn("Infinity", text)
        self.assertNotIn("NaN", text)

    def test_same_recorded_input_produces_identical_transition_jsonl(self):
        def replay():
            automaton = SocialAutomaton()
            context = automaton.initial_context()
            lines = []
            for stamp in (7, 8):
                events = visible_snapshot(stamp)
                step = automaton.step(context, events)
                context = step.context
                if step.transition is None:
                    continue
                observation = observation_payload(
                    event_snapshot=events,
                    state=context.state,
                    behavior_type=1,
                    agent_id=1,
                    agent_name="formal_human",
                    config_sha256="abc",
                    reset_count=0,
                )
                lines.append(
                    json_text(
                        transition_payload(observation, step.transition)
                    )
                )
            return "\n".join(lines) + "\n"

        self.assertEqual(replay(), replay())


if __name__ == "__main__":
    unittest.main()
