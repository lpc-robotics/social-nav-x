"""Phase 4 semantics, exercised without ROS or Isaac."""

from dataclasses import replace
import math
from itertools import product
from types import SimpleNamespace

import pytest

from formal_social_behavior.model import FormalState, PlanarKinematics, MotionSnapshot
from formal_social_behavior.automaton import SocialAutomaton
from formal_social_behavior.event_extractor import EventExtractor
from formal_social_behavior.multi_agent.automaton import MultiAgentAutomaton
from formal_social_behavior.multi_agent.automaton import DANGER
from formal_social_behavior.multi_agent.config import MultiConfig, PeerThresholds, default_profiles
from formal_social_behavior.multi_agent.model import MultiState, PairMemory, SceneSnapshot
from formal_social_behavior.multi_agent.peer_events import evaluate_geometry, segment_distance, point_segment_distance


SECOND = 1_000_000_000


def scene(t, x=3.0, y=3.0, vx=0.0, humans=None):
    if humans is None:
        humans = ((1, PlanarKinematics(6, 1.8, yaw=math.pi/2, radius=.4)),
                  (2, PlanarKinematics(6, 4.2, yaw=-math.pi/2, radius=.4)))
    return SceneSnapshot(round(t * SECOND), PlanarKinematics(x, y, vx=vx, radius=.35), tuple(humans))


def formed(engine=None):
    engine = engine or MultiAgentAutomaton()
    first = engine.evaluate(engine.initial_context(), scene(0))
    step = engine.evaluate(first.context, scene(1))
    assert states(step) == ("SOCIAL", "SOCIAL")
    return engine, step.context


def states(step):
    return tuple(agent.context.state.value for agent in step.context.agents)


def shared(step):
    return tuple(event.name.value for event in step.pair_evaluation.shared_events)


def test_synchronized_formation_holds_despite_robot_near():
    engine = MultiAgentAutomaton()
    first = engine.evaluate(engine.initial_context(), scene(0))
    before = engine.evaluate(first.context, scene(.999))
    assert states(before) == ("ATTENTION", "ATTENTION")
    formed_step = engine.evaluate(before.context, scene(1))
    assert states(formed_step) == ("SOCIAL", "SOCIAL")
    assert shared(formed_step) == ("SOCIAL_SPACE_FORMED",)
    assert formed_step.transitions[0].shared_event_ids == formed_step.transitions[1].shared_event_ids
    near = engine.evaluate(formed_step.context, scene(11, 4.85, vx=.15))
    assert states(near) == ("SOCIAL", "SOCIAL")
    assert near.transitions == ()
    assert engine.config.behavior_profiles[MultiState.SOCIAL] == default_profiles()[MultiState.NORMAL]


def test_social_intrusion_is_distinct_from_personal_danger():
    engine, context = formed()
    entered = engine.evaluate(context, scene(2, 5.21, vx=.15))
    assert states(entered) == ("SURPRISED", "SURPRISED")
    assert shared(entered) == ("ROBOT_INTRUSION", "SOCIAL_SPACE_BROKEN")
    assert all(t.cause == "ROBOT_INTRUSION" for t in entered.transitions)
    assert not entered.context.pair.active
    for _, event in entered.robot_events:
        assert event.metrics.distance > 1.0
        assert math.isinf(event.metrics.ttc_seconds)
    stationary = engine.evaluate(entered.context, scene(2.1, 5.21))
    assert shared(stationary) == ()
    assert stationary.transitions == ()


def test_same_shared_event_allows_asymmetric_response():
    engine, context = formed()
    entered = engine.evaluate(context, scene(2, 5.21, 2.4, .15))
    assert states(entered) == ("SCARED", "SURPRISED")
    assert entered.transitions[0].cause == "PERSONAL_SPACE_VIOLATION"
    assert entered.transitions[1].cause == "ROBOT_INTRUSION"
    assert entered.transitions[0].shared_event_ids == entered.transitions[1].shared_event_ids


@pytest.mark.parametrize("bits,intrusion", tuple(product(product((False, True), repeat=3), (False, True))))
def test_social_guard_priority_is_exhaustively_deterministic(bits, intrusion):
    engine, context = formed()
    snapshot = scene(2, 5.21 if intrusion else 3.)
    extractor = engine.extractor
    selected = frozenset(event for event, active in zip(DANGER, bits) if active)
    def evaluate(motion, memory):
        result = extractor.evaluate(motion, memory)
        active = selected if motion.human.y < 3 else frozenset()
        return SimpleNamespace(event_snapshot=replace(result.event_snapshot, events=active), memory=result.memory)
    engine.extractor = SimpleNamespace(evaluate=evaluate)
    first = engine.evaluate(context, snapshot)
    assert first == engine.evaluate(context, snapshot)
    if selected:
        assert first.transitions[0].cause == next(event.value for event in DANGER if event in selected)
        assert states(first) == ("SCARED", "SURPRISED" if intrusion else "NORMAL")
    elif intrusion:
        assert states(first) == ("SURPRISED", "SURPRISED")
        assert first.transitions[0].cause == "ROBOT_INTRUSION"
    else:
        assert not first.transitions and states(first) == ("SOCIAL", "SOCIAL")


def test_member_danger_breaks_peer_before_spatial_intrusion():
    engine, context = formed()
    dangerous = engine.evaluate(context, scene(2, 3, 1.8, .55))
    assert states(dangerous) == ("SCARED", "NORMAL")
    assert shared(dangerous) == ("SOCIAL_SPACE_BROKEN",)
    assert dangerous.pair_evaluation.shared_events[0].triggered_by_ids == (1,)
    assert dangerous.transitions[1].cause == "SOCIAL_SPACE_BROKEN"


def test_gaze_loss_dwell_and_reentry_cooldown():
    engine, context = formed()
    humans = list(scene(0).humans)
    humans[1] = (2, replace(humans[1][1], yaw=0))
    loss = engine.evaluate(context, scene(2, humans=humans))
    assert states(loss) == ("SOCIAL", "SOCIAL")
    before = engine.evaluate(loss.context, scene(2.299, humans=humans))
    assert not before.transitions
    broken = engine.evaluate(before.context, scene(2.3, humans=humans))
    assert states(broken) == ("NORMAL", "NORMAL")
    assert broken.pair_evaluation.shared_events[0].reason == "PEER_GEOMETRY_LOST"
    cool = engine.evaluate(broken.context, scene(3.299))
    assert cool.context.pair.ready_since_ns is None
    ready = engine.evaluate(cool.context, scene(3.3))
    again = engine.evaluate(ready.context, scene(4.3))
    assert states(again) == ("SOCIAL", "SOCIAL")
    assert again.context.pair.session == 2


def test_initial_occupancy_and_one_sided_gaze_cannot_form():
    for snapshot in (scene(0, 6), scene(0, humans=(
        scene(0).humans[0], (2, replace(scene(0).humans[1][1], yaw=0))))):
        engine = MultiAgentAutomaton()
        first = engine.evaluate(engine.initial_context(), snapshot)
        second = engine.evaluate(first.context, replace(snapshot, sim_time_ns=2 * SECOND))
        assert not second.context.pair.active
        assert "SOCIAL" not in states(second)


def test_swept_crossing_is_detected_with_both_endpoints_outside():
    engine, context = formed()
    crossed = engine.evaluate(context, scene(2, 8))
    assert crossed.pair_evaluation.geometry.space_distance > 1
    assert crossed.pair_evaluation.geometry.swept_distance == 0
    assert "ROBOT_INTRUSION" in shared(crossed)


def test_duplicate_conflict_rollback_and_order_independence():
    engine, context = formed()
    duplicate = engine.evaluate(context, scene(1))
    assert duplicate.duplicate and not duplicate.transitions
    with pytest.raises(ValueError, match="conflicting"):
        engine.evaluate(context, scene(1, 4))
    rollback = engine.evaluate(context, scene(.5))
    assert states(rollback) == ("NORMAL", "NORMAL")
    assert rollback.clock_reset and rollback.context.epoch == 1
    assert rollback.context.pair == PairMemory()
    assert all(t.cause == "TIME_RESET" for t in rollback.transitions)
    ordered = engine.evaluate(context, scene(2, 5.21))
    reversed_step = engine.evaluate(context, scene(2, 5.21, humans=tuple(reversed(scene(0).humans))))
    assert ordered == reversed_step
    # Evaluation produces candidates without changing the committed context.
    assert context.pair.active and all(a.context.state is MultiState.SOCIAL for a in context.agents)


def test_local_mode_is_exactly_two_v1_evaluators():
    engine = MultiAgentAutomaton(shared_events_enabled=False)
    context = engine.initial_context()
    v1 = SocialAutomaton()
    extractor = EventExtractor()
    reference = {agent_id: (v1.initial_context(), None) for agent_id in (1, 2)}
    for snapshot in (scene(0), scene(.5, 4.0, vx=.15), scene(1, 4.1, vx=.8), scene(2, 2, vx=-.3), scene(6, 1, vx=-.3)):
        step = engine.evaluate(context, snapshot)
        assert step.pair_evaluation is None
        for agent, (agent_id, human) in zip(step.context.agents, snapshot.humans):
            previous, memory = reference[agent_id]
            events = extractor.evaluate(MotionSnapshot(snapshot.sim_time_ns, snapshot.robot, human), memory)
            expected = v1.step(previous, events.event_snapshot)
            assert agent.context.as_v1() == expected.context
            reference[agent_id] = (expected.context, events.memory)
        context = step.context


@pytest.mark.parametrize("start,end,distance", [((0, 0), (2, 0), 1), ((0, 0), (0, 0), math.sqrt(2))])
def test_finite_segment_distance(start, end, distance):
    assert point_segment_distance((1, 1), start, end) == pytest.approx(distance)
    assert point_segment_distance((3, 0), (0, 0), (2, 0)) == 1
    assert segment_distance((0, 0), (2, 0), (1, -1), (1, 1)) == 0
    assert segment_distance((0, 0), (2, 0), (3, 0), (4, 0)) == 1


def test_gaze_stationarity_and_space_schmitt_boundaries():
    config = PeerThresholds()
    initial = evaluate_geometry(scene(0), PairMemory(), config)
    assert initial.gaze and initial.stationary
    humans = tuple((i, replace(h, yaw=h.yaw + math.radians(30), vx=.07)) for i, h in scene(0).humans)
    held = evaluate_geometry(scene(1, humans=humans), PairMemory(visible=(True, True), near=True, gaze=True, stationary=True), config)
    assert held.gaze and held.stationary
    cold = evaluate_geometry(scene(1, humans=humans), PairMemory(), config)
    assert not cold.gaze and not cold.stationary
    occupied = evaluate_geometry(scene(1, 5.21), PairMemory(), config)
    assert occupied.intrusion_active
    band = evaluate_geometry(scene(2, 5.1), PairMemory(intrusion_active=True), config)
    assert band.intrusion_active
    exit_ = evaluate_geometry(scene(3, 5), PairMemory(intrusion_active=True), config)
    assert not exit_.intrusion_active


@pytest.mark.parametrize("document", [
    {"agent_ids": [1, 1]}, {"agent_ids": [True, 2]}, {"agent_ids": [1, 2, 3]},
    {"agent_ids": ["1", 2]}, {"agent_ids": [0, 2]}, {"typo": 1},
    {"peer": {"gaze_enter_deg": 40}}, {"peer": {"formation_dwell_seconds": -1}},
    {"peer": {"gaze_enter_deg": float("nan")}}, {"peer": {"gaze_enter_deg": "25"}},
    {"behavior_profiles": {"NORMAL": {"type": 1}}},
    {"formal_social_multi": {"ros__parameters": {}}, "typo": 1},
    {"thresholds": []}, {"timing": []}, {"peer": []},
])
def test_invalid_configs_fail(document):
    with pytest.raises((ValueError, TypeError)):
        MultiConfig.from_mapping(document)


def test_v1_state_vocabulary_is_unchanged():
    assert len(FormalState) == 5
    assert "SOCIAL" not in {state.value for state in FormalState}
    with pytest.raises(ValueError):
        SceneSnapshot(0, PlanarKinematics(0, 0), ((1, PlanarKinematics(0, 0)),) * 2)
