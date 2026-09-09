"""Snapshot/synchronize/evaluate protocol composing unchanged V1 evaluators."""

from dataclasses import dataclass, replace

from ..automaton import SocialAutomaton
from ..event_extractor import EventExtractor
from ..model import EventMemory, MotionSnapshot, SocialEvent
from .config import MultiConfig
from .coordinator import evaluate_pair
from .model import (AgentMemory, MultiAgentAutomatonContext, MultiContext, MultiState,
                    MultiStep, MultiTransition, PairMemory, PeerEvent, SceneSnapshot)


DANGER = (SocialEvent.PERSONAL_SPACE_VIOLATION, SocialEvent.TTC_LOW, SocialEvent.ROBOT_FAST_APPROACH)


@dataclass(frozen=True)
class SocialRule:
    rule_id: str
    guard: str
    destination: MultiState
    cause: str


SOCIAL_RULES = tuple(SocialRule(f"social.{event.value.lower()}", event.value,
                              MultiState.SCARED, event.value) for event in DANGER) + (
    SocialRule("social.intrusion", "ROBOT_INTRUSION", MultiState.SURPRISED, "ROBOT_INTRUSION"),
    SocialRule("social.broken", "SOCIAL_SPACE_BROKEN", MultiState.NORMAL, "SOCIAL_SPACE_BROKEN"),
)


class MultiAgentAutomaton:
    def __init__(self, config: MultiConfig | None = None, *, shared_events_enabled=True):
        self.config = config or MultiConfig()
        self.shared_events_enabled = shared_events_enabled
        self.extractor = EventExtractor(self.config.thresholds)
        self.v1 = SocialAutomaton(self.config.timing)

    def initial_context(self):
        return MultiContext(tuple(AgentMemory(agent_id) for agent_id in self.config.agent_ids))

    def evaluate(self, context: MultiContext, snapshot: SceneSnapshot) -> MultiStep:
        ids = tuple(agent_id for agent_id, _ in snapshot.humans)
        if ids != self.config.agent_ids or tuple(agent.agent_id for agent in context.agents) != ids:
            raise ValueError("scene/context human IDs differ from configured pair")
        if not self.shared_events_enabled and (context.pair.active or any(
            agent.context.state is MultiState.SOCIAL for agent in context.agents
        )):
            raise ValueError("shared event mode cannot be changed on a live SOCIAL context")
        if context.last_snapshot and snapshot.sim_time_ns == context.last_snapshot.sim_time_ns:
            if snapshot != context.last_snapshot:
                raise ValueError("conflicting snapshot at the same simulation timestamp")
            return MultiStep(context, (), None, duplicate=True)
        stamp = snapshot.sim_time_ns
        evaluations = tuple((agent.agent_id, self.extractor.evaluate(
            MotionSnapshot(stamp, snapshot.robot, human), agent.robot_events
        )) for agent, (_, human) in zip(context.agents, snapshot.humans))
        events = tuple((agent_id, result.event_snapshot) for agent_id, result in evaluations)
        if context.last_snapshot and stamp < context.last_snapshot.sim_time_ns:
            agents = tuple(AgentMemory(agent.agent_id,
                MultiAgentAutomatonContext(last_stamp_ns=stamp,
                    last_transition_stamp_ns=stamp if agent.context.state is not MultiState.NORMAL else None),
                EventMemory(last_stamp_ns=stamp)) for agent in context.agents)
            transitions = tuple(MultiTransition(agent.agent_id, agent.context.state, MultiState.NORMAL,
                                "TIME_RESET", "clock.reset") for agent in context.agents
                                if agent.context.state is not MultiState.NORMAL)
            return MultiStep(MultiContext(agents, PairMemory(), context.epoch + 1, snapshot),
                             events, None, transitions, clock_reset=True)
        dangers = tuple(agent_id for agent_id, event in events if any(item in event.events for item in DANGER))
        eligible = all(agent.context.state in (MultiState.NORMAL, MultiState.ATTENTION)
            and (agent.context.cooldown_until_ns is None or stamp >= agent.context.cooldown_until_ns)
            and not set(DANGER + (SocialEvent.SUDDEN_NEAR, SocialEvent.ROBOT_LOST)).intersection(event.events)
            for agent, (_, event) in zip(context.agents, events))
        pair = evaluate_pair(snapshot, context.pair, self.config.peer, eligible=eligible,
                             danger_ids=dangers, epoch=context.epoch) if self.shared_events_enabled else None
        shared = pair.shared_events if pair else ()
        shared_names = {event.name.value for event in shared}
        shared_ids = tuple(event.event_id for event in shared)
        agents, transitions = [], []
        for agent, (_, evaluation) in zip(context.agents, evaluations):
            event = evaluation.event_snapshot
            old = agent.context
            rule = None
            if old.state is MultiState.SOCIAL:
                active = {item.value for item in event.events} | shared_names
                rule = next((rule for rule in SOCIAL_RULES if rule.guard in active), None)
                next_context = replace(old, last_stamp_ns=stamp,
                    safe_since_ns=None if agent.agent_id in dangers else
                    (old.safe_since_ns if old.safe_since_ns is not None else stamp))
            elif PeerEvent.SOCIAL_SPACE_FORMED.value in shared_names:
                rule = SocialRule("social.form", "SOCIAL_SPACE_FORMED", MultiState.SOCIAL, "SOCIAL_SPACE_FORMED")
                next_context = old
            else:
                result = self.v1.step(old.as_v1(), event)
                next_context = MultiAgentAutomatonContext.from_v1(result.context)
                if result.transition:
                    transition = result.transition
                    transitions.append(MultiTransition(agent.agent_id, old.state, next_context.state,
                        transition.cause.value,
                        f"v1.{old.state.value.lower()}.{transition.cause.value.lower()}"))
            if rule:
                next_context = MultiAgentAutomatonContext(
                    state=rule.destination, state_entered_ns=stamp, last_stamp_ns=stamp,
                    last_transition_stamp_ns=stamp,
                    safe_since_ns=None if agent.agent_id in dangers else stamp,
                    cooldown_until_ns=stamp + self.config.timing.reentry_cooldown_ns
                        if rule.destination is MultiState.NORMAL else None)
                transitions.append(MultiTransition(agent.agent_id, old.state, rule.destination,
                                                   rule.cause, rule.rule_id, shared_ids))
            agents.append(AgentMemory(agent.agent_id, next_context, evaluation.memory))
        next_context = MultiContext(tuple(agents), pair.memory if pair else context.pair,
                                    context.epoch, snapshot)
        social_count = sum(agent.context.state is MultiState.SOCIAL for agent in agents)
        if social_count not in (0, 2) or next_context.pair.active != (social_count == 2):
            raise AssertionError("pair/social synchronization invariant violated")
        return MultiStep(next_context, events, pair, tuple(transitions))
