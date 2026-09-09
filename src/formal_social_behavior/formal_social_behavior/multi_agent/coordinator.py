"""Bounded pair synchronization protocol. Does not own either human state."""

from dataclasses import dataclass, replace

from .config import PeerThresholds
from .model import PairEvaluation, PairMemory, PeerEvent, SceneSnapshot, SharedEvent
from .peer_events import evaluate_geometry, point_segment_distance


@dataclass(frozen=True)
class PairRule:
    rule_id: str
    source: str
    guard: str
    destination: str
    reason: str


PAIR_RULES = (
    PairRule("pair.intrusion", "ACTIVE", "intrusion_or_sweep", "INACTIVE", "ROBOT_INTRUSION"),
    PairRule("pair.danger", "ACTIVE", "member_danger", "INACTIVE", "MEMBER_DANGER"),
    PairRule("pair.geometry", "ACTIVE", "geometry_loss_timeout", "INACTIVE", "PEER_GEOMETRY_LOST"),
    PairRule("pair.form", "INACTIVE", "formation_dwell", "ACTIVE", "MUTUAL_GAZE_DWELL"),
)


def ns(seconds):
    return int(round(seconds * 1_000_000_000))


def evaluate_pair(snapshot: SceneSnapshot, memory: PairMemory, config: PeerThresholds,
                  *, eligible: bool, danger_ids: tuple[int, ...], epoch: int) -> PairEvaluation:
    stamp = snapshot.sim_time_ns
    geometry = evaluate_geometry(snapshot, memory, config)
    humans = tuple((human.x, human.y) for _, human in snapshot.humans)
    robot = (snapshot.robot.x, snapshot.robot.y)
    # After a broken interaction, its frozen space still controls exit hysteresis.
    # A fresh formation additionally checks the candidates' current space.
    current_distance = point_segment_distance(robot, *humans)
    clear = (not geometry.intrusion_active and
             current_distance >= snapshot.robot.radius + config.intrusion_exit_margin)
    cooldown_done = memory.cooldown_until_ns is None or stamp >= memory.cooldown_until_ns
    ready = geometry.valid and eligible and clear and cooldown_done and not danger_ids
    ready_since = (memory.ready_since_ns if memory.ready_since_ns is not None else stamp) if ready else None
    broken_since = ((memory.broken_since_ns if memory.broken_since_ns is not None else stamp)
                    if memory.active and not geometry.valid else None)
    guards = {
        "intrusion_or_sweep": geometry.intrusion_active or geometry.swept_distance <=
                              snapshot.robot.radius + config.intrusion_enter_margin,
        "member_danger": bool(danger_ids),
        "geometry_loss_timeout": broken_since is not None and
                                 stamp - broken_since >= ns(config.break_dwell_seconds),
        "formation_dwell": ready_since is not None and
                           stamp - ready_since >= ns(config.formation_dwell_seconds),
    }
    candidate = replace(memory, visible=geometry.visible, near=geometry.near, gaze=geometry.gaze,
                        stationary=geometry.stationary, ready_since_ns=ready_since,
                        broken_since_ns=broken_since, previous_robot=robot,
                        intrusion_active=geometry.intrusion_active)
    source = "ACTIVE" if memory.active else "INACTIVE"
    rule = next((rule for rule in PAIR_RULES if rule.source == source and guards[rule.guard]), None)
    if rule is None:
        return PairEvaluation(candidate, geometry)
    participants = tuple(agent_id for agent_id, _ in snapshot.humans)
    session = memory.session + (rule.destination == "ACTIVE")

    def event(name, reason):
        event_id = f"e{epoch}:p{participants[0]}-{participants[1]}:s{session}:t{stamp}:{name.value}"
        return SharedEvent(name, event_id, participants, reason, danger_ids)

    if rule.destination == "ACTIVE":
        candidate = replace(candidate, active=True, session=session, anchors=humans,
                            ready_since_ns=None, broken_since_ns=None, cooldown_until_ns=None,
                            intrusion_active=False)
        shared = (event(PeerEvent.SOCIAL_SPACE_FORMED, rule.reason),)
    else:
        candidate = replace(candidate, active=False, ready_since_ns=None, broken_since_ns=None,
                            cooldown_until_ns=stamp + ns(config.reentry_cooldown_seconds))
        shared = (event(PeerEvent.SOCIAL_SPACE_BROKEN, rule.reason),)
        if rule.guard == "intrusion_or_sweep":
            shared = (event(PeerEvent.ROBOT_INTRUSION, rule.reason),) + shared
    return PairEvaluation(candidate, geometry, shared)
