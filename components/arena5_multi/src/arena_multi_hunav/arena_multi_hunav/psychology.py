"""Version 1 psychology extension: pure, transactional, simulation-time based.

Plugins return a proposed state; only the adapter commits it after successful
motion computation. No plugin may control robots or modify physical geometry.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
import math
from typing import Mapping, Protocol


@dataclass(frozen=True)
class Entity:
    id: int
    name: str
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    goals: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True)
class Neighbor:
    source_id: int
    target_id: int
    distance: float
    relative_vx: float
    relative_vy: float
    is_robot: bool


@dataclass(frozen=True)
class Context:
    stamp_ns: int
    dt: float
    people: tuple[Entity, ...]
    robots: tuple[Entity, ...]
    neighbors: tuple[Neighbor, ...]


@dataclass(frozen=True)
class Modifiers:
    speed_scale: float = 1.0
    social_scale: float = 1.0
    robot_scale: float = 1.0
    space_scale: float = 1.0

    def validate(self):
        values = (self.speed_scale, self.social_scale, self.robot_scale, self.space_scale)
        if not all(math.isfinite(v) and 0 <= v <= 4 for v in values) or self.space_scale < 1:
            raise ValueError("invalid psychology modifiers")


@dataclass
class MentalState:
    variables: dict[str, float] = field(default_factory=dict)
    compartments: dict[str, float] = field(default_factory=dict)


@dataclass
class Proposal:
    states: dict[int, MentalState]
    modifiers: dict[int, Modifiers]

    def validate(self, agent_ids):
        expected = set(agent_ids)
        if set(self.states) != expected or set(self.modifiers) != expected:
            raise ValueError("psychology identity set changed")
        for modifier in self.modifiers.values():
            modifier.validate()
        for state in self.states.values():
            for value in state.variables.values():
                if not math.isfinite(value):
                    raise ValueError("non-finite mental state")
            for value in state.compartments.values():
                if not math.isfinite(value) or value < 0:
                    raise ValueError("invalid compartment state")


class Psychology(Protocol):
    name: str
    version: str
    def initialize(self, config: Mapping, agent_ids: tuple[int, ...], seed: int) -> None: ...
    def step(self, context: Context, previous_state: dict[int, MentalState]) -> Proposal: ...
    def reset(self, config: Mapping, agent_ids: tuple[int, ...], seed: int) -> None: ...
    def snapshot(self) -> dict: ...
    def restore(self, snapshot: dict) -> None: ...


def prepare_step(model: Psychology, context: Context, previous_state):
    """Prepare a motion transaction without committing plugin or mental state."""
    before = copy.deepcopy(model.snapshot())
    try:
        proposal = model.step(context, copy.deepcopy(previous_state))
        proposal.validate(previous_state)
        return proposal, copy.deepcopy(model.snapshot())
    finally:
        # A future plugin can mutate its RNG/internal state and then raise or
        # return an invalid proposal. Those paths must roll back as well.
        model.restore(before)


class NoOpPsychology:
    name = "noop"
    version = "1"

    def initialize(self, config, agent_ids, seed):
        if config:
            raise ValueError("noop accepts no model parameters")
        self.agent_ids = tuple(sorted(agent_ids))
        self.seed = seed

    reset = initialize

    def step(self, context, previous_state):
        return Proposal(copy.deepcopy(previous_state), {i: Modifiers() for i in self.agent_ids})

    def snapshot(self):
        return {"model": self.name, "version": self.version, "agent_ids": list(self.agent_ids), "seed": self.seed}

    def restore(self, snapshot):
        if snapshot["model"] != self.name or snapshot["version"] != self.version:
            raise ValueError("incompatible psychology snapshot")
        self.initialize({}, tuple(snapshot["agent_ids"]), snapshot["seed"])


def make_context(stamp_ns, dt, people, robots):
    neighbors = tuple(
        Neighbor(p.id, other.id, math.hypot(other.x - p.x, other.y - p.y),
                 other.vx - p.vx, other.vy - p.vy, other.id < 0)
        for p in sorted(people, key=lambda a: a.id)
        for other in sorted((*people, *robots), key=lambda a: a.id) if p.id != other.id
    )
    return Context(stamp_ns, dt, tuple(people), tuple(robots), neighbors)


# Deliberately explicit registration: future models implement the protocol and
# register here; scenario files cannot execute arbitrary imports or code.
MODELS = {"noop": NoOpPsychology}
