import json

from formal_social_behavior.multi_agent.automaton import MultiAgentAutomaton
from formal_social_behavior.multi_agent.config import MultiConfig
from formal_social_behavior.multi_agent.model_export import describe_model, model_hash
from formal_social_behavior.multi_agent.trace import replay_frames, snapshot_from_payload

from test_multi_agent_core import scene


def test_full_input_replay_and_semantic_frames_are_byte_stable():
    inputs = [scene(0), scene(.5), scene(1), scene(2, 5.21, 2.4, .15), scene(3, 3, vx=-.3), scene(.1)]
    engine = MultiAgentAutomaton()
    first = replay_frames(engine, inputs)
    second = replay_frames(engine, [snapshot_from_payload(json.loads(line)["input"]) for line in first])
    assert first == second
    assert "Infinity" not in "".join(first)
    formed = json.loads(first[2])
    assert formed["pair"]["events"] == ["PEER_NEAR", "MUTUAL_GAZE"]
    assert all(agent["peer_events"] == ["PEER_VISIBLE"] for agent in formed["agents"])
    assert len(formed["transitions"]) == 2


def test_model_description_matches_runtime_rules_and_retains_priorities():
    config = MultiConfig()
    model = describe_model(config)
    known_ids = {row["rule_id"] for rows in model["transitions"].values() for row in rows}
    frames = replay_frames(MultiAgentAutomaton(), [scene(0), scene(1), scene(2, 5.21), scene(6, 3)])
    for line in frames:
        for transition in json.loads(line)["transitions"]:
            assert transition["rule_id"] in known_ids
    for rows in model["transitions"].values():
        assert rows[-1]["guard"] == "TRUE"
        for index, row in enumerate(rows):
            assert row["priority"] == index
            assert row["effective_guard"]["exclude"] == [r["guard"] for r in rows[:index]]
    assert model["transitions"]["SOCIAL"][0]["cause"] == "PERSONAL_SPACE_VIOLATION"
    assert model_hash(config) == model_hash(MultiConfig())
    assert not model["uppaal_exported"]
