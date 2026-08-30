import pytest

from arena_humble_compat.bridge_validation import (
    character_model_mapping,
    format_ready_status,
    validate_agent_definitions,
)


STRICT_DEFINITIONS = [
    (behavior_type, f"agent{behavior_type}", behavior_type, 1)
    for behavior_type in range(1, 7)
]


def test_strict_six_behavior_definitions_are_accepted():
    validate_agent_definitions(STRICT_DEFINITIONS, True)


def test_strict_mode_preserves_types_error():
    with pytest.raises(
        RuntimeError,
        match=r"six-behavior config must contain types 1\.\.6",
    ):
        validate_agent_definitions(STRICT_DEFINITIONS[:-1], True)


def test_dynamic_mode_accepts_one_agent_and_repeated_behavior_types():
    validate_agent_definitions([(1, "person", 1, 1)], False)
    validate_agent_definitions(
        [(1, "person_a", 5, 1), (2, "person_b", 5, 1)], False
    )


@pytest.mark.parametrize(
    "definitions, message",
    [
        ([], "at least one agent"),
        ([(1, "a", 1, 1), (1, "b", 2, 1)], "ids must be unique"),
        ([(1, "a", 1, 1), (2, "a", 2, 1)], "names must be unique"),
        ([(1, "", 1, 1)], "names must be non-empty"),
        ([(1, "a", 7, 1)], "types must be in 1..6"),
    ],
)
def test_dynamic_mode_rejects_invalid_definitions(definitions, message):
    with pytest.raises(RuntimeError, match=message):
        validate_agent_definitions(definitions, False)


def test_character_models_must_match_agent_count():
    assert character_model_mapping(["a"], ["model_a"]) == {"a": "model_a"}
    with pytest.raises(RuntimeError, match="exactly one entry"):
        character_model_mapping(["a", "b"], ["model_a"])


def test_default_ready_status_is_byte_for_byte_compatible():
    status = format_ready_status(
        "SIX_BEHAVIORS_READY", 6, [1, 2, 3, 4, 5, 6], 40.0, 0.025
    )
    assert status == (
        "SIX_BEHAVIORS_READY pedestrians=6 behavior_types=1,2,3,4,5,6 "
        "compute_rate=40.0 max_dt=0.025 robot_state=odom "
        "hunav=/compute_agents isaac=/isaac/UpdatePedestrians"
    )


def test_dynamic_ready_status_uses_configured_marker_and_count():
    status = format_ready_status(
        "FORMAL_SOCIAL_BRIDGE_READY", 1, [5], 40.0, 0.025
    )
    assert status.startswith(
        "FORMAL_SOCIAL_BRIDGE_READY pedestrians=1 behavior_types=5 "
    )
