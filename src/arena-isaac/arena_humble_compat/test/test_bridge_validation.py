import pytest

from arena_humble_compat.bridge_validation import (
    RuntimeRates,
    calculate_runtime_rates,
    character_model_mapping,
    format_ready_status,
    format_runtime_status,
    require_plain_integer,
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


def test_runtime_rates_use_latest_monotonic_report_interval():
    rates = calculate_runtime_rates(
        now_monotonic=112.0,
        metrics_start_monotonic=100.0,
        compute_count=120,
        update_count=60,
        previous_report_monotonic=108.0,
        previous_compute_count=72,
        previous_update_count=40,
    )

    assert rates == RuntimeRates(
        compute_hz=10.0,
        display_hz=5.0,
        steady_compute=12.0,
        steady_display=5.0,
    )


@pytest.mark.parametrize(
    "overrides, message",
    [
        ({"now_monotonic": 108.0}, "intervals must be positive"),
        ({"compute_count": 71}, "counters must not decrease"),
        ({"update_count": 39}, "counters must not decrease"),
    ],
)
def test_runtime_rates_reject_invalid_intervals_and_counters(
    overrides, message
):
    arguments = {
        "now_monotonic": 112.0,
        "metrics_start_monotonic": 100.0,
        "compute_count": 120,
        "update_count": 60,
        "previous_report_monotonic": 108.0,
        "previous_compute_count": 72,
        "previous_update_count": 40,
    }
    arguments.update(overrides)

    with pytest.raises(ValueError, match=message):
        calculate_runtime_rates(**arguments)


def test_runtime_status_preserves_old_fields_and_adds_steady_rates():
    status = format_runtime_status(
        compute_count=240,
        update_count=30,
        rates=RuntimeRates(20.0, 2.5, 18.125, 4.625),
        max_integration_step=0.025,
        lag=0.004,
        substep_count=7,
        states="person:5/0",
    )

    assert status == (
        "SIX_BEHAVIORS_RUNNING compute=240 updates=30 "
        "compute_hz=20.0 display_hz=2.5 "
        "steady_compute=18.125 steady_display=4.625 "
        "max_dt=0.025 lag=0.004 substeps=7 states=person:5/0"
    )
    assert status.count("compute_hz=") == 1
    assert status.count("display_hz=") == 1


def test_plain_integer_accepts_only_non_bool_int():
    assert require_plain_integer(5, "agent.id") == 5


@pytest.mark.parametrize(
    "value, field_name",
    [
        (5.7, "person.id"),
        (True, "person.behavior.type"),
        ("1", "person.behavior.configuration"),
    ],
)
def test_plain_integer_rejects_lossy_bool_and_string_values(
    value, field_name
):
    with pytest.raises(RuntimeError, match=f"{field_name} must be an integer"):
        require_plain_integer(value, field_name)
