VALID_BEHAVIOR_TYPES = frozenset(range(1, 7))
STRICT_SIX_BEHAVIOR_TYPES = [1, 2, 3, 4, 5, 6]


def validate_agent_definitions(agent_definitions, strict_six_behavior_demo):
    """Validate the ROS-independent fields used to construct HuNav agents."""

    definitions = list(agent_definitions)
    actual_types = sorted(item[2] for item in definitions)
    if strict_six_behavior_demo:
        if actual_types != STRICT_SIX_BEHAVIOR_TYPES:
            raise RuntimeError(
                "six-behavior config must contain types 1..6, "
                f"got {actual_types}"
            )
        if any(item[3] != 1 for item in definitions):
            raise RuntimeError(
                "all six demo agents must use deterministic custom config"
            )
        return

    if not definitions:
        raise RuntimeError(
            "HuNav agent config must contain at least one agent"
        )

    agent_ids = [item[0] for item in definitions]
    if len(set(agent_ids)) != len(agent_ids):
        raise RuntimeError(f"HuNav agent ids must be unique, got {agent_ids}")

    agent_names = [item[1] for item in definitions]
    if any(not name for name in agent_names):
        raise RuntimeError("HuNav agent names must be non-empty")
    if len(set(agent_names)) != len(agent_names):
        raise RuntimeError(
            f"HuNav agent names must be unique, got {agent_names}"
        )

    invalid_types = sorted(
        set(actual_types).difference(VALID_BEHAVIOR_TYPES)
    )
    if invalid_types:
        raise RuntimeError(
            "HuNav behavior types must be in 1..6, "
            f"got invalid types {invalid_types}"
        )


def character_model_mapping(agent_names, model_names):
    """Return the one-to-one agent/model mapping after checking its size."""

    names = list(agent_names)
    models = list(model_names)
    if len(models) != len(names):
        raise RuntimeError(
            "character_models must have exactly one entry per HuNav agent"
        )
    return dict(zip(names, models))


def format_ready_status(
    marker, agent_count, behavior_types, compute_rate, max_integration_step
):
    """Format the bridge READY line, including its compatibility marker."""

    type_list = ",".join(str(value) for value in sorted(behavior_types))
    return (
        f"{marker} pedestrians={agent_count} behavior_types={type_list} "
        f"compute_rate={compute_rate:.1f} max_dt={max_integration_step:.3f} "
        "robot_state=odom hunav=/compute_agents "
        "isaac=/isaac/UpdatePedestrians"
    )
