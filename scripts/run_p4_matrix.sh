#!/usr/bin/env bash
set -Eeuo pipefail

MPC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GPU="${GPU_ID:-2}"
REPETITIONS="${P4_REPETITIONS:-5}"
DOMAIN_BASE="${P4_DOMAIN_BASE:-120}"
START_REPETITION="${P4_START_REPETITION:-1}"
START_SCENARIO="${P4_START_SCENARIO:-crossing}"
STABLE_SIX="/home/lpc/workspace/arena5_ws/install/arena_bringup/share/arena_bringup/configs/hunav_agents/isaac_six_behaviors_warehouse.yaml"
SCENARIOS=(crossing head_on same_direction multi_crossing stop_turn id_change backlog six_behaviors)

run_case() {
    local scenario="$1"
    local repetition="$2"
    local domain="$3"
    local -a environment=(
        "P4_DOMAIN=$domain"
        "P4_REPETITION=$repetition"
        "GPU_ID=$GPU"
    )
    local -a arguments

    case "$scenario" in
        crossing)
            environment+=("P4_REQUIRE_INTERACTION_AGENTS=101")
            arguments=(crossing 5 0 6 1.5)
            ;;
        head_on)
            environment+=("P4_REQUIRE_INTERACTION_AGENTS=101")
            arguments=(head_on 5 0 6 1.5)
            ;;
        same_direction)
            environment+=("P4_REQUIRE_INTERACTION_AGENTS=101")
            arguments=(same_direction 5 0 6 1.5)
            ;;
        multi_crossing)
            environment+=("P4_REQUIRE_INTERACTION_AGENTS=101,102")
            arguments=(multi_crossing 5 0 6 1.5)
            ;;
        stop_turn)
            environment+=(
                "P4_REQUIRE_STOP_AGENT=103"
                "P4_REQUIRE_TURN_AGENT=104"
                "P4_REQUIRE_INTERACTION_AGENTS=103,104"
            )
            arguments=(stop_turn 5 0 6 1.5)
            ;;
        id_change)
            environment+=(
                "P4_INJECT_ID_CHANGE=true"
                "P4_REQUIRE_INTERACTION_AGENTS=101"
            )
            arguments=(id_change 5 0 6 1.5)
            ;;
        backlog)
            environment+=(
                "P4_BACKLOG_DURATION=2.0"
                "P4_BACKLOG_DELAY=0.25"
                "P4_REQUIRE_INTERACTION_AGENTS=101"
            )
            arguments=(backlog 5 0 6 1.5)
            ;;
        six_behaviors)
            environment+=(
                "P4_CONFIG=$STABLE_SIX"
                "P4_REQUIRE_INTERACTION_AGENTS=6"
            )
            # Threatening agent 6 holds roughly 1.2--1.4 m in front of the
            # robot. A 1.2 m goal coincides with that moving hold point; 0.6 m
            # still enters the interaction gate without placing the goal on it.
            arguments=(six_behaviors 0.6 0 6 1.5)
            ;;
        *)
            echo "unknown P4 scenario: $scenario" >&2
            return 2
            ;;
    esac

    echo "P4 formal run: scenario=$scenario repetition=$repetition domain=$domain gpu=$GPU"
    env "${environment[@]}" "$MPC_WS/scripts/run_p4_scenario.sh" "${arguments[@]}"
}

start_index=-1
for index in "${!SCENARIOS[@]}"; do
    if [[ "${SCENARIOS[$index]}" == "$START_SCENARIO" ]]; then
        start_index="$index"
        break
    fi
done
if ((start_index < 0)); then
    echo "unknown P4_START_SCENARIO: $START_SCENARIO" >&2
    exit 2
fi

for repetition in $(seq "$START_REPETITION" "$REPETITIONS"); do
    for index in "${!SCENARIOS[@]}"; do
        if ((repetition == START_REPETITION && index < start_index)); then
            continue
        fi
        run_case "${SCENARIOS[$index]}" "$repetition" "$((DOMAIN_BASE + repetition * 10 + index))"
    done
done
