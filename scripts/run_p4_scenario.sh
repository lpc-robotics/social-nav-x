#!/usr/bin/env bash
set -Eeuo pipefail

MPC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STABLE_WS="${ARENA_STABLE_WS:-/home/lpc/workspace/arena5_ws}"
SCENARIO="${1:?usage: run_p4_scenario.sh SCENARIO GOAL_DX GOAL_DY EXPECTED_AGENTS [INTERACTION_DISTANCE]}"
GOAL_DX="${2:?goal dx is required}"
GOAL_DY="${3:?goal dy is required}"
EXPECTED_AGENTS="${4:?expected agent count is required}"
INTERACTION_DISTANCE="${5:-1.5}"
DOMAIN="${P4_DOMAIN:?set P4_DOMAIN to a private ROS domain}"
GPU="${GPU_ID:-0}"
REPETITION="${P4_REPETITION:-1}"
CONFIG="${P4_CONFIG:-$MPC_WS/install/arena_mpc_bringup/share/arena_mpc_bringup/config/hunav/p4_${SCENARIO}.yaml}"
OUTPUT="$MPC_WS/evidence/p4/${SCENARIO}_run${REPETITION}.json"
LAUNCH_LOG="$MPC_WS/evidence/p4/${SCENARIO}_run${REPETITION}_launch.log"

if [[ ! -f "$CONFIG" ]]; then
    echo "P4 HuNav config is missing: $CONFIG" >&2
    exit 1
fi
mkdir -p "$MPC_WS/evidence/p4"

CONFIG_REALPATH="$(realpath "$CONFIG")"
CONFIG_SHA256="$(sha256sum "$CONFIG" | awk '{print $1}')"
CONTROLLER_SHA256="$(sha256sum "$MPC_WS/src/arena_mpc_controller/src/mpc_controller.cpp" | awk '{print $1}')"
PROBE_SHA256="$(sha256sum "$MPC_WS/tools/p4_human_probe.py" | awk '{print $1}')"
LAUNCH_SHA256="$(sha256sum "$MPC_WS/src/arena_mpc_bringup/launch/mpc_six_behaviors.launch.py" | awk '{print $1}')"
NAV2_OVERRIDES_SHA256="$(sha256sum "$MPC_WS/src/arena_mpc_bringup/config/nav2_overrides.yaml" | awk '{print $1}')"
CONTROLLER_CONFIG_SHA256="$(sha256sum "$MPC_WS/src/arena_mpc_bringup/config/controller_model.yaml" | awk '{print $1}')"
GIT_HEAD="$(git -C "$MPC_WS" rev-parse HEAD)"
GPU_SNAPSHOT="$(nvidia-smi --query-gpu=index,uuid,memory.total,memory.used,memory.free --format=csv,noheader,nounits | sed -n "$((GPU + 1))p")"
if [[ -z "$GPU_SNAPSHOT" ]]; then
    echo "GPU index $GPU was not present in nvidia-smi output" >&2
    exit 1
fi

export ROS_DOMAIN_ID="$DOMAIN"
export GPU_ID="$GPU"
export MPC_SCENARIO=six_behaviors
export LIVESTREAM=false
export FOXGLOVE=false
export ARENA_WEBRTC_SIGNAL_PORT="$((49500 + DOMAIN))"
export ARENA_WEBRTC_MEDIA_PORT="$((48500 + DOMAIN))"
export ARENA_FOXGLOVE_PORT="$((9100 + DOMAIN))"

setsid "$MPC_WS/scripts/run_mpc.sh" agent_config:="$CONFIG" >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
        for _ in $(seq 1 100); do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 0.2
        done
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
        fi
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

set +u
source "$STABLE_WS/scripts/env.sh" >/dev/null
source "$MPC_WS/install/setup.bash"
set -u

set +e
PROBE_ARGS=(
    "$SCENARIO"
    --goal-dx "$GOAL_DX"
    --goal-dy "$GOAL_DY"
    --expected-agents "$EXPECTED_AGENTS"
    --interaction-distance "$INTERACTION_DISTANCE"
    --timeout "${P4_TIMEOUT:-240}"
    --output "$OUTPUT"
    --ros-domain-id "$DOMAIN"
    --gpu-index "$GPU"
    --gpu-snapshot "$GPU_SNAPSHOT"
    --config-path "$CONFIG_REALPATH"
    --config-sha256 "$CONFIG_SHA256"
    --controller-sha256 "$CONTROLLER_SHA256"
    --probe-sha256 "$PROBE_SHA256"
    --launch-sha256 "$LAUNCH_SHA256"
    --nav2-overrides-sha256 "$NAV2_OVERRIDES_SHA256"
    --controller-config-sha256 "$CONTROLLER_CONFIG_SHA256"
    --git-head "$GIT_HEAD"
    --launch-log "$LAUNCH_LOG"
    --webrtc-signal-port "$ARENA_WEBRTC_SIGNAL_PORT"
    --webrtc-media-port "$ARENA_WEBRTC_MEDIA_PORT"
    --foxglove-port "$ARENA_FOXGLOVE_PORT"
)
if [[ "${P4_INJECT_ID_CHANGE:-false}" == "true" ]]; then
    PROBE_ARGS+=(--inject-id-change)
fi
if [[ "${P4_BACKLOG_DURATION:-0}" != "0" ]]; then
    PROBE_ARGS+=(
        --backlog-duration "$P4_BACKLOG_DURATION"
        --backlog-delay "${P4_BACKLOG_DELAY:-0.25}"
    )
fi
if [[ -n "${P4_REQUIRE_STOP_AGENT:-}" ]]; then
    PROBE_ARGS+=(--require-stop-agent "$P4_REQUIRE_STOP_AGENT")
fi
if [[ -n "${P4_REQUIRE_TURN_AGENT:-}" ]]; then
    PROBE_ARGS+=(
        --require-turn-agent "$P4_REQUIRE_TURN_AGENT"
        --turn-threshold "${P4_TURN_THRESHOLD:-0.3}"
    )
fi
if [[ -n "${P4_REQUIRE_INTERACTION_AGENTS:-}" ]]; then
    IFS=',' read -ra REQUIRED_INTERACTION_IDS <<< "$P4_REQUIRE_INTERACTION_AGENTS"
    for AGENT_ID in "${REQUIRED_INTERACTION_IDS[@]}"; do
        PROBE_ARGS+=(--require-interaction-agent "$AGENT_ID")
    done
fi
python "$MPC_WS/tools/p4_human_probe.py" "${PROBE_ARGS[@]}"
PROBE_STATUS=$?
set -e

cleanup
trap - EXIT INT TERM
exit "$PROBE_STATUS"
