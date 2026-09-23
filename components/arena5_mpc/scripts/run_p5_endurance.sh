#!/usr/bin/env bash
set -Eeuo pipefail

MPC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STABLE_WS="${ARENA_STABLE_WS:-/home/lpc/workspace/arena5_ws}"
GPU="${GPU_ID:-0}"
DOMAIN="${P5_DOMAIN:-190}"
DURATION="${P5_DURATION:-1800}"
OUTPUT_DIR="$MPC_WS/evidence/p5/endurance"
OUTPUT="${P5_OUTPUT:-$OUTPUT_DIR/endurance_30min.json}"
LAUNCH_LOG="${P5_LAUNCH_LOG:-$OUTPUT_DIR/endurance_30min_launch.log}"
CONFIG="$STABLE_WS/install/arena_bringup/share/arena_bringup/configs/hunav_agents/isaac_six_behaviors_warehouse.yaml"
GPU_SNAPSHOT="$(nvidia-smi --query-gpu=index,uuid,memory.total,memory.used,memory.free --format=csv,noheader,nounits | sed -n "$((GPU + 1))p")"
CONFIG_SHA256="$(sha256sum "$CONFIG" | awk '{print $1}')"
CONTROLLER_SHA256="$(sha256sum "$MPC_WS/src/arena_mpc_controller/src/mpc_controller.cpp" | awk '{print $1}')"
PROGRESS_CHECKER_SHA256="$(sha256sum "$MPC_WS/src/arena_mpc_controller/src/safety_aware_progress_checker.cpp" | awk '{print $1}')"
CONTROLLER_CONFIG_SHA256="$(sha256sum "$MPC_WS/src/arena_mpc_bringup/config/controller_model.yaml" | awk '{print $1}')"
NAV2_OVERRIDES_SHA256="$(sha256sum "$MPC_WS/src/arena_mpc_bringup/config/nav2_overrides.yaml" | awk '{print $1}')"
PROBE_SHA256="$(sha256sum "$MPC_WS/tools/p5_endurance_probe.py" | awk '{print $1}')"

mkdir -p "$OUTPUT_DIR"
export ROS_DOMAIN_ID="$DOMAIN"
export GPU_ID="$GPU"
export MPC_SCENARIO=six_behaviors
export LIVESTREAM=false
export FOXGLOVE=false
export ARENA_WEBRTC_SIGNAL_PORT="$((51500 + DOMAIN))"
export ARENA_WEBRTC_MEDIA_PORT="$((51000 + DOMAIN))"
export ARENA_FOXGLOVE_PORT="$((9500 + DOMAIN))"

setsid "$MPC_WS/scripts/run_mpc.sh" "agent_config:=$CONFIG" >"$LAUNCH_LOG" 2>&1 &
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

python "$MPC_WS/tools/p5_endurance_probe.py" \
    --duration "$DURATION" \
    --startup-timeout "${P5_STARTUP_TIMEOUT:-180}" \
    --goal-timeout "${P5_GOAL_TIMEOUT:-0}" \
    --minimum-goals "${P5_MINIMUM_GOALS:-10}" \
    --maximum-completed-goals "${P5_MAXIMUM_COMPLETED_GOALS:-0}" \
    --output "$OUTPUT" \
    --ros-domain-id "$DOMAIN" \
    --gpu-index "$GPU" \
    --gpu-snapshot "$GPU_SNAPSHOT" \
    --config-path "$CONFIG" \
    --config-sha256 "$CONFIG_SHA256" \
    --controller-sha256 "$CONTROLLER_SHA256" \
    --progress-checker-sha256 "$PROGRESS_CHECKER_SHA256" \
    --controller-config-sha256 "$CONTROLLER_CONFIG_SHA256" \
    --nav2-overrides-sha256 "$NAV2_OVERRIDES_SHA256" \
    --probe-sha256 "$PROBE_SHA256" \
    --git-head "$(git -C "$MPC_WS" rev-parse HEAD)" \
    --launch-log "$LAUNCH_LOG"

cleanup
trap - EXIT INT TERM
(cd "$STABLE_WS" && sha256sum --check "$MPC_WS/config/stable_protected.sha256")
