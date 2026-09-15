#!/usr/bin/env bash
set -Eeuo pipefail

RELEASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STABLE_WS="${ARENA_STABLE_WS:-/home/lpc/workspace/arena5_ws}"
OVERLAY_SETUP="$RELEASE_ROOT/install/local_setup.bash"

if [[ ! -f "$OVERLAY_SETUP" ]]; then
    echo "MPC release overlay is missing: $OVERLAY_SETUP" >&2
    exit 1
fi
(cd "$STABLE_WS" && sha256sum --check "$RELEASE_ROOT/stable_protected.sha256")

set +u
source "$STABLE_WS/scripts/env.sh"
source "$OVERLAY_SETUP"
set -u

if [[ "$(readlink -f "$(ros2 pkg prefix arena_mpc_controller)")" != \
      "$(readlink -f "$RELEASE_ROOT/install")" ]]; then
    echo "arena_mpc_controller does not resolve to this immutable release." >&2
    exit 1
fi
if [[ "$(readlink -f "$(ros2 pkg prefix arena_bringup)")" != \
      "$(readlink -f "$STABLE_WS/install/arena_bringup")" ]]; then
    echo "arena_bringup does not resolve to the protected underlay." >&2
    exit 1
fi

if [[ "${1:-}" == "--check-runtime-only" ]]; then
    if (( $# != 1 )); then
        echo "--check-runtime-only does not accept additional arguments." >&2
        exit 2
    fi
    echo "MPC_RELEASE_RUNTIME_OK release=$RELEASE_ROOT overlay=$(ros2 pkg prefix arena_mpc_controller) underlay=$(ros2 pkg prefix arena_bringup)"
    exit 0
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
export ARENA_WEBRTC_SIGNAL_PORT="${ARENA_WEBRTC_SIGNAL_PORT:-49200}"
export ARENA_WEBRTC_MEDIA_PORT="${ARENA_WEBRTC_MEDIA_PORT:-48008}"
export ARENA_FOXGLOVE_PORT="${ARENA_FOXGLOVE_PORT:-8775}"
if [[ -z "${GPU_ID:-}" ]]; then
    GPU_ID="$(nvidia-smi --query-gpu=index,memory.free --format=csv,noheader,nounits | \
        awk -F, '{gsub(/ /, "", $1); gsub(/ /, "", $2); print $2, $1}' | \
        sort -nr | awk 'NR == 1 {print $2}')"
fi
if [[ -z "$GPU_ID" ]] || ! [[ "$GPU_ID" =~ ^[0-9]+$ ]]; then
    echo "Unable to select a GPU. Set GPU_ID to a valid GPU index." >&2
    exit 1
fi
GPU_FREE_MIB="$(nvidia-smi --id="$GPU_ID" --query-gpu=memory.free --format=csv,noheader,nounits | tr -d ' ')"
MPC_MIN_FREE_MIB="${MPC_MIN_FREE_MIB:-8192}"
if (( GPU_FREE_MIB < MPC_MIN_FREE_MIB )); then
    echo "GPU $GPU_ID has ${GPU_FREE_MIB} MiB free; ${MPC_MIN_FREE_MIB} MiB is required." >&2
    exit 1
fi
export GPU_ID CUDA_VISIBLE_DEVICES="$GPU_ID" ARENA_RENDER_GPU="$GPU_ID" ARENA_INTERNAL_GPU=0
export ARENA_IDEAL_CHASSIS="${ARENA_IDEAL_CHASSIS:-true}"
export ARENA_PHYSICS_DT="${ARENA_PHYSICS_DT:-0.016666666666666666}"

RUN_ID="$(date +%Y%m%d_%H%M%S)_mpc_release_gpu${GPU_ID}"
RUN_DIR="$STABLE_WS/logs/runs/$RUN_ID"
mkdir -p "$RUN_DIR/ros"
export ROS_LOG_DIR="$RUN_DIR/ros"
export ARENA_KIT_LOG="$RUN_DIR/isaac_kit.log"

echo "Starting immutable Arena MPC release on GPU $GPU_ID (${GPU_FREE_MIB} MiB free)"
echo "MPC command chain: controller_server -> velocity_smoother -> watchdog -> /cmd_vel"
echo "Release: $RELEASE_ROOT"
echo "Logs: $RUN_DIR"

exec ros2 launch arena_mpc_bringup mpc_six_behaviors.launch.py \
    headless:=true \
    livestream:="${LIVESTREAM:-true}" \
    mpc_visualization:="${MPC_VISUALIZATION:-true}" \
    webrtc_ip:="$ARENA_WEBRTC_IP" \
    webrtc_signal_port:="$ARENA_WEBRTC_SIGNAL_PORT" \
    webrtc_media_port:="$ARENA_WEBRTC_MEDIA_PORT" \
    foxglove:="${FOXGLOVE:-true}" \
    foxglove_address:="$ARENA_FOXGLOVE_ADDRESS" \
    foxglove_port:="$ARENA_FOXGLOVE_PORT" \
    "$@"
