#!/usr/bin/env bash
set -Eeuo pipefail

MPC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STABLE_WS="${ARENA_STABLE_WS:-/home/lpc/workspace/arena5_ws}"
OVERLAY_SETUP="$MPC_WS/install/setup.bash"

if [[ ! -f "$OVERLAY_SETUP" ]]; then
    echo "MPC overlay is not built. Run $MPC_WS/scripts/build_mpc.sh first." >&2
    exit 1
fi

cd "$STABLE_WS"
sha256sum --check "$MPC_WS/config/stable_protected.sha256"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
export ARENA_WEBRTC_SIGNAL_PORT="${ARENA_WEBRTC_SIGNAL_PORT:-49200}"
export ARENA_WEBRTC_MEDIA_PORT="${ARENA_WEBRTC_MEDIA_PORT:-48008}"
export ARENA_FOXGLOVE_PORT="${ARENA_FOXGLOVE_PORT:-8775}"

set +u
source "$STABLE_WS/scripts/env.sh"
source "$OVERLAY_SETUP"
set -u

if [[ "$(readlink -f "$(ros2 pkg prefix arena_mpc_controller)")" != \
      "$(readlink -f "$MPC_WS/install/arena_mpc_controller")" ]]; then
    echo "arena_mpc_controller does not resolve to the isolated overlay." >&2
    exit 1
fi
if [[ "$(readlink -f "$(ros2 pkg prefix arena_bringup)")" != \
      "$(readlink -f "$STABLE_WS/install/arena_bringup")" ]]; then
    echo "arena_bringup no longer resolves to the protected underlay." >&2
    exit 1
fi

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
MPC_SCENARIO="${MPC_SCENARIO:-six_behaviors}"
case "$MPC_SCENARIO" in
    six_behaviors) MPC_LAUNCH="mpc_six_behaviors.launch.py" ;;
    empty) MPC_LAUNCH="mpc_empty.launch.py" ;;
    *)
        echo "MPC_SCENARIO must be six_behaviors or empty." >&2
        exit 2
        ;;
esac

RUN_ID="$(date +%Y%m%d_%H%M%S)_mpc_gpu${GPU_ID}"
RUN_DIR="$MPC_WS/logs/runs/$RUN_ID"
mkdir -p "$RUN_DIR/ros"
export ROS_LOG_DIR="$RUN_DIR/ros"
export ARENA_KIT_LOG="$RUN_DIR/isaac_kit.log"

{
    printf 'method=mpc\n'
    printf 'scenario=%s\n' "$MPC_SCENARIO"
    printf 'stable_workspace=%s\n' "$STABLE_WS"
    printf 'mpc_workspace=%s\n' "$MPC_WS"
    printf 'ros_domain_id=%s\n' "$ROS_DOMAIN_ID"
    printf 'gpu_id=%s\n' "$GPU_ID"
    printf 'gpu_free_mib_at_start=%s\n' "$GPU_FREE_MIB"
    printf 'webrtc_signal_port=%s\n' "$ARENA_WEBRTC_SIGNAL_PORT"
    printf 'webrtc_media_port=%s\n' "$ARENA_WEBRTC_MEDIA_PORT"
    printf 'foxglove_port=%s\n' "$ARENA_FOXGLOVE_PORT"
    printf 'mpc_commit=%s\n' "$(git -C "$MPC_WS" rev-parse HEAD)"
    nvidia-smi --id="$GPU_ID" --query-gpu=index,uuid,name,memory.total,memory.used,memory.free,utilization.gpu \
        --format=csv,noheader
} > "$RUN_DIR/runtime_manifest.txt"

echo "Starting Arena MPC on GPU $GPU_ID (${GPU_FREE_MIB} MiB free), ROS domain $ROS_DOMAIN_ID"
echo "MPC command chain: controller_server -> velocity_smoother -> watchdog -> /cmd_vel"
echo "MPC scenario: $MPC_SCENARIO"
echo "Logs: $RUN_DIR"

exec ros2 launch arena_mpc_bringup "$MPC_LAUNCH" \
    headless:=true \
    livestream:="${LIVESTREAM:-true}" \
    webrtc_ip:="$ARENA_WEBRTC_IP" \
    webrtc_signal_port:="$ARENA_WEBRTC_SIGNAL_PORT" \
    webrtc_media_port:="$ARENA_WEBRTC_MEDIA_PORT" \
    foxglove:="${FOXGLOVE:-true}" \
    foxglove_address:="$ARENA_FOXGLOVE_ADDRESS" \
    foxglove_port:="$ARENA_FOXGLOVE_PORT" \
    "$@"
