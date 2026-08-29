#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
cd "$ARENA_WS"

export GPU_ID="${GPU_ID:-0}"
export CUDA_VISIBLE_DEVICES="$GPU_ID"
export ARENA_RENDER_GPU="$GPU_ID"
export ARENA_INTERNAL_GPU=0
export NAVIGATION="${NAVIGATION:-true}"
export ARENA_IDEAL_CHASSIS="${ARENA_IDEAL_CHASSIS:-true}"
export ARENA_PHYSICS_DT="${ARENA_PHYSICS_DT:-0.016666666666666666}"

if [[ "$NAVIGATION" != "true" && "$NAVIGATION" != "false" ]]; then
    echo "NAVIGATION must be true or false." >&2
    exit 2
fi

if [[ ! -x "$ARENA_WS/install/arena_humble_compat/lib/arena_humble_compat/hunav_six_behaviors_bridge" ]]; then
    echo "Six-behavior demo is not built. Run ./scripts/build.sh first." >&2
    exit 1
fi
if [[ ! -s "$ARENA_WS/config/generated/jackal.urdf" ]]; then
    echo "Generated Jackal URDF is missing. Run ./scripts/build.sh first." >&2
    exit 1
fi
if [[ "$NAVIGATION" == "true" ]]; then
    if ! ros2 pkg prefix nav2_map_server >/dev/null 2>&1 \
        || ! ros2 pkg prefix arena_simulation_setup >/dev/null 2>&1; then
        echo "Navigation packages are not built. Run ./scripts/build.sh first." >&2
        exit 1
    fi
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)_six_behaviors_gpu${GPU_ID}"
RUN_DIR="$ARENA_WS/logs/runs/$RUN_ID"
mkdir -p "$RUN_DIR"
export ROS_LOG_DIR="$RUN_DIR/ros"
export ARENA_KIT_LOG="$RUN_DIR/isaac_kit.log"
export ARENA_SCREENSHOT="$RUN_DIR/webrtc_frame.png"
export ARENA_SCREENSHOT_DELAY="${ARENA_SCREENSHOT_DELAY:-25}"

echo "Starting HuNav six-behavior demo on host GPU $GPU_ID (Isaac internal cuda:0)"
echo "Foxglove: ws://$ARENA_FOXGLOVE_ADDRESS:$ARENA_FOXGLOVE_PORT"
echo "WebRTC: $ARENA_WEBRTC_IP TCP/$ARENA_WEBRTC_SIGNAL_PORT UDP/$ARENA_WEBRTC_MEDIA_PORT"
echo "Navigation: $NAVIGATION (map_empty + NavFn + DWB)"
echo "Ideal D6 chassis: $ARENA_IDEAL_CHASSIS (physics_dt=$ARENA_PHYSICS_DT)"
echo "Logs: $RUN_DIR"

exec ros2 launch arena_bringup isaac_six_behaviors.launch.py \
    headless:=true \
    livestream:="${LIVESTREAM:-true}" \
    webrtc_ip:="$ARENA_WEBRTC_IP" \
    webrtc_signal_port:="$ARENA_WEBRTC_SIGNAL_PORT" \
    webrtc_media_port:="$ARENA_WEBRTC_MEDIA_PORT" \
    foxglove_address:="$ARENA_FOXGLOVE_ADDRESS" \
    foxglove_port:="$ARENA_FOXGLOVE_PORT" \
    navigation:="$NAVIGATION" \
    "$@"
