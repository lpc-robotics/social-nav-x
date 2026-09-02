#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
cd "$ARENA_WS"

export ARENA_SIX_BEHAVIORS_USE_OVERLAY="${ARENA_SIX_BEHAVIORS_USE_OVERLAY:-false}"
export ARENA_SIX_BEHAVIORS_OVERLAY_ROOT="${ARENA_SIX_BEHAVIORS_OVERLAY_ROOT:-$ARENA_WS/.colcon-formal-v1}"
case "$ARENA_SIX_BEHAVIORS_USE_OVERLAY" in
    true|false) ;;
    *)
        echo "ARENA_SIX_BEHAVIORS_USE_OVERLAY must be true or false." >&2
        exit 2
        ;;
esac

if [[ "$ARENA_SIX_BEHAVIORS_USE_OVERLAY" == "true" ]]; then
    OVERLAY_SETUP="$ARENA_SIX_BEHAVIORS_OVERLAY_ROOT/install/local_setup.bash"
    if [[ ! -f "$OVERLAY_SETUP" ]]; then
        echo "The isolated six-behavior overlay is missing: $OVERLAY_SETUP" >&2
        echo "Build it with:" >&2
        echo "  FORMAL_OVERLAY_ROOT=$ARENA_SIX_BEHAVIORS_OVERLAY_ROOT scripts/build_formal_overlay.sh" >&2
        exit 1
    fi

    # env.sh establishes the shared underlay first. Load the isolated overlay
    # afterwards only when an explicit overlay comparison is requested.
    set +u
    source "$OVERLAY_SETUP"
    set -u

    ARENA_ISAAC_PREFIX="$(ros2 pkg prefix arena_isaac)"
    ARENA_COMPAT_PREFIX="$(ros2 pkg prefix arena_humble_compat)"
    EXPECTED_ARENA_ISAAC_PREFIX="$ARENA_SIX_BEHAVIORS_OVERLAY_ROOT/install/arena_isaac"
    EXPECTED_ARENA_COMPAT_PREFIX="$ARENA_SIX_BEHAVIORS_OVERLAY_ROOT/install/arena_humble_compat"
    if [[ "$(readlink -f "$ARENA_ISAAC_PREFIX")" != "$(readlink -f "$EXPECTED_ARENA_ISAAC_PREFIX")" ]]; then
        echo "arena_isaac did not resolve to the isolated overlay: $ARENA_ISAAC_PREFIX" >&2
        exit 1
    fi
    if [[ "$(readlink -f "$ARENA_COMPAT_PREFIX")" != "$(readlink -f "$EXPECTED_ARENA_COMPAT_PREFIX")" ]]; then
        echo "arena_humble_compat did not resolve to the isolated overlay: $ARENA_COMPAT_PREFIX" >&2
        exit 1
    fi

    RUNTIME_MODE="overlay"
else
    ARENA_ISAAC_PREFIX="$(ros2 pkg prefix arena_isaac)"
    ARENA_COMPAT_PREFIX="$(ros2 pkg prefix arena_humble_compat)"
    EXPECTED_ARENA_ISAAC_PREFIX="$ARENA_WS/install/arena_isaac"
    EXPECTED_ARENA_COMPAT_PREFIX="$ARENA_WS/install/arena_humble_compat"
    if [[ "$(readlink -f "$ARENA_ISAAC_PREFIX")" != "$(readlink -f "$EXPECTED_ARENA_ISAAC_PREFIX")" ]]; then
        echo "arena_isaac did not resolve to the shared activity install: $ARENA_ISAAC_PREFIX" >&2
        echo "Use a new shell without a sourced overlay, or set ARENA_SIX_BEHAVIORS_USE_OVERLAY=true." >&2
        exit 1
    fi
    if [[ "$(readlink -f "$ARENA_COMPAT_PREFIX")" != "$(readlink -f "$EXPECTED_ARENA_COMPAT_PREFIX")" ]]; then
        echo "arena_humble_compat did not resolve to the shared activity install: $ARENA_COMPAT_PREFIX" >&2
        echo "Use a new shell without a sourced overlay, or set ARENA_SIX_BEHAVIORS_USE_OVERLAY=true." >&2
        exit 1
    fi
    RUNTIME_MODE="shared"
fi

SOURCE_PERSON="$ARENA_WS/src/arena-isaac/arena_isaac/pedestrian/simulator/logic/people/person.py"
INSTALLED_PERSON="$ARENA_ISAAC_PREFIX/lib/python3.11/site-packages/pedestrian/simulator/logic/people/person.py"
INSTALLED_FRAMES="$ARENA_ISAAC_PREFIX/lib/python3.11/site-packages/pedestrian/simulator/logic/people/character_frames.py"
if [[ ! -f "$INSTALLED_FRAMES" ]] || ! cmp -s "$SOURCE_PERSON" "$INSTALLED_PERSON"; then
    echo "The selected arena_isaac install is stale or lacks the Character-frame fix: $ARENA_ISAAC_PREFIX" >&2
    if [[ "$RUNTIME_MODE" == "overlay" ]]; then
        echo "Rebuild it with:" >&2
        echo "  FORMAL_OVERLAY_ROOT=$ARENA_SIX_BEHAVIORS_OVERLAY_ROOT scripts/build_formal_overlay.sh" >&2
    else
        echo "Selectively rebuild the shared package with:" >&2
        echo "  source scripts/env.sh" >&2
        echo "  colcon build --packages-select arena_isaac --cmake-args -DCMAKE_BUILD_TYPE=Release" >&2
    fi
    exit 1
fi

if [[ "${1:-}" == "--check-runtime-only" || "${1:-}" == "--check-overlay-only" ]]; then
    if (( $# != 1 )); then
        echo "${1} does not accept additional arguments." >&2
        exit 2
    fi
    if [[ "${1}" == "--check-overlay-only" && "$ARENA_SIX_BEHAVIORS_USE_OVERLAY" != "true" ]]; then
        echo "--check-overlay-only requires ARENA_SIX_BEHAVIORS_USE_OVERLAY=true." >&2
        exit 2
    fi
    echo "SIX_BEHAVIORS_RUNTIME_OK mode=$RUNTIME_MODE arena_isaac=$ARENA_ISAAC_PREFIX compat=$ARENA_COMPAT_PREFIX person_sha256=$(sha256sum "$INSTALLED_PERSON" | awk '{print $1}')"
    exit 0
fi

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

{
    printf 'six_behavior_runtime_mode=%s\n' "$RUNTIME_MODE"
    printf 'six_behavior_overlay_enabled=%s\n' "$ARENA_SIX_BEHAVIORS_USE_OVERLAY"
    printf 'six_behavior_overlay_root=%s\n' "$ARENA_SIX_BEHAVIORS_OVERLAY_ROOT"
    printf 'arena_isaac_prefix=%s\n' "$ARENA_ISAAC_PREFIX"
    printf 'arena_humble_compat_prefix=%s\n' "$ARENA_COMPAT_PREFIX"
    printf 'person_sha256=%s\n' "$(sha256sum "$INSTALLED_PERSON" | awk '{print $1}')"
    printf 'character_forward_conversion=%s\n' 'ros_plus_x_to_isaac_minus_y'
} > "$RUN_DIR/runtime_manifest.txt"

echo "Starting HuNav six-behavior demo on host GPU $GPU_ID (Isaac internal cuda:0)"
echo "Foxglove: ws://$ARENA_FOXGLOVE_ADDRESS:$ARENA_FOXGLOVE_PORT"
echo "WebRTC: $ARENA_WEBRTC_IP TCP/$ARENA_WEBRTC_SIGNAL_PORT UDP/$ARENA_WEBRTC_MEDIA_PORT"
echo "Navigation: $NAVIGATION (map_empty + NavFn + DWB)"
echo "Ideal D6 chassis: $ARENA_IDEAL_CHASSIS (physics_dt=$ARENA_PHYSICS_DT)"
echo "Runtime package mode: $RUNTIME_MODE"
echo "arena_isaac: $ARENA_ISAAC_PREFIX"
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
