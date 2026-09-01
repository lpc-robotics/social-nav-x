#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BASE_WS="${ARENA_BASE_WS:-/home/lpc/workspace/arena5_ws}"
OVERLAY_ROOT="${FORMAL_OVERLAY_ROOT:-$FEATURE_ROOT/.colcon}"
SCENARIO="${1:-safe}"

case "$SCENARIO" in
    safe) DEFAULT_HOLD_SECONDS=8.0 ;;
    sudden|fast) DEFAULT_HOLD_SECONDS=2.0 ;;
    *)
        echo "Scenario must be safe, sudden, or fast: $SCENARIO" >&2
        exit 2
        ;;
esac
HOLD_SECONDS="${2:-$DEFAULT_HOLD_SECONDS}"
if (( $# > 2 )); then
    echo "Usage: $0 [safe|sudden|fast] [hold_seconds]" >&2
    exit 2
fi
if ! awk -v value="$HOLD_SECONDS" \
    'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value <= 60.0) }'; then
    echo "hold_seconds must be a number in [0, 60]: $HOLD_SECONDS" >&2
    exit 2
fi

if [[ ! -f "$BASE_WS/scripts/env.sh" ]]; then
    echo "Arena underlay is not ready: $BASE_WS" >&2
    exit 1
fi
if [[ ! -f "$OVERLAY_ROOT/install/local_setup.bash" ]]; then
    echo "Build the formal overlay first: $SCRIPT_DIR/build_formal_overlay.sh" >&2
    exit 1
fi

source "$BASE_WS/scripts/env.sh"
set +u
source "$OVERLAY_ROOT/install/local_setup.bash"
set -u

VISUAL_ID="$(date +%Y%m%d_%H%M%S_%N)_${SCENARIO}_pid${BASHPID}"
VISUAL_DIR="$FEATURE_ROOT/logs/formal_visual/$VISUAL_ID"
mkdir -p "$VISUAL_DIR"
{
    printf 'visual_id=%s\n' "$VISUAL_ID"
    printf 'git_commit=%s\n' "$(git -C "$FEATURE_ROOT" rev-parse HEAD)"
    printf 'scenario=%s\n' "$SCENARIO"
    printf 'hold_seconds=%s\n' "$HOLD_SECONDS"
    printf 'ros_domain_id=%s\n' "$ROS_DOMAIN_ID"
    printf 'overlay_root=%s\n' "$OVERLAY_ROOT"
} > "$VISUAL_DIR/run_manifest.txt"
exec > >(tee "$VISUAL_DIR/visual.log") 2>&1

echo "Visual log directory: $VISUAL_DIR"
echo "Visual scenario: $SCENARIO"
echo "Target-state observation window: up to $HOLD_SECONDS wall-clock seconds"
echo "Watch the existing WebRTC stream while this verifier drives /cmd_vel."
exec ros2 run formal_social_behavior verify_formal_social_scenario \
    --ros-args \
    -p scenario:="$SCENARIO" \
    -p round:=1 \
    -p visual_mode:=true \
    -p visual_hold_seconds:="$HOLD_SECONDS"
