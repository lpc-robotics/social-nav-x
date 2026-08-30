#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BASE_WS="${ARENA_BASE_WS:-/home/lpc/workspace/arena5_ws}"
OVERLAY_ROOT="${FORMAL_OVERLAY_ROOT:-$FEATURE_ROOT/.colcon}"

if [[ ! -f "$BASE_WS/scripts/env.sh" || ! -f "$BASE_WS/install/setup.bash" ]]; then
    echo "Arena underlay is not ready: $BASE_WS" >&2
    exit 1
fi

source "$BASE_WS/scripts/env.sh"
cd "$FEATURE_ROOT"

colcon --log-base "$OVERLAY_ROOT/log" build \
    --base-paths \
        "$FEATURE_ROOT/src/arena-isaac/arena_humble_compat" \
        "$FEATURE_ROOT/src/formal_social_behavior" \
    --build-base "$OVERLAY_ROOT/build" \
    --install-base "$OVERLAY_ROOT/install" \
    --packages-select arena_humble_compat formal_social_behavior

echo "Formal social overlay: $OVERLAY_ROOT/install"
