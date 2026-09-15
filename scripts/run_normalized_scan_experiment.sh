#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

EXPERIMENT_ROOT="$ARENA_WS/.workspaces/laserscan-v1"
if [[ ! -f "$EXPERIMENT_ROOT/install/setup.bash" ]]; then
    echo "Normalized-scan overlay is missing. Run scripts/build_normalized_scan.sh first." >&2
    exit 1
fi

export ARENA_SIX_BEHAVIORS_USE_OVERLAY=true
export ARENA_SIX_BEHAVIORS_OVERLAY_ROOT="$EXPERIMENT_ROOT"
export ARENA_RUN_LOG_ROOT="$EXPERIMENT_ROOT/log/runs"
export ARENA_DEPTH_CLEARING=true
export ARENA_NORMALIZED_SCAN=true
export ARENA_NORMALIZED_SCAN_NOISE="${ARENA_NORMALIZED_SCAN_NOISE:-configured}"
export ARENA_NORMALIZED_SCAN_SEED="${ARENA_NORMALIZED_SCAN_SEED:-0}"

exec "$SCRIPT_DIR/run_six_behaviors.sh" "$@"
