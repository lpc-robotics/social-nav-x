#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$SCRIPT_DIR/verify_depth_clearing_baseline.py" --runtime-only

export ARENA_SIX_BEHAVIORS_USE_OVERLAY=false
export ARENA_DEPTH_CLEARING=true
export ARENA_NORMALIZED_SCAN=false
exec "$SCRIPT_DIR/run_six_behaviors.sh" "$@"
