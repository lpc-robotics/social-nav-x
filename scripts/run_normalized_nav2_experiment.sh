#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

EXPERIMENT_ROOT="$ARENA_WS/.workspaces/laserscan-v1"
SETUP_SOURCE="$EXPERIMENT_ROOT/src/simulation-setup"
INSTALLED_MODEL="$EXPERIMENT_ROOT/install/arena_simulation_setup/share/arena_simulation_setup/entities/robots/jackal/model_params.yaml"
if [[ ! -f "$INSTALLED_MODEL" ]] || ! cmp -s \
    "$SETUP_SOURCE/entities/robots/jackal/model_params.yaml" "$INSTALLED_MODEL"; then
    echo "The normalized Nav2 overlay is missing or stale. Run scripts/build_normalized_nav2_experiment.sh first." >&2
    exit 1
fi

set +u
source "$EXPERIMENT_ROOT/install/local_setup.bash"
set -u
EXPECTED_PREFIX="$EXPERIMENT_ROOT/install/arena_simulation_setup"
if [[ "$(readlink -f "$(ros2 pkg prefix arena_simulation_setup)")" != "$(readlink -f "$EXPECTED_PREFIX")" ]]; then
    echo "arena_simulation_setup did not resolve to the normalized Nav2 overlay." >&2
    exit 1
fi

export ARENA_SIX_BEHAVIORS_USE_OVERLAY=true
export ARENA_SIX_BEHAVIORS_OVERLAY_ROOT="$EXPERIMENT_ROOT"
export ARENA_RUN_LOG_ROOT="$EXPERIMENT_ROOT/log/runs"
export ARENA_DEPTH_CLEARING=false
export ARENA_NORMALIZED_SCAN=true
export ARENA_NORMALIZED_SCAN_NOISE="${ARENA_NORMALIZED_SCAN_NOISE:-configured}"
export ARENA_NORMALIZED_SCAN_SEED="${ARENA_NORMALIZED_SCAN_SEED:-0}"
export NAVIGATION=true
exec "$SCRIPT_DIR/run_six_behaviors.sh" "$@"
