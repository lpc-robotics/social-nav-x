#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

EXPERIMENT_ROOT="$ARENA_WS/.workspaces/laserscan-v1"
SETUP_SOURCE="$EXPERIMENT_ROOT/src/simulation-setup"
if [[ "$(git -C "$SETUP_SOURCE" branch --show-current)" != "feat/normalized-laserscan-nav2" ]]; then
    echo "Expected the isolated normalized-Nav2 simulation-setup worktree." >&2
    exit 1
fi

"$SCRIPT_DIR/build_normalized_scan.sh"

colcon --log-base "$EXPERIMENT_ROOT/log/colcon-nav2" build \
    --base-paths "$SETUP_SOURCE" \
    --build-base "$EXPERIMENT_ROOT/build" \
    --install-base "$EXPERIMENT_ROOT/install" \
    --packages-select arena_simulation_setup

set +u
source "$EXPERIMENT_ROOT/install/local_setup.bash"
set -u
EXPECTED_PREFIX="$EXPERIMENT_ROOT/install/arena_simulation_setup"
if [[ "$(readlink -f "$(ros2 pkg prefix arena_simulation_setup)")" != "$(readlink -f "$EXPECTED_PREFIX")" ]]; then
    echo "arena_simulation_setup did not resolve to the isolated overlay." >&2
    exit 1
fi
INSTALLED_MODEL="$EXPECTED_PREFIX/share/arena_simulation_setup/entities/robots/jackal/model_params.yaml"
cmp "$SETUP_SOURCE/entities/robots/jackal/model_params.yaml" "$INSTALLED_MODEL"
echo "Normalized Nav2 replacement overlay: $EXPECTED_PREFIX"
