#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

EXPERIMENT_ROOT="$ARENA_WS/.workspaces/laserscan-v1"
SOURCE_ROOT="$EXPERIMENT_ROOT/src/arena-isaac"
if [[ ! -f "$SOURCE_ROOT/arena_isaac/package.xml" ]]; then
    echo "Normalized-scan worktree is missing: $SOURCE_ROOT" >&2
    exit 1
fi
if [[ "$(git -C "$SOURCE_ROOT" branch --show-current)" != "feat/normalized-laserscan-v1" ]]; then
    echo "Unexpected normalized-scan branch in $SOURCE_ROOT" >&2
    exit 1
fi

colcon --log-base "$EXPERIMENT_ROOT/log/colcon" build \
    --base-paths \
        "$SOURCE_ROOT/arena_isaac" \
        "$SOURCE_ROOT/arena_humble_compat" \
    --build-base "$EXPERIMENT_ROOT/build" \
    --install-base "$EXPERIMENT_ROOT/install" \
    --packages-select arena_isaac arena_humble_compat \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$SOURCE_ROOT/arena_isaac:$PYTHONPATH" \
python -m pytest \
    "$SOURCE_ROOT/arena_isaac/test/test_clearing_geometry.py" \
    "$SOURCE_ROOT/arena_isaac/test/test_scan_geometry.py" \
    -q -p no:cacheprovider

echo "Normalized scan overlay: $EXPERIMENT_ROOT/install"
