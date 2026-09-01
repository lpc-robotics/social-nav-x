#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BASE_WS="${ARENA_BASE_WS:-/home/lpc/workspace/arena5_ws}"
OVERLAY_ROOT="${FORMAL_OVERLAY_ROOT:-$FEATURE_ROOT/.colcon}"

if [[ ! -f "$OVERLAY_ROOT/install/local_setup.bash" ]]; then
    echo "Build the formal overlay first: $SCRIPT_DIR/build_formal_overlay.sh" >&2
    exit 1
fi

source "$BASE_WS/scripts/env.sh"
# Generated colcon setup hooks legitimately inspect unset variables.
set +u
source "$OVERLAY_ROOT/install/local_setup.bash"
set -u
cd "$FEATURE_ROOT"

# The read-only underlay currently contains an old launch_testing plugin that
# is incompatible with its newer pytest. These packages need no auto-loaded
# plugins, so disable discovery rather than altering the shared environment.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

# arena_isaac's full test suite imports Isaac Kit's `omni` modules and cannot
# run in the plain ROS Python process.  The frame conversion module is pure
# Python, so exercise its focused regression test explicitly without loading
# the Kit-dependent package tests.
ROS_PYTHON="$BASE_WS/.conda/arena_ros/bin/python"
PYTHONPATH="$FEATURE_ROOT/src/arena-isaac/arena_isaac${PYTHONPATH:+:$PYTHONPATH}" \
    "$ROS_PYTHON" -m pytest -q \
    "$FEATURE_ROOT/src/arena-isaac/arena_isaac/test/test_character_frames.py"

colcon --log-base "$OVERLAY_ROOT/test-log" test \
    --build-base "$OVERLAY_ROOT/build" \
    --install-base "$OVERLAY_ROOT/install" \
    --packages-select arena_humble_compat formal_social_behavior \
    --event-handlers console_cohesion+

colcon test-result \
    --test-result-base "$OVERLAY_ROOT/build/arena_humble_compat" \
    --verbose

colcon test-result \
    --test-result-base "$OVERLAY_ROOT/build/formal_social_behavior" \
    --verbose
