#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh" >/dev/null
"$SCRIPT_DIR/verify_baseline.py"
"$SCRIPT_DIR/validate_config.py"
"$SCRIPT_DIR/validate_hunav_config.py"
python -m py_compile \
    "$ARENA_MULTI_WS"/scripts/*.py \
    "$ARENA_MULTI_WS"/src/arena_multi_control/arena_multi_control/*.py \
    "$ARENA_MULTI_WS"/src/arena_multi_bringup/arena_multi_bringup/*.py \
    "$ARENA_MULTI_WS"/src/arena_multi_bringup/launch/*.py \
    "$ARENA_MULTI_WS"/src/arena_multi_hunav/arena_multi_hunav/*.py
PYTHONPATH="$ARENA_MULTI_WS/src/arena_multi_control:$ARENA_MULTI_WS/src/arena_multi_hunav${PYTHONPATH:+:$PYTHONPATH}" \
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q "$ARENA_MULTI_WS/src/arena_multi_control/test" "$ARENA_MULTI_WS/src/arena_multi_hunav/test"
"$ARENA_MULTI_WS/build/arena_multi_hunav_core/test_core"
python "$ARENA_MULTI_WS/scripts/verify_multi_sfm_vendor.py"
ros2 launch arena_multi_bringup multirobot.launch.py --show-args >/dev/null
echo "MULTIROBOT_OFFLINE_VALIDATION_OK"
