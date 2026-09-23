#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$ARENA_WS"
# Build against the independent ROS environment, not Isaac's setuptools.
export PYTHONPATH="${PYTHONPATH//$ISAAC_SITE:/}"
colcon build --packages-select arena_isaac arena_simulation_setup
cmake -S tools/costmap_harness -B build/costmap_harness \
    -DPython3_EXECUTABLE="$ARENA_ROS_ENV/bin/python"
cmake --build build/costmap_harness -j 2
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$ARENA_WS/src/arena-isaac/arena_isaac:$PYTHONPATH" \
python -m pytest src/arena-isaac/arena_isaac/test/test_clearing_geometry.py -q
