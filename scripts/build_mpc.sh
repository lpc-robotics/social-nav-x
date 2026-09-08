#!/usr/bin/env bash
set -Eeuo pipefail

MPC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STABLE_WS="${ARENA_STABLE_WS:-/home/lpc/workspace/arena5_ws}"
CASADI_CMAKE="$MPC_WS/third_party/casadi-3.8.0-wheel/casadi/cmake"

if [[ ! -f "$STABLE_WS/scripts/env.sh" ]]; then
    echo "Stable Arena environment is missing: $STABLE_WS" >&2
    exit 1
fi
if [[ ! -f "$CASADI_CMAKE/casadi-config.cmake" ]]; then
    echo "Audited CasADi SDK is missing: $CASADI_CMAKE" >&2
    exit 1
fi

cd "$STABLE_WS"
sha256sum --check "$MPC_WS/config/stable_protected.sha256"
set +u
source "$STABLE_WS/scripts/env.sh"
set -u

cd "$MPC_WS"
colcon build \
    --base-paths src \
    --packages-up-to arena_mpc_bringup \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -Dcasadi_DIR="$CASADI_CMAKE" \
        -DCMAKE_CXX_COMPILER="$STABLE_WS/.conda/arena_ros/bin/x86_64-conda-linux-gnu-c++" \
        -DPython3_EXECUTABLE="$STABLE_WS/.conda/arena_ros/bin/python"
