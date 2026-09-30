#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ARENA_MULTI_SKIP_OVERLAY=true
source "$SCRIPT_DIR/env.sh"
unset ARENA_MULTI_SKIP_OVERLAY
"$SCRIPT_DIR/verify_baseline.py"

colcon --log-base "$ARENA_MULTI_WS/log/colcon" build \
    --base-paths "$ARENA_MULTI_WS/src" \
    --build-base "$ARENA_MULTI_WS/build" \
    --install-base "$ARENA_MULTI_WS/install" \
    --packages-select arena_multi_hunav_msgs arena_multi_hunav_core arena_isaac arena_multi_control arena_peer_costmap arena_multi_bringup arena_multi_hunav \
    --packages-ignore arena_people_msgs isaacsim_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

set +u
source "$ARENA_MULTI_WS/install/local_setup.bash"
set -u
for package in arena_multi_hunav_msgs arena_multi_hunav_core arena_isaac arena_multi_control arena_peer_costmap arena_multi_bringup arena_multi_hunav; do
    prefix="$(ros2 pkg prefix "$package")"
    case "$(readlink -f "$prefix")" in
        "$(readlink -f "$ARENA_MULTI_WS/install")"/*) ;;
        *) echo "$package resolved outside the isolated overlay: $prefix" >&2; exit 1 ;;
    esac
done
echo "MULTIROBOT_BUILD_OK overlay=$ARENA_MULTI_WS/install"
