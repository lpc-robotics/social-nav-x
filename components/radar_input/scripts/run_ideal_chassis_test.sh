#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
cd "$ARENA_WS"

export GPU_ID="${GPU_ID:-3}"
export CUDA_VISIBLE_DEVICES="$GPU_ID"
export ARENA_RENDER_GPU="$GPU_ID"
export ARENA_INTERNAL_GPU=0
export ARENA_PHYSICS_DT="${ARENA_PHYSICS_DT:-0.016666666666666666}"
export ARENA_IDEAL_CHASSIS=true
ROBOT_X="${ROBOT_X:-15.0}"
ROBOT_Y="${ROBOT_Y:-11.5}"
ROBOT_YAW="${ROBOT_YAW:-0.0}"
printf -v ROBOT_X_FLOAT '%.6f' "$ROBOT_X"
printf -v ROBOT_Y_FLOAT '%.6f' "$ROBOT_Y"
printf -v ROBOT_YAW_FLOAT '%.9f' "$ROBOT_YAW"

RUN_ID="$(date +%Y%m%d_%H%M%S)_ideal_d6_gpu${GPU_ID}"
RUN_DIR="$ARENA_WS/logs/runs/$RUN_ID"
mkdir -p "$RUN_DIR"
export ARENA_KIT_LOG="$RUN_DIR/isaac_kit.log"

PIDS=()
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    local pid
    for pid in "${PIDS[@]}"; do
        kill -INT -- "-$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    wait "${PIDS[@]}" 2>/dev/null || true
    echo "Logs: $RUN_DIR"
    exit "$status"
}
trap cleanup EXIT INT TERM

start_logged() {
    local name="$1"
    shift
    setsid "$@" >"$RUN_DIR/$name.log" 2>&1 &
    PIDS+=("$!")
}

echo "Starting clean ideal-D6 chassis test on GPU $GPU_ID"
echo "physics_dt=$ARENA_PHYSICS_DT robot=($ROBOT_X_FLOAT,$ROBOT_Y_FLOAT,$ROBOT_YAW_FLOAT)"
echo "HuNav=false Nav2=false ideal_chassis=$ARENA_IDEAL_CHASSIS"

start_logged isaac \
    "$ISAAC_PYTHON" "$ARENA_WS/install/arena_isaac/lib/arena_isaac/run_isaacsim" \
    --headless true --livestream false
ISAAC_PID="${PIDS[${#PIDS[@]}-1]}"

ISAAC_READY=0
for _ in $(seq 1 240); do
    if ros2 service list --no-daemon 2>/dev/null | grep -qx '/isaac/SpawnUrdf'; then
        ISAAC_READY=1
        break
    fi
    if ! kill -0 "$ISAAC_PID" 2>/dev/null; then
        echo "Isaac Sim exited during startup; see $RUN_DIR/isaac.log" >&2
        exit 1
    fi
    sleep 1
done
if [[ "$ISAAC_READY" != 1 ]]; then
    echo "Timed out waiting for the Isaac ROS2 Bridge." >&2
    exit 1
fi

start_logged robot_state_publisher \
    ros2 run robot_state_publisher robot_state_publisher \
    "$ARENA_WS/config/generated/jackal.urdf" \
    --ros-args -p use_sim_time:=true

start_logged arena_scene \
    ros2 run arena_humble_compat scene_bridge \
    --ros-args -p urdf_path:="$ARENA_WS/config/generated/jackal.urdf" \
    -p dynamic_people:=false \
    -p robot_x:="$ROBOT_X_FLOAT" -p robot_y:="$ROBOT_Y_FLOAT" \
    -p robot_yaw:="$ROBOT_YAW_FLOAT"
SCENE_PID="${PIDS[${#PIDS[@]}-1]}"

SCENE_READY=0
for _ in $(seq 1 180); do
    if grep -q 'SCENE_ROBOT_OK' "$RUN_DIR/arena_scene.log" 2>/dev/null; then
        SCENE_READY=1
        break
    fi
    if ! kill -0 "$SCENE_PID" 2>/dev/null; then
        echo "Scene bridge exited; see $RUN_DIR/arena_scene.log" >&2
        exit 1
    fi
    sleep 1
done
if [[ "$SCENE_READY" != 1 ]]; then
    echo "Timed out while spawning the clean chassis scene." >&2
    exit 1
fi

echo "IDEAL_D6_TEST_READY run_dir=$RUN_DIR"
echo "Only the measurement process may publish /cmd_vel."
wait "$ISAAC_PID"
