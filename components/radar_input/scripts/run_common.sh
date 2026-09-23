#!/usr/bin/env bash
set -Eeuo pipefail

MODE="${1:-headless}"
if [[ "$MODE" != "headless" && "$MODE" != "webrtc" ]]; then
    echo "usage: $0 headless|webrtc" >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
cd "$ARENA_WS"

export GPU_ID="${GPU_ID:-0}"
export CUDA_VISIBLE_DEVICES="$GPU_ID"
export ARENA_RENDER_GPU="$GPU_ID"
export ARENA_INTERNAL_GPU=0
export ARENA_IDEAL_CHASSIS="${ARENA_IDEAL_CHASSIS:-true}"
export ARENA_PHYSICS_DT="${ARENA_PHYSICS_DT:-0.016666666666666666}"

if [[ ! -x "$ARENA_WS/install/arena_isaac/lib/arena_isaac/run_isaacsim" ]]; then
    echo "Arena is not built. Run ./scripts/build.sh first." >&2
    exit 1
fi
if [[ ! -s "$ARENA_WS/config/generated/jackal.urdf" ]]; then
    echo "Generated Jackal URDF is missing. Run ./scripts/build.sh first." >&2
    exit 1
fi
if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    echo "Foxglove Bridge is not built. Run ./scripts/build.sh first." >&2
    exit 1
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)_${MODE}_gpu${GPU_ID}"
RUN_DIR="$ARENA_WS/logs/runs/$RUN_ID"
mkdir -p "$RUN_DIR"
export ARENA_KIT_LOG="$RUN_DIR/isaac_kit.log"
export ARENA_SCREENSHOT="$RUN_DIR/webrtc_frame.png"
export ARENA_SCREENSHOT_DELAY="${ARENA_SCREENSHOT_DELAY:-25}"

PIDS=()
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    if ((${#PIDS[@]})); then
        local pid
        for pid in "${PIDS[@]}"; do
            kill -INT -- "-$pid" 2>/dev/null || true
        done
        sleep 2
        for pid in "${PIDS[@]}"; do
            kill -TERM -- "-$pid" 2>/dev/null || true
        done
        wait "${PIDS[@]}" 2>/dev/null || true
    fi
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

ISAAC_ARGS=(
    --headless true
    --livestream false
)
if [[ "$MODE" == "webrtc" ]]; then
    ISAAC_ARGS=(
        --headless true
        --livestream true
        --webrtc-ip "$ARENA_WEBRTC_IP"
        --webrtc-signal-port "$ARENA_WEBRTC_SIGNAL_PORT"
        --webrtc-media-port "$ARENA_WEBRTC_MEDIA_PORT"
    )
fi

echo "Starting Arena 5 / Isaac Sim 5.1 on host GPU $GPU_ID (internal CUDA device cuda:0)"
echo "Ideal D6 chassis: $ARENA_IDEAL_CHASSIS (physics_dt=$ARENA_PHYSICS_DT)"
if ss -H -ltn 2>/dev/null | grep -Eq "[.:]${ARENA_FOXGLOVE_PORT}[[:space:]]"; then
    echo "TCP/$ARENA_FOXGLOVE_PORT is already in use; choose another ARENA_FOXGLOVE_PORT." >&2
    exit 1
fi
start_logged isaac \
    "$ISAAC_PYTHON" "$ARENA_WS/install/arena_isaac/lib/arena_isaac/run_isaacsim" \
    "${ISAAC_ARGS[@]}"
ISAAC_PID="${PIDS[${#PIDS[@]}-1]}"

echo "Starting Foxglove Bridge on $ARENA_FOXGLOVE_ADDRESS TCP/$ARENA_FOXGLOVE_PORT"
start_logged foxglove "$SCRIPT_DIR/run_foxglove.sh"
FOXGLOVE_PID="${PIDS[${#PIDS[@]}-1]}"

FOXGLOVE_READY=0
for _ in $(seq 1 30); do
    if ! kill -0 "$FOXGLOVE_PID" 2>/dev/null; then
        echo "Foxglove Bridge exited during startup; see $RUN_DIR/foxglove.log" >&2
        exit 1
    fi
    if ss -H -ltn 2>/dev/null | grep -Eq "[.:]${ARENA_FOXGLOVE_PORT}[[:space:]]"; then
        FOXGLOVE_READY=1
        break
    fi
    sleep 1
done
if [[ "$FOXGLOVE_READY" != 1 ]]; then
    echo "Timed out waiting for Foxglove on TCP/$ARENA_FOXGLOVE_PORT." >&2
    exit 1
fi

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

MAP_YAML="$ARENA_WS/src/arena/simulation-setup/worlds/map_empty/map/map.yaml"
start_logged map_server \
    ros2 run nav2_map_server map_server \
    --ros-args -r __node:=map_server -r map:=/task_generator_node/map \
    -p use_sim_time:=true -p yaml_filename:="$MAP_YAML"
start_logged map_lifecycle \
    ros2 run nav2_lifecycle_manager lifecycle_manager \
    --ros-args -r __node:=lifecycle_manager_map \
    -p use_sim_time:=true -p autostart:=true -p "node_names:=['map_server']"

ARENA_NAV_LAUNCH="$ARENA_WS/install/arena_simulation_setup/share/arena_simulation_setup/launch/nav2.launch.py"
start_logged arena_nav2 \
    ros2 launch "$ARENA_NAV_LAUNCH" \
    robot:=jackal task_generator_node:=/task_generator_node \
    use_sim_time:=true \
    global_planner:=navfn local_planner:=dwb \
    inter_planner:=navigate_w_replanning_time amcl:=false

start_logged arena_scene \
    ros2 run arena_humble_compat scene_bridge \
    --ros-args -p urdf_path:="$ARENA_WS/config/generated/jackal.urdf" \
    -p dynamic_people:=true

SCENE_READY=0
for _ in $(seq 1 360); do
    if grep -q 'SCENE_PEDESTRIAN_DYNAMIC_OK' "$RUN_DIR/arena_scene.log" 2>/dev/null; then
        SCENE_READY=1
        break
    fi
    if ! kill -0 "$ISAAC_PID" 2>/dev/null; then
        echo "Isaac Sim exited while spawning the scene." >&2
        exit 1
    fi
    sleep 1
done
if [[ "$SCENE_READY" != 1 ]]; then
    echo "Timed out while spawning the Arena scene; see $RUN_DIR/arena_scene.log" >&2
    exit 1
fi

echo "Arena scene ready: Jackal + map_empty + dynamic pedestrian + Nav2"
echo "Foxglove: ws://$ARENA_FOXGLOVE_ADDRESS:$ARENA_FOXGLOVE_PORT"
if [[ "$MODE" == "webrtc" ]]; then
    echo "WebRTC: $ARENA_WEBRTC_IP TCP/$ARENA_WEBRTC_SIGNAL_PORT UDP/$ARENA_WEBRTC_MEDIA_PORT"
fi

if [[ "${ARENA_SMOKE_TEST:-0}" == 1 ]]; then
    set +e
    ros2 run arena_humble_compat verify_runtime >"$RUN_DIR/navigation_smoke.log" 2>&1
    VERIFY_STATUS=$?
    set -e
    if [[ "$VERIFY_STATUS" != 0 ]]; then
        echo "Navigation smoke test failed; see $RUN_DIR/navigation_smoke.log" >&2
        exit "$VERIFY_STATUS"
    fi
    grep 'SMOKE_NAVIGATION_OK' "$RUN_DIR/navigation_smoke.log"
    if [[ "$MODE" == "webrtc" ]] && ! grep -q 'rtx_ready for streaming' "$ARENA_KIT_LOG"; then
        echo "WebRTC extension did not report rtx_ready." >&2
        exit 1
    fi
    if [[ "$MODE" == "webrtc" ]]; then
        for _ in $(seq 1 30); do
            [[ -s "$ARENA_SCREENSHOT" ]] && break
            sleep 1
        done
        if [[ ! -s "$ARENA_SCREENSHOT" ]]; then
            echo "WebRTC viewport did not produce a rendered frame." >&2
            exit 1
        fi
        echo "WEBRTC_FRAME_OK $ARENA_SCREENSHOT"
    fi
    echo "Smoke test complete."
    exit 0
fi

wait "$ISAAC_PID"
