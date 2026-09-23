#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
cd "$ARENA_WS"

OUTPUT_DIR="${1:-$ARENA_WS/logs/chassis_control/$(date +%Y%m%d_%H%M%S)_ideal_d6_collisions}"
mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd)"

# label | x | y | yaw | linear | angular | wall
CASES=(
    "frontal|0.70|11.50|3.141592654|0.80|0.00|left"
    "oblique|0.80|10.00|2.356194490|0.80|0.00|left"
    "combined|0.65|10.00|1.570796327|0.60|0.80|left"
)

LAUNCHER_PID=""
stop_launcher() {
    if [[ -n "$LAUNCHER_PID" ]] && kill -0 "$LAUNCHER_PID" 2>/dev/null; then
        kill -TERM "$LAUNCHER_PID" 2>/dev/null || true
        wait "$LAUNCHER_PID" 2>/dev/null || true
    fi
    LAUNCHER_PID=""
}
trap stop_launcher EXIT INT TERM

for index in "${!CASES[@]}"; do
    IFS='|' read -r label robot_x robot_y robot_yaw linear angular wall \
        <<<"${CASES[$index]}"
    case_number=$((index + 1))
    launcher_log="$OUTPUT_DIR/${label}_launcher.log"
    echo "COLLISION_CASE_START $case_number/${#CASES[@]} label=$label"

    ROBOT_X="$robot_x" ROBOT_Y="$robot_y" ROBOT_YAW="$robot_yaw" \
        setsid "$SCRIPT_DIR/run_ideal_chassis_test.sh" \
        >"$launcher_log" 2>&1 &
    LAUNCHER_PID="$!"

    ready=0
    for _ in $(seq 1 300); do
        if grep -q 'IDEAL_D6_TEST_READY' "$launcher_log" 2>/dev/null; then
            ready=1
            break
        fi
        if ! kill -0 "$LAUNCHER_PID" 2>/dev/null; then
            echo "D6 launcher exited before readiness; see $launcher_log" >&2
            exit 1
        fi
        sleep 1
    done
    if [[ "$ready" != 1 ]]; then
        echo "Timed out waiting for collision case; see $launcher_log" >&2
        exit 1
    fi

    python3 "$SCRIPT_DIR/ideal_chassis_collision.py" \
        --output "$OUTPUT_DIR" --append \
        --label "$label" --wall "$wall" \
        --linear "$linear" --angular "$angular"

    stop_launcher
    for _ in $(seq 1 60); do
        if ! ros2 service list --no-daemon 2>/dev/null | grep -qx '/isaac/SpawnUrdf'; then
            break
        fi
        sleep 1
    done
    sleep 1
    echo "COLLISION_CASE_DONE $case_number/${#CASES[@]} label=$label"
done

trap - EXIT INT TERM
echo "COLLISION_SUITE_COMPLETE output=$OUTPUT_DIR cases=${#CASES[@]}"
