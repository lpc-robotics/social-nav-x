#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
cd "$ARENA_WS"

DEFAULT_CASES=(
    "0.2,0.0"
    "0.5,0.0"
    "0.8,0.0"
    "0.0,0.2"
    "0.0,-0.2"
    "0.0,0.4"
    "0.0,-0.4"
    "0.0,0.8"
    "0.0,-0.8"
    "0.0,1.2"
    "0.3,0.4"
    "0.3,-0.4"
    "0.3,0.8"
    "0.3,-0.8"
    "0.6,0.4"
    "0.6,-0.4"
    "0.6,0.8"
    "0.6,-0.8"
    "0.45,0.6"
    "0.45,-0.6"
    "0.2,0.6"
    "0.7,-0.6"
)

if (($# > 0)); then
    OUTPUT_DIR="$1"
    shift
else
    OUTPUT_DIR="$ARENA_WS/logs/chassis_control/$(date +%Y%m%d_%H%M%S)_ideal_d6_isolated"
fi

if (($# > 0)); then
    CASES=("$@")
else
    CASES=("${DEFAULT_CASES[@]}")
fi

mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd)"

LAUNCHER_PID=""
stop_launcher() {
    if [[ -n "$LAUNCHER_PID" ]] && kill -0 "$LAUNCHER_PID" 2>/dev/null; then
        # A non-interactive background bash inherits SIGINT as ignored; TERM
        # reliably invokes run_ideal_chassis_test.sh's cleanup trap.
        kill -TERM "$LAUNCHER_PID" 2>/dev/null || true
        wait "$LAUNCHER_PID" 2>/dev/null || true
    fi
    LAUNCHER_PID=""
}
trap stop_launcher EXIT INT TERM

for index in "${!CASES[@]}"; do
    case_number=$((index + 1))
    case_value="${CASES[$index]}"
    printf -v case_tag 'case_%02d' "$case_number"
    launcher_log="$OUTPUT_DIR/${case_tag}_launcher.log"

    echo "ISOLATED_CASE_START $case_number/${#CASES[@]} cmd=$case_value"
    setsid "$SCRIPT_DIR/run_ideal_chassis_test.sh" >"$launcher_log" 2>&1 &
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
        echo "Timed out waiting for isolated D6 case; see $launcher_log" >&2
        exit 1
    fi

    python3 "$SCRIPT_DIR/ideal_chassis_matrix.py" \
        --output "$OUTPUT_DIR" \
        --case="$case_value" \
        --append

    stop_launcher

    # Do not start the next world until the previous Isaac ROS service is gone.
    for _ in $(seq 1 60); do
        if ! ros2 service list --no-daemon 2>/dev/null | grep -qx '/isaac/SpawnUrdf'; then
            break
        fi
        sleep 1
    done
    sleep 1
    echo "ISOLATED_CASE_DONE $case_number/${#CASES[@]} cmd=$case_value"
done

trap - EXIT INT TERM
echo "ISOLATED_MATRIX_COMPLETE output=$OUTPUT_DIR cases=${#CASES[@]}"
