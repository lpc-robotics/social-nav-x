#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BASE_WS="${ARENA_BASE_WS:-/home/lpc/workspace/arena5_ws}"
OVERLAY_ROOT="${FORMAL_OVERLAY_ROOT:-$FEATURE_ROOT/.colcon}"
DOMAIN_BASE="${FORMAL_ACCEPTANCE_DOMAIN_BASE:-71}"

if [[ ! -f "$BASE_WS/scripts/env.sh" ]]; then
    echo "Arena underlay is not ready: $BASE_WS" >&2
    exit 1
fi
if [[ ! -f "$OVERLAY_ROOT/install/local_setup.bash" ]]; then
    echo "Build the formal overlay first: $SCRIPT_DIR/build_formal_overlay.sh" >&2
    exit 1
fi
if ! [[ "$DOMAIN_BASE" =~ ^[0-9]+$ ]] || (( DOMAIN_BASE < 0 || DOMAIN_BASE > 226 )); then
    echo "FORMAL_ACCEPTANCE_DOMAIN_BASE must be an integer in [0, 226]" >&2
    exit 2
fi

source "$BASE_WS/scripts/env.sh"
set +u
source "$OVERLAY_ROOT/install/local_setup.bash"
set -u

export GPU_ID="${GPU_ID:-3}"
export NAVIGATION=false
export ARENA_IDEAL_CHASSIS=true
export ARENA_PHYSICS_DT="${ARENA_PHYSICS_DT:-0.016666666666666666}"

MATRIX_ID="$(date +%Y%m%d_%H%M%S_%N)_pid${BASHPID}"
MATRIX_ROOT="$FEATURE_ROOT/logs/formal_acceptance/$MATRIX_ID"
mkdir -p "$MATRIX_ROOT"
exec > >(tee "$MATRIX_ROOT/matrix.log") 2>&1

echo "Formal acceptance matrix: $MATRIX_ROOT"
echo "GPU: $GPU_ID; domain base: $DOMAIN_BASE"

KEY_MANIFEST="/home/lpc/workspace/arena5_ws_archives/20260829_164103_full_workspace_pre_log_cleanup/KEY_SHA256SUMS"
PROTECTED_PATHS=(
    "$BASE_WS/config/generated/jackal.urdf"
    "$BASE_WS/scripts/run_six_behaviors.sh"
    "$BASE_WS/src/arena-isaac/arena_isaac/arena_isaac/run_isaacsim.py"
    "$BASE_WS/src/arena-isaac/arena_isaac/arena_isaac/services/SpawnUrdf.py"
    "$BASE_WS/src/arena-isaac/arena_isaac/isaac_utils/graphs/odom.py"
    "$BASE_WS/src/arena-isaac/arena_isaac/pedestrian/simulator/logic/people/person.py"
    "$BASE_WS/src/arena-isaac/arena_humble_compat/arena_humble_compat/hunav_six_behaviors_bridge.py"
    "$BASE_WS/src/arena-rosnav/arena_bringup/configs/hunav_agents/isaac_six_behaviors_warehouse.yaml"
    "$BASE_WS/src/arena-rosnav/arena_bringup/launch/isaac_six_behaviors.launch.py"
    "$BASE_WS/src/arena/simulation-setup/launch/nav2.launch.py"
)
for protected_path in "${PROTECTED_PATHS[@]}"; do
    if [[ ! -f "$protected_path" ]]; then
        echo "Protected baseline file is missing: $protected_path" >&2
        exit 1
    fi
done
if [[ ! -f "$KEY_MANIFEST" ]]; then
    echo "Protected baseline manifest is missing: $KEY_MANIFEST" >&2
    exit 1
fi
(
    cd "$BASE_WS"
    sha256sum --check "$KEY_MANIFEST"
)
sha256sum "${PROTECTED_PATHS[@]}" > "$MATRIX_ROOT/protected_before.sha256"
echo "FORMAL_ACCEPTANCE_PROTECTED_BASELINE_OK files=${#PROTECTED_PATHS[@]}"

CURRENT_PID=""

cleanup_current() {
    if [[ -z "${CURRENT_PID:-}" ]]; then
        return
    fi
    kill -INT -- "-$CURRENT_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do
        if ! kill -0 "$CURRENT_PID" 2>/dev/null; then
            wait "$CURRENT_PID" || true
            CURRENT_PID=""
            return
        fi
        sleep 1
    done
    kill -TERM -- "-$CURRENT_PID" 2>/dev/null || true
    wait "$CURRENT_PID" || true
    CURRENT_PID=""
}

trap cleanup_current EXIT

check_launch_health() {
    local launch_log="$1"
    local quiet="${2:-false}"
    if grep -Eq \
        'Traceback|process has died|formal social (compute|telemetry) failed|Isaac pedestrian update (failed|returned errors)|HuNav compute (failed|returned an invalid response)' \
        "$launch_log"; then
        if [[ "$quiet" != true ]]; then
            grep -En \
                'Traceback|process has died|formal social (compute|telemetry) failed|Isaac pedestrian update (failed|returned errors)|HuNav compute (failed|returned an invalid response)' \
                "$launch_log" >&2
        fi
        return 1
    fi
    if grep -Eiq '(^|[^[:alpha:]])(nan|infinity)([^[:alpha:]]|$)|non-finite' "$launch_log"; then
        if [[ "$quiet" != true ]]; then
            echo "NaN/Inf marker found in launch log" >&2
        fi
        return 1
    fi
}

RUNTIME_COMPUTE_COUNT=""
RUNTIME_UPDATE_COUNT=""

check_runtime_log() {
    local launch_log="$1"
    local minimum_compute_count="${2:--1}"
    local minimum_update_count="${3:--1}"
    local quiet="${4:-false}"
    local runtime_line
    local compute_count
    local update_count
    local compute_hz
    local display_hz
    local steady_compute
    local steady_display
    local max_dt
    local lag

    runtime_line="$(grep 'SIX_BEHAVIORS_RUNNING' "$launch_log" | tail -1 || true)"
    if [[ -z "$runtime_line" ]]; then
        if [[ "$quiet" != true ]]; then
            echo "No steady runtime metrics were reported" >&2
        fi
        return 1
    fi
    compute_count="$(sed -nE 's/.* compute=([0-9]+) .*/\1/p' <<<"$runtime_line")"
    update_count="$(sed -nE 's/.* updates=([0-9]+) .*/\1/p' <<<"$runtime_line")"
    compute_hz="$(sed -nE 's/.* compute_hz=([0-9]+([.][0-9]+)?) .*/\1/p' <<<"$runtime_line")"
    display_hz="$(sed -nE 's/.* display_hz=([0-9]+([.][0-9]+)?) .*/\1/p' <<<"$runtime_line")"
    steady_compute="$(sed -nE 's/.* steady_compute=([0-9]+([.][0-9]+)?) .*/\1/p' <<<"$runtime_line")"
    steady_display="$(sed -nE 's/.* steady_display=([0-9]+([.][0-9]+)?) .*/\1/p' <<<"$runtime_line")"
    max_dt="$(sed -nE 's/.* max_dt=([0-9]+([.][0-9]+)?) .*/\1/p' <<<"$runtime_line")"
    lag="$(sed -nE 's/.* lag=([0-9]+([.][0-9]+)?) .*/\1/p' <<<"$runtime_line")"
    if [[ -z "$compute_count" || -z "$update_count" || -z "$compute_hz" || -z "$display_hz" || -z "$steady_compute" || -z "$steady_display" || -z "$max_dt" || -z "$lag" ]]; then
        if [[ "$quiet" != true ]]; then
            echo "Could not parse runtime metrics: $runtime_line" >&2
        fi
        return 1
    fi
    if ! awk \
        -v compute_count="$compute_count" \
        -v update_count="$update_count" \
        -v minimum_compute_count="$minimum_compute_count" \
        -v minimum_update_count="$minimum_update_count" \
        -v steady_compute="$steady_compute" \
        -v steady_display="$steady_display" \
        -v max_dt="$max_dt" \
        -v lag="$lag" \
        'BEGIN {
            if (compute_count <= minimum_compute_count || update_count <= minimum_update_count || steady_compute < 10.0 || steady_display < 4.5 || max_dt > 0.026 || lag > 0.100) {
                exit 1
            }
        }'; then
        if [[ "$quiet" != true ]]; then
            echo "Runtime freshness or thresholds failed: $runtime_line" >&2
        fi
        return 1
    fi
    if ! check_launch_health "$launch_log" "$quiet"; then
        return 1
    fi
    RUNTIME_COMPUTE_COUNT="$compute_count"
    RUNTIME_UPDATE_COUNT="$update_count"
    if [[ "$quiet" != true ]]; then
        echo "FORMAL_ACCEPTANCE_RUNTIME_OK compute=$compute_count updates=$update_count steady_compute_hz=$steady_compute steady_display_hz=$steady_display cumulative_compute_hz=$compute_hz cumulative_display_hz=$display_hz max_dt=$max_dt lag=$lag"
    fi
}

validate_case_evidence() {
    local scenario="$1"
    local round_index="$2"
    local verifier_log="$3"
    local launch_log="$4"
    local target_state
    local behavior_types
    local target_cause
    case "$scenario" in
        safe)
            target_state="CURIOUS"
            behavior_types="1,5"
            target_cause="ATTENTION_DWELL"
            ;;
        sudden)
            target_state="SURPRISED"
            behavior_types="1,3"
            target_cause="SUDDEN_NEAR"
            ;;
        fast)
            target_state="SCARED"
            behavior_types="1,4"
            target_cause="ROBOT_FAST_APPROACH"
            ;;
        *)
            echo "Unknown acceptance scenario: $scenario" >&2
            return 1
            ;;
    esac

    local marker_count
    local marker
    marker_count="$(grep -c '^FORMAL_SOCIAL_SCENARIO_OK ' "$verifier_log" || true)"
    if [[ "$marker_count" != 1 ]]; then
        echo "Expected exactly one scenario success marker, got $marker_count" >&2
        return 1
    fi
    marker="$(grep '^FORMAL_SOCIAL_SCENARIO_OK ' "$verifier_log")"
    for token in \
        "scenario=$scenario" \
        "round=$round_index" \
        "target=$target_state" \
        "resets=2" \
        "cmd_vel_publishers=1" \
        "regular_motion=stopped" \
        "behavior_types=$behavior_types"; do
        if [[ "$marker" != *"$token"* ]]; then
            echo "Scenario marker is missing $token: $marker" >&2
            return 1
        fi
    done

    local reset_calls
    local attention_transitions
    local target_transitions
    local recovery_transitions
    reset_calls="$(grep -c '=== RESET AGENTS SERVICE CALLED ===' "$launch_log" || true)"
    attention_transitions="$(grep -c \
        'FORMAL_SOCIAL_TRANSITION .*old=NORMAL new=ATTENTION cause=ROBOT_VISIBLE resets=0' \
        "$launch_log" || true)"
    target_transitions="$(grep -c \
        "FORMAL_SOCIAL_TRANSITION .*old=ATTENTION new=$target_state cause=$target_cause resets=1" \
        "$launch_log" || true)"
    recovery_transitions="$(grep -c \
        "FORMAL_SOCIAL_TRANSITION .*old=$target_state new=NORMAL .*resets=2" \
        "$launch_log" || true)"
    if [[ "$reset_calls" != 2 ]]; then
        echo "Expected exactly two raw HuNav reset calls, got $reset_calls" >&2
        return 1
    fi
    if [[ "$attention_transitions" != 1 || "$target_transitions" != 1 || "$recovery_transitions" != 1 ]]; then
        echo "Unexpected committed transition counts: attention=$attention_transitions target=$target_transitions recovery=$recovery_transitions" >&2
        return 1
    fi
    echo "FORMAL_ACCEPTANCE_CASE_EVIDENCE_OK scenario=$scenario round=$round_index raw_resets=$reset_calls required_transitions=3"
}

run_case() {
    local scenario="$1"
    local round_index="$2"
    local domain_id="$3"
    local acceleration="2.0"
    local physics_dt="$ARENA_PHYSICS_DT"
    local command_timeout="${ARENA_IDEAL_COMMAND_TIMEOUT:-0.5}"
    if [[ "$scenario" == "fast" ]]; then
        # The fast case is a bounded, Nav2-free pulse test. A 10 ms physics
        # step and 12 ms command watchdog preserve a real 0.8 m/s odometry
        # sample while ensuring the stale approach cannot outlive the next
        # SCARED compute beat. Production demo defaults remain unchanged.
        acceleration="100.0"
        physics_dt="0.01"
        command_timeout="0.012"
    fi

    local case_dir="$MATRIX_ROOT/${scenario}_round${round_index}_domain${domain_id}"
    local launch_log="$case_dir/launch.log"
    mkdir -p "$case_dir"
    export ROS_DOMAIN_ID="$domain_id"

    echo "FORMAL_ACCEPTANCE_CASE_START scenario=$scenario round=$round_index domain=$domain_id acceleration=$acceleration physics_dt=$physics_dt command_timeout=$command_timeout"
    setsid env \
        DRL_VO_GUI=false \
        ROS_DOMAIN_ID="$domain_id" \
        GPU_ID="$GPU_ID" \
        NAVIGATION=false \
        ARENA_IDEAL_CHASSIS=true \
        ARENA_IDEAL_LINEAR_ACCELERATION="$acceleration" \
        ARENA_IDEAL_COMMAND_TIMEOUT="$command_timeout" \
        ARENA_PHYSICS_DT="$physics_dt" \
        "$SCRIPT_DIR/run_formal_social_demo.sh" \
        headless:=true livestream:=false foxglove:=false \
        >"$launch_log" 2>&1 &
    CURRENT_PID=$!

    local ready=false
    for _ in $(seq 1 150); do
        if grep -q 'FORMAL_SOCIAL_BRIDGE_READY' "$launch_log"; then
            ready=true
            break
        fi
        if ! kill -0 "$CURRENT_PID" 2>/dev/null; then
            tail -200 "$launch_log"
            return 1
        fi
        sleep 1
    done
    if [[ "$ready" != true ]]; then
        tail -200 "$launch_log"
        return 1
    fi

    # The first exact report interval can contain service startup.  Wait for an
    # interval that satisfies every steady-state gate before driving motion.
    local runtime_ready=false
    for _ in $(seq 1 90); do
        if check_runtime_log "$launch_log" -1 -1 true; then
            runtime_ready=true
            break
        fi
        if ! kill -0 "$CURRENT_PID" 2>/dev/null; then
            tail -200 "$launch_log"
            return 1
        fi
        sleep 1
    done
    if [[ "$runtime_ready" != true ]]; then
        check_runtime_log "$launch_log" -1 -1 false
        return 1
    fi
    check_runtime_log "$launch_log" -1 -1 false
    local pre_action_compute_count="$RUNTIME_COMPUTE_COUNT"
    local pre_action_update_count="$RUNTIME_UPDATE_COUNT"

    ros2 run formal_social_behavior verify_formal_social_scenario \
        --ros-args \
        -p scenario:="$scenario" \
        -p round:="$round_index" \
        | tee "$case_dir/verifier.log"
    validate_case_evidence \
        "$scenario" \
        "$round_index" \
        "$case_dir/verifier.log" \
        "$launch_log"

    # Require a different, later status interval after the behavior and
    # recovery.  This proves compute/display progress and prevents reusing the
    # pre-action green marker when a future or display service stalls.
    local post_runtime_ready=false
    for _ in $(seq 1 90); do
        if check_runtime_log \
            "$launch_log" \
            "$pre_action_compute_count" \
            "$pre_action_update_count" \
            true; then
            post_runtime_ready=true
            break
        fi
        if ! kill -0 "$CURRENT_PID" 2>/dev/null; then
            tail -200 "$launch_log"
            return 1
        fi
        sleep 1
    done
    if [[ "$post_runtime_ready" != true ]]; then
        check_runtime_log \
            "$launch_log" \
            "$pre_action_compute_count" \
            "$pre_action_update_count" \
            false
        return 1
    fi
    check_runtime_log \
        "$launch_log" \
        "$pre_action_compute_count" \
        "$pre_action_update_count" \
        false
    cleanup_current
    check_launch_health "$launch_log" false
    echo "FORMAL_ACCEPTANCE_POST_CLEANUP_LOG_OK scenario=$scenario round=$round_index"
    sleep 2
}

case_index=0
for scenario in safe sudden fast; do
    for round_index in 1 2; do
        run_case "$scenario" "$round_index" "$((DOMAIN_BASE + case_index))"
        case_index=$((case_index + 1))
    done
done

sha256sum --check "$MATRIX_ROOT/protected_before.sha256"
(
    cd "$BASE_WS"
    sha256sum --check "$KEY_MANIFEST"
)
echo "FORMAL_ACCEPTANCE_PROTECTED_FINAL_OK files=${#PROTECTED_PATHS[@]}"
echo "FORMAL_SOCIAL_ACCEPTANCE_MATRIX_OK cases=6 rounds=2 scenarios=safe,sudden,fast"
