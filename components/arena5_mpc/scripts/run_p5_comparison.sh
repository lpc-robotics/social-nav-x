#!/usr/bin/env bash
set -Eeuo pipefail

MPC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STABLE_WS="${ARENA_STABLE_WS:-/home/lpc/workspace/arena5_ws}"
GPU="${GPU_ID:-0}"
DOMAIN_BASE="${P5_DOMAIN_BASE:-150}"
CONFIG="${P5_CONFIG:-$STABLE_WS/install/arena_bringup/share/arena_bringup/configs/hunav_agents/isaac_six_behaviors_warehouse.yaml}"
OUTPUT_DIR="$MPC_WS/evidence/p5/comparison"
PROBE="$MPC_WS/tools/p5_comparison_probe.py"
START_PAIR="${P5_START_PAIR:-1}"
START_ORDER="${P5_START_ORDER:-1}"

mkdir -p "$OUTPUT_DIR"
CONFIG="$(realpath "$CONFIG")"
CONFIG_SHA256="$(sha256sum "$CONFIG" | awk '{print $1}')"
PROBE_SHA256="$(sha256sum "$PROBE" | awk '{print $1}')"
CONTROLLER_SHA256="$(sha256sum "$MPC_WS/src/arena_mpc_controller/src/mpc_controller.cpp" | awk '{print $1}')"
STABLE_MANIFEST_SHA256="$(sha256sum "$MPC_WS/config/stable_protected.sha256" | awk '{print $1}')"
GPU_SNAPSHOT="$(nvidia-smi --query-gpu=index,uuid,memory.total,memory.used,memory.free --format=csv,noheader,nounits | sed -n "$((GPU + 1))p")"

run_method() {
    local method="$1"
    local pair="$2"
    local order="$3"
    local domain="$4"
    local output="$OUTPUT_DIR/pair${pair}_${order}_${method}.json"
    local launch_log="$OUTPUT_DIR/pair${pair}_${order}_${method}_launch.log"

    export ROS_DOMAIN_ID="$domain"
    export GPU_ID="$GPU"
    export LIVESTREAM=false
    export FOXGLOVE=false
    export ARENA_IDEAL_CHASSIS=true
    export ARENA_PHYSICS_DT=0.016666666666666666
    export ARENA_WEBRTC_SIGNAL_PORT="$((50500 + domain))"
    export ARENA_WEBRTC_MEDIA_PORT="$((50000 + domain))"
    export ARENA_FOXGLOVE_PORT="$((9300 + domain))"

    local -a command
    local revision
    if [[ "$method" == "dwb" ]]; then
        command=("$STABLE_WS/scripts/run_six_behaviors.sh" "agent_config:=$CONFIG")
        revision="stable-manifest:$STABLE_MANIFEST_SHA256"
    else
        export MPC_SCENARIO=six_behaviors
        command=("$MPC_WS/scripts/run_mpc.sh" "agent_config:=$CONFIG")
        revision="controller:$CONTROLLER_SHA256"
    fi

    echo "P5 paired run: pair=$pair order=$order method=$method domain=$domain gpu=$GPU"
    setsid "${command[@]}" >"$launch_log" 2>&1 &
    local launch_pid=$!
    cleanup_method() {
        if kill -0 "$launch_pid" 2>/dev/null; then
            kill -INT -- "-$launch_pid" 2>/dev/null || true
            for _ in $(seq 1 100); do
                kill -0 "$launch_pid" 2>/dev/null || break
                sleep 0.2
            done
            if kill -0 "$launch_pid" 2>/dev/null; then
                kill -TERM -- "-$launch_pid" 2>/dev/null || true
            fi
            wait "$launch_pid" 2>/dev/null || true
        fi
    }
    trap cleanup_method EXIT INT TERM

    set +u
    source "$STABLE_WS/scripts/env.sh" >/dev/null
    if [[ "$method" == "mpc" ]]; then
        source "$MPC_WS/install/setup.bash"
    fi
    set -u

    set +e
    python "$PROBE" "$method" \
        --pair "$pair" \
        --order "$order" \
        --target-x 3.6 \
        --target-y 3.0 \
        --expected-agents 6 \
        --interaction-distance 1.5 \
        --output "$output" \
        --ros-domain-id "$domain" \
        --gpu-index "$GPU" \
        --gpu-snapshot "$GPU_SNAPSHOT" \
        --ideal-chassis "$ARENA_IDEAL_CHASSIS" \
        --physics-dt "$ARENA_PHYSICS_DT" \
        --config-path "$CONFIG" \
        --config-sha256 "$CONFIG_SHA256" \
        --probe-sha256 "$PROBE_SHA256" \
        --source-revision "$revision" \
        --launch-log "$launch_log"
    local probe_status=$?
    set -e

    cleanup_method
    trap - EXIT INT TERM
    (cd "$STABLE_WS" && sha256sum --check "$MPC_WS/config/stable_protected.sha256")
    return "$probe_status"
}

for pair in $(seq 1 "${P5_PAIRS:-5}"); do
    if ((pair % 2 == 1)); then
        methods=(dwb mpc)
    else
        methods=(mpc dwb)
    fi
    for order_index in 0 1; do
        order="$((order_index + 1))"
        if ((pair < START_PAIR || pair == START_PAIR && order < START_ORDER)); then
            continue
        fi
        run_method \
            "${methods[$order_index]}" \
            "$pair" \
            "$order" \
            "$((DOMAIN_BASE + pair * 2 + order_index))"
    done
done
