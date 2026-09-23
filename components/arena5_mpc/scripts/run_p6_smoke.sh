#!/usr/bin/env bash
set -Eeuo pipefail

MPC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STABLE_WS="${ARENA_STABLE_WS:-/home/lpc/workspace/arena5_ws}"
RELEASE_ID="${1:?usage: run_p6_smoke.sh RELEASE_ID}"
RELEASE_ROOT="$STABLE_WS/optional/mpc/releases/$RELEASE_ID"
GPU="${GPU_ID:-0}"
DOMAIN_BASE="${P6_DOMAIN_BASE:-200}"
OUTPUT_DIR="$MPC_WS/evidence/p6"
PROBE="$MPC_WS/tools/p5_comparison_probe.py"
CONFIG="$STABLE_WS/install/arena_bringup/share/arena_bringup/configs/hunav_agents/isaac_six_behaviors_warehouse.yaml"

if [[ ! -f "$RELEASE_ROOT/RELEASE" ]]; then
    echo "MPC release is missing: $RELEASE_ROOT" >&2
    exit 1
fi
mkdir -p "$OUTPUT_DIR"
CONFIG_SHA256="$(sha256sum "$CONFIG" | awk '{print $1}')"
PROBE_SHA256="$(sha256sum "$PROBE" | awk '{print $1}')"
GPU_SNAPSHOT="$(nvidia-smi --query-gpu=index,uuid,memory.total,memory.used,memory.free --format=csv,noheader,nounits | sed -n "$((GPU + 1))p")"

run_one() (
    set -Eeuo pipefail
    local method="$1"
    local domain="$2"
    local output="$OUTPUT_DIR/${method}_smoke.json"
    local launch_log="$OUTPUT_DIR/${method}_smoke_launch.log"
    local -a command
    local revision

    export ROS_DOMAIN_ID="$domain"
    export GPU_ID="$GPU"
    export LIVESTREAM=false
    export FOXGLOVE=false
    export ARENA_IDEAL_CHASSIS=true
    export ARENA_PHYSICS_DT=0.016666666666666666
    export ARENA_WEBRTC_SIGNAL_PORT="$((52500 + domain))"
    export ARENA_WEBRTC_MEDIA_PORT="$((52000 + domain))"
    export ARENA_FOXGLOVE_PORT="$((9700 + domain))"

    if [[ "$method" == "mpc" ]]; then
        command=("$STABLE_WS/scripts/run_six_behaviors_mpc.sh" "agent_config:=$CONFIG")
        revision="release:$RELEASE_ID"
    else
        command=("$STABLE_WS/scripts/run_six_behaviors.sh" "agent_config:=$CONFIG")
        revision="stable-manifest:$(sha256sum "$MPC_WS/config/stable_protected.sha256" | awk '{print $1}')"
    fi

    setsid "${command[@]}" >"$launch_log" 2>&1 &
    local launch_pid=$!
    cleanup_run() {
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
    trap cleanup_run EXIT INT TERM

    set +u
    source "$STABLE_WS/scripts/env.sh" >/dev/null
    if [[ "$method" == "mpc" ]]; then
        source "$RELEASE_ROOT/install/local_setup.bash"
    fi
    set -u

    python "$PROBE" "$method" \
        --pair 6 \
        --order "$([[ "$method" == "mpc" ]] && echo 1 || echo 2)" \
        --target-x 3.6 \
        --target-y 3.0 \
        --expected-agents 6 \
        --interaction-distance 1.5 \
        --expected-max-linear "$([[ "$method" == "mpc" ]] && echo 0.8 || echo 0.26)" \
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

    cleanup_run
    trap - EXIT INT TERM
    (cd "$STABLE_WS" && sha256sum --check "$MPC_WS/config/stable_protected.sha256")
)

run_one mpc "$DOMAIN_BASE"
run_one dwb "$((DOMAIN_BASE + 1))"

python - "$OUTPUT_DIR" "$RELEASE_ID" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
release_id = sys.argv[2]
results = {name: json.loads((root / f"{name}_smoke.json").read_text()) for name in ("mpc", "dwb")}
summary = {
    "gate": "PASS" if all(item.get("pass") for item in results.values()) else "FAIL",
    "release_id": release_id,
    "results": {
        name: {
            "pass": item.get("pass"),
            "action_status": item.get("action_status"),
            "follow_path_plugin": item.get("controller", {}).get("follow_path_plugin"),
            "position_error_m": item.get("position_error_m"),
            "minimum_clearance_lower_bound_m": item.get("safety", {}).get("minimum_clearance_lower_bound_m"),
        }
        for name, item in results.items()
    },
}
(root / "formal_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
print(json.dumps(summary, indent=2, sort_keys=True))
raise SystemExit(0 if summary["gate"] == "PASS" else 2)
PY
