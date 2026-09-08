#!/usr/bin/env bash
set -Eeuo pipefail

MPC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STABLE_WS="${ARENA_STABLE_WS:-/home/lpc/workspace/arena5_ws}"
SCENARIO="${1:?usage: run_p3_scenario.sh SCENARIO}"
DOMAIN="${P3_DOMAIN:?set P3_DOMAIN to a private ROS domain}"
GPU="${GPU_ID:-0}"
OUTPUT="$MPC_WS/evidence/p3/${SCENARIO}.json"
LAUNCH_LOG="$MPC_WS/evidence/p3/${SCENARIO}_launch.log"

mkdir -p "$MPC_WS/evidence/p3"

export ROS_DOMAIN_ID="$DOMAIN"
export GPU_ID="$GPU"
export MPC_SCENARIO=empty
export LIVESTREAM=false
export FOXGLOVE=false
export ARENA_WEBRTC_SIGNAL_PORT="$((49300 + DOMAIN))"
export ARENA_WEBRTC_MEDIA_PORT="$((48300 + DOMAIN))"
export ARENA_FOXGLOVE_PORT="$((8900 + DOMAIN))"

setsid "$MPC_WS/scripts/run_mpc.sh" >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
        for _ in $(seq 1 100); do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 0.2
        done
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
        fi
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

set +u
source "$STABLE_WS/scripts/env.sh" >/dev/null
source "$MPC_WS/install/setup.bash"
set -u

set +e
python "$MPC_WS/tools/p3_static_probe.py" "$SCENARIO" \
    --repetitions 5 --wall-timeout 180 --output "$OUTPUT"
PROBE_STATUS=$?
set -e

cleanup
trap - EXIT INT TERM
exit "$PROBE_STATUS"
