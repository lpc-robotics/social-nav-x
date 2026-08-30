#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BASE_WS="${ARENA_BASE_WS:-/home/lpc/workspace/arena5_ws}"
OVERLAY_ROOT="${FORMAL_OVERLAY_ROOT:-$FEATURE_ROOT/.colcon}"

if [[ ! -f "$OVERLAY_ROOT/install/local_setup.bash" ]]; then
    echo "Build the formal overlay first: $SCRIPT_DIR/build_formal_overlay.sh" >&2
    exit 1
fi

export GPU_ID="${GPU_ID:-3}"
export NAVIGATION="${NAVIGATION:-false}"
case "$NAVIGATION" in
    true|false) ;;
    *)
        echo "NAVIGATION must be true or false, got: $NAVIGATION" >&2
        exit 1
        ;;
esac

source "$BASE_WS/scripts/env.sh"
# Generated colcon setup hooks legitimately inspect unset variables.
set +u
source "$OVERLAY_ROOT/install/local_setup.bash"
set -u

AUTOMATA_CONFIG="${FORMAL_SOCIAL_AUTOMATA_CONFIG:-$FEATURE_ROOT/src/formal_social_behavior/config/formal_social_automata.yaml}"
AGENT_CONFIG="${FORMAL_SOCIAL_AGENT_CONFIG:-$FEATURE_ROOT/src/formal_social_behavior/config/formal_social_agent.yaml}"
if [[ ! -f "$AUTOMATA_CONFIG" || ! -f "$AGENT_CONFIG" ]]; then
    echo "Formal social configuration file is missing" >&2
    exit 1
fi

RUN_ID="$(date +%Y%m%d_%H%M%S_%N)_formal_social_gpu${GPU_ID}_pid${BASHPID}"
RUN_DIR="$FEATURE_ROOT/logs/formal_social/$RUN_ID"
mkdir -p "$RUN_DIR"
export FORMAL_SOCIAL_TRACE_FILE="$RUN_DIR/transitions.jsonl"

AUTOMATA_SHA256="$(sha256sum "$AUTOMATA_CONFIG" | awk '{print $1}')"
AGENT_SHA256="$(sha256sum "$AGENT_CONFIG" | awk '{print $1}')"
{
    printf 'run_id=%s\n' "$RUN_ID"
    printf 'git_commit=%s\n' "$(git -C "$FEATURE_ROOT" rev-parse HEAD)"
    printf 'automata_config=%s\n' "$AUTOMATA_CONFIG"
    printf 'automata_sha256=%s\n' "$AUTOMATA_SHA256"
    printf 'agent_config=%s\n' "$AGENT_CONFIG"
    printf 'agent_sha256=%s\n' "$AGENT_SHA256"
    printf 'gpu_id=%s\n' "$GPU_ID"
    printf 'navigation=%s\n' "$NAVIGATION"
} > "$RUN_DIR/run_manifest.txt"

echo "Formal social run directory: $RUN_DIR"
echo "Navigation: $NAVIGATION"
echo "Automata config SHA-256: $AUTOMATA_SHA256"
echo "Agent config SHA-256: $AGENT_SHA256"
exec ros2 launch formal_social_behavior formal_social_demo.launch.py \
    navigation:="$NAVIGATION" \
    automata_config:="$AUTOMATA_CONFIG" \
    agent_config:="$AGENT_CONFIG" \
    "$@"
