#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BASE_WS="${ARENA_BASE_WS:-/home/lpc/workspace/arena5_ws}"
export FORMAL_OVERLAY_ROOT="${FORMAL_OVERLAY_ROOT:-$FEATURE_ROOT/.colcon-formal-phase4}"
export GPU_ID="${GPU_ID:-3}" NAVIGATION="${NAVIGATION:-false}"
export ARENA_IDEAL_CHASSIS="${ARENA_IDEAL_CHASSIS:-true}"
export ARENA_PHYSICS_DT="${ARENA_PHYSICS_DT:-0.016666666666666666}"
export ARENA_IDEAL_COMMAND_TIMEOUT="${ARENA_IDEAL_COMMAND_TIMEOUT:-0.5}"
ENABLED="${FORMAL_MULTI_ENABLED:-true}"
SHARED="${FORMAL_MULTI_SHARED_EVENTS:-true}"
for boolean in "$NAVIGATION" "$ARENA_IDEAL_CHASSIS" "$ENABLED" "$SHARED"; do
    case "$boolean" in true|false) ;; *) echo "Expected boolean, got: $boolean" >&2; exit 2;; esac
done
if [[ ! -f "$FORMAL_OVERLAY_ROOT/install/local_setup.bash" ]]; then
    echo "Build first: FORMAL_OVERLAY_ROOT=$FORMAL_OVERLAY_ROOT scripts/build_formal_overlay.sh" >&2
    exit 1
fi
source "$BASE_WS/scripts/env.sh"
set +u
source "$FORMAL_OVERLAY_ROOT/install/local_setup.bash"
set -u
for package in arena_isaac arena_humble_compat formal_social_behavior; do
    actual="$(ros2 pkg prefix "$package")"
    if [[ "$(readlink -f "$actual")" != "$(readlink -f "$FORMAL_OVERLAY_ROOT/install/$package")" ]]; then
        echo "Incorrect Phase 4 package prefix: $package=$actual" >&2
        exit 1
    fi
done
AUTOMATA="${FORMAL_MULTI_AUTOMATA_CONFIG:-$FEATURE_ROOT/src/formal_social_behavior/config/formal_social_multi_automata.yaml}"
AGENTS="${FORMAL_MULTI_AGENT_CONFIG:-$FEATURE_ROOT/src/formal_social_behavior/config/formal_social_multi_agents.yaml}"
# Critical values use the environment/config interface so manifest hashes remain exact.
for arg in "$@"; do
    case "$arg" in automata_config:=*|agent_config:=*|enabled:=*|shared_events_enabled:=*|navigation:=*|trace_file:=*|steps_file:=*|backend_trace_file:=*)
        echo "Use FORMAL_MULTI_*/NAVIGATION environment variables for critical configuration: $arg" >&2; exit 2;;
    esac
done
RUN_DIR="$FEATURE_ROOT/logs/formal_multi/$(date +%Y%m%d_%H%M%S_%N)_gpu${GPU_ID}_pid${BASHPID}"
mkdir -p "$RUN_DIR"
export ROS_LOG_DIR="$RUN_DIR/ros"
export FORMAL_MULTI_TRACE_FILE="$RUN_DIR/transitions.jsonl"
export FORMAL_MULTI_STEPS_FILE="$RUN_DIR/steps.jsonl"
export FORMAL_MULTI_BACKEND_TRACE_FILE="$RUN_DIR/backend.jsonl"
{
    printf 'git_commit=%s\n' "$(git -C "$FEATURE_ROOT" rev-parse HEAD)"
    printf 'source_sha256=%s\n' "$(cd "$FEATURE_ROOT" && git ls-files -z --cached --others --exclude-standard src/formal_social_behavior | sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1)"
    printf 'gpu_id=%s\nros_domain_id=%s\noverlay=%s\n' "$GPU_ID" "$ROS_DOMAIN_ID" "$FORMAL_OVERLAY_ROOT"
    printf 'enabled=%s\nshared_events_enabled=%s\nnavigation=%s\n' "$ENABLED" "$SHARED" "$NAVIGATION"
    printf 'physics_dt=%s\nideal_chassis=%s\ncommand_timeout=%s\n' "$ARENA_PHYSICS_DT" "$ARENA_IDEAL_CHASSIS" "$ARENA_IDEAL_COMMAND_TIMEOUT"
    printf 'linear_acceleration=%s\n' "${ARENA_IDEAL_LINEAR_ACCELERATION:-2.0}"
    printf 'automata_config=%s\nagent_config=%s\n' "$AUTOMATA" "$AGENTS"
    sha256sum "$AUTOMATA" "$AGENTS"
    printf 'extra_launch_arg=%s\n' "$@"
} > "$RUN_DIR/run_manifest.txt"
ros2 run formal_social_behavior export_formal_social_multi_model "$AUTOMATA" --output "$RUN_DIR/model.json"
exec > >(tee "$RUN_DIR/console.log") 2>&1
echo "FORMAL_SOCIAL_MULTI_RUN_DIR=$RUN_DIR"
exec ros2 launch formal_social_behavior formal_social_multi_demo.launch.py \
    navigation:="$NAVIGATION" enabled:="$ENABLED" shared_events_enabled:="$SHARED" \
    automata_config:="$AUTOMATA" agent_config:="$AGENTS" "$@"
