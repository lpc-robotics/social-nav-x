#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARENA_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
DRY_RUN=0

if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=1
elif [[ $# -ne 0 ]]; then
    echo "usage: $0 [--dry-run]" >&2
    exit 2
fi

CURRENT_UID="$(id -u)"
SELF_PGID="$(ps -o pgid= -p "$$" | tr -d ' ')"
RUNTIME_PATTERN='run_isaacsim|foxglove_bridge|robot_state_publisher|nav2_|topic_tools/relay|arena_humble_compat|arena_simulation_setup|arena_bringup|hunav_agent_manager|hunav_loader'
declare -a target_pgids=()

while read -r pid pgid args; do
    [[ -n "$pid" && -n "$pgid" ]] || continue
    [[ "$pgid" != "$SELF_PGID" ]] || continue
    [[ "$args" == *"$ARENA_WS"* ]] || continue
    [[ "$args" =~ $RUNTIME_PATTERN ]] || continue
    target_pgids+=("$pgid")
done < <(ps -u "$CURRENT_UID" -o pid=,pgid=,args=)

if ((${#target_pgids[@]})); then
    mapfile -t target_pgids < <(printf '%s\n' "${target_pgids[@]}" | sort -n -u)
fi

echo "Arena workspace: $ARENA_WS"
if ((${#target_pgids[@]} == 0)); then
    echo "No Arena runtime process groups found for UID $CURRENT_UID."
else
    echo "Arena runtime process groups: ${target_pgids[*]}"
    ps -u "$CURRENT_UID" -o pid=,ppid=,pgid=,stat=,args= | \
        awk -v groups=" ${target_pgids[*]} " 'index(groups, " " $3 " ")'
fi

if ((DRY_RUN)); then
    echo "Dry run only; no signals sent and the ROS 2 daemon was not stopped."
    exit 0
fi

if ((${#target_pgids[@]})); then
    for pgid in "${target_pgids[@]}"; do
        kill -INT -- "-$pgid" 2>/dev/null || true
    done
    sleep 5

    for pgid in "${target_pgids[@]}"; do
        if kill -0 -- "-$pgid" 2>/dev/null; then
            echo "PGID $pgid did not exit after SIGINT; sending SIGTERM."
            kill -TERM -- "-$pgid" 2>/dev/null || true
        fi
    done
    sleep 3

    for pgid in "${target_pgids[@]}"; do
        if kill -0 -- "-$pgid" 2>/dev/null; then
            echo "PGID $pgid did not exit after SIGTERM; sending SIGKILL."
            kill -KILL -- "-$pgid" 2>/dev/null || true
        fi
    done
fi

# Stop only this workspace's configured ROS domain daemon. This avoids stale
# ros2cli graph results without matching or killing unrelated ROS processes.
if [[ -f "$SCRIPT_DIR/env.sh" ]]; then
    # env.sh is intentionally quiet here; cleanup should show only its targets.
    source "$SCRIPT_DIR/env.sh" >/dev/null
    ROS2CLI_DISABLE_DAEMON=0 ros2 daemon stop >/dev/null 2>&1 || true
fi

echo "Arena runtime cleanup complete."
