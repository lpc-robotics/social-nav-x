#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARENA_MULTI_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
ARENA_MULTI_STATE_ROOT="${ARENA_MULTI_STATE_ROOT:-$ARENA_MULTI_WS}"
ARENA_MULTI_LOG_ROOT="${ARENA_MULTI_LOG_ROOT:-$ARENA_MULTI_STATE_ROOT/logs/runs}"
found=0
while IFS= read -r pid_file; do
    pid="$(tr -d '[:space:]' < "$pid_file")"
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    group_rows="$(ps -eo pid=,pgid=,cmd= | awk -v group="$pid" '$2 == group {print}')"
    [[ -n "$group_rows" ]] || continue
    if [[ "$group_rows" != *"arena_multi_bringup"* && "$group_rows" != *"run_isaacsim"* && "$group_rows" != *"$ARENA_MULTI_WS"* ]]; then
        echo "Refusing PGID $pid because it has no Arena multi process." >&2
        continue
    fi
    echo "Stopping Arena multi process group $pid"
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in {1..40}; do
        ps -eo pgid= | awk -v group="$pid" '$1 == group {found=1} END {exit !found}' || break
        sleep 0.25
    done
    if ps -eo pgid= | awk -v group="$pid" '$1 == group {found=1} END {exit !found}'; then
        echo "PGID $pid did not stop after SIGINT; sending SIGTERM."
        kill -TERM -- "-$pid" 2>/dev/null || true
        for _ in {1..20}; do
            ps -eo pgid= | awk -v group="$pid" '$1 == group {found=1} END {exit !found}' || break
            sleep 0.25
        done
    fi
    if ps -eo pgid= | awk -v group="$pid" '$1 == group {found=1} END {exit !found}'; then
        echo "PGID $pid did not stop after SIGTERM; sending SIGKILL."
        kill -KILL -- "-$pid" 2>/dev/null || true
    fi
    found=1
done < <(find "$ARENA_MULTI_LOG_ROOT" -type f -name launch.pid 2>/dev/null | sort)
if (( found == 0 )); then
    echo "No running Arena multi launch found."
fi
