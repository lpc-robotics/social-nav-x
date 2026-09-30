#!/usr/bin/env bash
set -Eeuo pipefail

RELEASE_ROOT="/home/lpc/workspace/arena5_ws/optional/multirobot/releases/20260924-bdd959da"
export ARENA_MULTI_STATE_ROOT="${ARENA_MULTI_STATE_ROOT:-/home/lpc/workspace/arena5_ws/.multirobot}"
export ARENA_MULTI_LOG_ROOT="${ARENA_MULTI_LOG_ROOT:-$ARENA_MULTI_STATE_ROOT/logs/runs}"
exec "$RELEASE_ROOT/scripts/run_multirobot.sh" "$@"
