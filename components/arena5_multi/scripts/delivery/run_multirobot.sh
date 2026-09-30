#!/usr/bin/env bash
set -Eeuo pipefail

# Human visualization defaults. Explicit overrides remain available for tests.
RELEASE_ROOT="/home/lpc/workspace/arena5_ws/optional/multirobot/releases/20260930-c599908-speed05"
export ARENA_MULTI_STATE_ROOT="${ARENA_MULTI_STATE_ROOT:-/home/lpc/workspace/arena5_ws/.multirobot}"
export ARENA_MULTI_LOG_ROOT="${ARENA_MULTI_LOG_ROOT:-$ARENA_MULTI_STATE_ROOT/logs/runs}"
export ARENA_WEBRTC_IP="${ARENA_WEBRTC_IP:-10.16.205.165}"
export ARENA_WEBRTC_SIGNAL_PORT="${ARENA_WEBRTC_SIGNAL_PORT:-49100}"
export ARENA_WEBRTC_MEDIA_PORT="${ARENA_WEBRTC_MEDIA_PORT:-47998}"
export ARENA_FOXGLOVE_ADDRESS="${ARENA_FOXGLOVE_ADDRESS:-127.0.0.1}"
export ARENA_FOXGLOVE_PORT="${ARENA_FOXGLOVE_PORT:-8765}"
export LIVESTREAM="${LIVESTREAM:-true}"
export FOXGLOVE="${FOXGLOVE:-true}"

cat <<EOF
Foxglove (Windows): ws://localhost:8765
Windows PowerShell (keep open):
  ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=3 -L 8765:127.0.0.1:$ARENA_FOXGLOVE_PORT lpc@$ARENA_WEBRTC_IP
WebRTC: Server=$ARENA_WEBRTC_IP Signal=$ARENA_WEBRTC_SIGNAL_PORT Stream=$ARENA_WEBRTC_MEDIA_PORT
WebRTC resolution: select 1920 x 1080 (FHD) in the client.
Server Foxglove listener: $ARENA_FOXGLOVE_ADDRESS:$ARENA_FOXGLOVE_PORT
EOF
if [[ "${1:-}" == "--check-visualization" ]]; then
    exit 0
fi
exec "$RELEASE_ROOT/scripts/run_multirobot.sh" "$@"
