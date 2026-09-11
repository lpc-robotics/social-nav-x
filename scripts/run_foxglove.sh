#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    echo "Foxglove Bridge is not built. Run ./scripts/build.sh first." >&2
    exit 1
fi

echo "Foxglove WebSocket: ws://$ARENA_FOXGLOVE_ADDRESS:$ARENA_FOXGLOVE_PORT"
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    address:="$ARENA_FOXGLOVE_ADDRESS" \
    port:="$ARENA_FOXGLOVE_PORT" \
    capabilities:="$ARENA_FOXGLOVE_CAPABILITIES" \
    asset_uri_allowlist:="$ARENA_FOXGLOVE_ASSET_ALLOWLIST" \
    publish_client_count:=true \
    use_sim_time:=true
