#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh" >/dev/null
RUN_DIR="${1:-${ARENA_MULTI_RUN_DIR:-}}"
if [[ -z "$RUN_DIR" || ! -d "$RUN_DIR" ]]; then
    echo "Usage: capture_graph.sh RUN_DIR" >&2
    exit 2
fi
ros2 node list --no-daemon | sort > "$RUN_DIR/ros_nodes.txt"
ros2 topic list -t --no-daemon | sort > "$RUN_DIR/ros_topics.txt"
ros2 service list -t --no-daemon | sort > "$RUN_DIR/ros_services.txt"
ros2 action list -t | sort > "$RUN_DIR/ros_actions.txt"
: > "$RUN_DIR/actual_parameters.yaml"
while IFS= read -r node; do
    echo "# $node" >> "$RUN_DIR/actual_parameters.yaml"
    # Some internal transform-listener nodes have no parameter service and can
    # otherwise block a complete evidence capture indefinitely.
    timeout 5s ros2 param dump "$node" --no-daemon >> "$RUN_DIR/actual_parameters.yaml" || \
        echo "# parameter dump unavailable or timed out" >> "$RUN_DIR/actual_parameters.yaml"
done < "$RUN_DIR/ros_nodes.txt"
if command -v nvidia-smi >/dev/null; then
    nvidia-smi -q > "$RUN_DIR/nvidia-smi-q.txt" || true
fi
echo "MULTIROBOT_GRAPH_CAPTURE_OK run_dir=$RUN_DIR"
