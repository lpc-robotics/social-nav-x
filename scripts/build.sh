#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
cd "$ARENA_WS"

cmake -E make_directory "$ARENA_WS/install/lightsfm/include/lightsfm"
cmake -E copy_directory \
    "$ARENA_WS/src/deps/hunav/lightsfm/include" \
    "$ARENA_WS/install/lightsfm/include/lightsfm"

colcon build --event-handlers console_cohesion+ \
    --packages-select \
        arena_people_msgs hunav_msgs isaacsim_msgs people_msgs task_generator_msgs \
        arena_rclpy_mixins rl_utils arena_simulation_setup jackal_description \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

set +u
source "$ARENA_WS/install/setup.bash"
set -u
colcon build --event-handlers console_cohesion+ \
    --packages-select \
        arena_humble_compat arena_isaac arena_bringup task_generator \
        hunav_agent_manager \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

set +u
source "$ARENA_WS/install/setup.bash"
set -u
colcon build --event-handlers console_cohesion+ \
    --packages-select foxglove_msgs foxglove_bridge \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DFOXGLOVE_BRIDGE_REMOTE_ACCESS=OFF

set +u
source "$ARENA_WS/install/setup.bash"
set -u
mkdir -p "$ARENA_WS/config/generated" "$ARENA_WS/logs/upstream_diffs"
ros2 run xacro xacro \
    "$ARENA_WS/src/arena/simulation-setup/entities/robots/jackal/urdf/jackal.urdf.xacro" \
    -o "$ARENA_WS/config/generated/jackal.urdf" \
    name:=arena_robot is_sim:=true \
    gazebo_controllers:="$ARENA_WS/src/arena/simulation-setup/entities/robots/jackal/control.yaml"
sed -i \
    "s#package://jackal_description#$ARENA_WS/src/arena/simulation-setup/entities/robots/jackal/urdf#g" \
    "$ARENA_WS/config/generated/jackal.urdf"

# Intent-to-add makes newly created compatibility packages visible to
# `git diff` without staging or committing their contents.
git -C "$ARENA_WS/src/arena-isaac" add --intent-to-add -- \
    arena_humble_compat arena_people_msgs \
    arena_isaac/isaac_utils/config/__init__.py \
    arena_isaac/isaac_utils/managers/__init__.py \
    arena_isaac/isaac_utils/utils/__init__.py
git -C "$ARENA_WS/src/arena-isaac" diff \
    --output="$ARENA_WS/logs/upstream_diffs/arena-isaac.diff"
git -C "$ARENA_WS/src/arena-rosnav" diff \
    --output="$ARENA_WS/logs/upstream_diffs/arena-rosnav.diff"
git -C "$ARENA_WS/src/arena/simulation-setup" diff \
    --output="$ARENA_WS/logs/upstream_diffs/simulation-setup.diff"
git -C "$ARENA_WS/src/deps/foxglove-sdk" diff \
    --output="$ARENA_WS/logs/upstream_diffs/foxglove-sdk.diff"

echo "Build complete. Generated URDF: $ARENA_WS/config/generated/jackal.urdf"
