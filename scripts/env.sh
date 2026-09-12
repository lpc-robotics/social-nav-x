#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Please source this file: source scripts/env.sh" >&2
    exit 1
fi

export ARENA_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ARENA_ROS_ENV="$ARENA_WS/.conda/arena_ros"
export ISAAC_ENV="/home/lpc/miniforge3/envs/isaaclab"
export ISAAC_PATH="$ISAAC_ENV/lib/python3.11/site-packages/isaacsim"
export ISAAC_SITE="$ISAAC_ENV/lib/python3.11/site-packages"
export EXP_PATH="$ISAAC_PATH/apps"
export ISAAC_PYTHON="$ISAAC_ENV/bin/python"

# Conda/ROS activation hooks legitimately probe unset variables.
_ARENA_NOUNSET_WAS_ON=0
if [[ $- == *u* ]]; then
    _ARENA_NOUNSET_WAS_ON=1
    set +u
fi
source /home/lpc/miniforge3/etc/profile.d/conda.sh
conda activate "$ARENA_ROS_ENV"
source "$ARENA_WS/install/setup.bash"
if [[ "$_ARENA_NOUNSET_WAS_ON" == 1 ]]; then
    set -u
fi
unset _ARENA_NOUNSET_WAS_ON

export ROS_DISTRO=humble
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-151}"
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"
export OMNI_KIT_ACCEPT_EULA=YES
export RENDER_PRESET="${RENDER_PRESET:-boring}"

export XDG_CACHE_HOME="$ARENA_WS/.cache/xdg"
export XDG_CONFIG_HOME="$ARENA_WS/.cache/xdg-config"
export XDG_DATA_HOME="$ARENA_WS/.cache/xdg-data"
export PIP_CACHE_DIR="$ARENA_WS/.cache/pip"
export CONDA_PKGS_DIRS="$ARENA_WS/.cache/conda-pkgs"
export ROS_LOG_DIR="$ARENA_WS/logs/ros"
export PYTHONDONTWRITEBYTECODE=1
export CPLUS_INCLUDE_PATH="$ARENA_WS/install/lightsfm/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"

# Isaac 5.1 embeds Python 3.11 while the local ROS environment also uses
# Python 3.11. Keep its Python modules and the Humble bridge libraries scoped
# to this workspace environment instead of installing anything in $HOME.
if [[ ":${PYTHONPATH:-}:" != *":$ISAAC_SITE:"* ]]; then
    export PYTHONPATH="$ISAAC_SITE${PYTHONPATH:+:$PYTHONPATH}"
fi
export LD_LIBRARY_PATH="$ARENA_WS/install/arena_people_msgs/lib:$ARENA_WS/install/isaacsim_msgs/lib:$ISAAC_PATH/exts/isaacsim.ros2.bridge/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ ":${LD_PRELOAD:-}:" != *":$ARENA_ROS_ENV/lib/libstdc++.so.6:"* ]]; then
    export LD_PRELOAD="$ARENA_ROS_ENV/lib/libstdc++.so.6${LD_PRELOAD:+:$LD_PRELOAD}"
fi

export ARENA_WEBRTC_SIGNAL_PORT="${ARENA_WEBRTC_SIGNAL_PORT:-49220}"
export ARENA_WEBRTC_MEDIA_PORT="${ARENA_WEBRTC_MEDIA_PORT:-48020}"
export ARENA_FOXGLOVE_ADDRESS="${ARENA_FOXGLOVE_ADDRESS:-127.0.0.1}"
export ARENA_FOXGLOVE_PORT="${ARENA_FOXGLOVE_PORT:-8875}"
export ARENA_FOXGLOVE_CAPABILITIES="${ARENA_FOXGLOVE_CAPABILITIES:-[clientPublish,connectionGraph,assets]}"
if [[ -z "${ARENA_FOXGLOVE_ASSET_ALLOWLIST:-}" ]]; then
    export ARENA_FOXGLOVE_ASSET_ALLOWLIST="['^package://[A-Za-z0-9_%./-]+[.](dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$', '^file://$ARENA_WS/src/arena/simulation-setup/entities/robots/jackal/urdf/meshes/[A-Za-z0-9_.-]+[.]stl$']"
fi
if [[ -z "${ARENA_WEBRTC_IP:-}" ]]; then
    if [[ -n "${SSH_CONNECTION:-}" ]]; then
        export ARENA_WEBRTC_IP="$(awk '{print $3}' <<<"$SSH_CONNECTION")"
    else
        export ARENA_WEBRTC_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
    fi
    # This host was inventoried as 10.16.202.189; the explicit environment
    # variable remains authoritative if networking changes.
    export ARENA_WEBRTC_IP="${ARENA_WEBRTC_IP:-10.16.202.189}"
fi

mkdir -p \
    "$XDG_CACHE_HOME" "$XDG_CONFIG_HOME" "$XDG_DATA_HOME" \
    "$PIP_CACHE_DIR" "$CONDA_PKGS_DIRS" "$ARENA_WS/logs"

echo "Arena workspace: $ARENA_WS"
echo "ROS 2: $ROS_DISTRO (domain $ROS_DOMAIN_ID)"
echo "Isaac Sim: $ISAAC_PATH"
echo "WebRTC endpoint: $ARENA_WEBRTC_IP:$ARENA_WEBRTC_SIGNAL_PORT (UDP $ARENA_WEBRTC_MEDIA_PORT)"
echo "Foxglove WebSocket: $ARENA_FOXGLOVE_ADDRESS:$ARENA_FOXGLOVE_PORT"
