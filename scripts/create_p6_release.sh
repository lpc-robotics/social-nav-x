#!/usr/bin/env bash
set -Eeuo pipefail

MPC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STABLE_WS="${ARENA_STABLE_WS:-/home/lpc/workspace/arena5_ws}"
RELEASE_ID="${1:?usage: create_p6_release.sh RELEASE_ID}"
DEST_ROOT="$STABLE_WS/optional/mpc/releases/$RELEASE_ID"
CASADI_CMAKE="$MPC_WS/third_party/casadi-3.8.0-wheel/casadi/cmake"

if [[ ! "$RELEASE_ID" =~ ^[0-9A-Za-z._-]+$ ]]; then
    echo "release ID contains unsupported characters" >&2
    exit 2
fi
if [[ -e "$DEST_ROOT" ]]; then
    echo "immutable release already exists: $DEST_ROOT" >&2
    exit 1
fi
(cd "$STABLE_WS" && sha256sum --check "$MPC_WS/config/stable_protected.sha256")

STAGE="$(mktemp -d /tmp/arena5-mpc-release.XXXXXX)"
cleanup() { rm -rf "$STAGE"; }
trap cleanup EXIT

set +u
source "$STABLE_WS/scripts/env.sh" >/dev/null
set -u

colcon --log-base "$STAGE/log" build \
    --base-paths "$MPC_WS/src" \
    --packages-up-to arena_mpc_bringup \
    --merge-install \
    --build-base "$STAGE/build" \
    --install-base "$STAGE/release/install" \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
        -Dcasadi_DIR="$CASADI_CMAKE" \
        "-DCMAKE_CXX_FLAGS=-ffile-prefix-map=$MPC_WS=/usr/src/arena_mpc -fmacro-prefix-map=$MPC_WS=/usr/src/arena_mpc -fdebug-prefix-map=$MPC_WS=/usr/src/arena_mpc -ffile-prefix-map=$STAGE=/usr/src/arena_mpc_build -fdebug-prefix-map=$STAGE=/usr/src/arena_mpc_build" \
        -DCMAKE_CXX_COMPILER="$STABLE_WS/.conda/arena_ros/bin/x86_64-conda-linux-gnu-c++" \
        -DPython3_EXECUTABLE="$STABLE_WS/.conda/arena_ros/bin/python"

cp "$MPC_WS/scripts/run_mpc_release.sh" "$STAGE/release/run_mpc_release.sh"
cp "$MPC_WS/config/stable_protected.sha256" "$STAGE/release/stable_protected.sha256"
chmod +x "$STAGE/release/run_mpc_release.sh"

# Colcon's Bash entry points are relocatable, but generated POSIX fallback
# scripts and parent-prefix resources retain the build-time install path.  The
# release runner uses local_setup.bash, which supplies COLCON_CURRENT_PREFIX to
# these package scripts.  Replace their unused fallback and remove this
# overlay's old self-prefix from parent metadata before the relocation scan.
python - "$STAGE/release/install" <<'PY'
import sys
from pathlib import Path

prefix = Path(sys.argv[1])
needle = str(prefix)
for path in prefix.rglob("*.sh"):
    text = path.read_text(encoding="utf-8")
    if needle in text:
        path.write_text(
            text.replace(needle, "${COLCON_CURRENT_PREFIX}"), encoding="utf-8"
        )
parent_index = prefix / "share/ament_index/resource_index/parent_prefix_path"
if parent_index.is_dir():
    for path in parent_index.iterdir():
        entries = path.read_text(encoding="utf-8").split(":")
        path.write_text(":".join(item for item in entries if item != needle), encoding="utf-8")
PY

if find "$STAGE/release" -type l -print -quit | grep -q .; then
    echo "release contains symlinks" >&2
    find "$STAGE/release" -type l -print >&2
    exit 1
fi
if rg -l --text -e "$MPC_WS" -e "$STAGE" "$STAGE/release" | grep -q .; then
    echo "release contains a development or staging absolute path" >&2
    rg -l --text -e "$MPC_WS" -e "$STAGE" "$STAGE/release" >&2
    exit 1
fi

mv "$STAGE/release" "$STAGE/relocated"
"$STAGE/relocated/run_mpc_release.sh" --check-runtime-only
set +u
source "$STAGE/relocated/install/local_setup.bash"
set -u
"$STAGE/relocated/install/lib/arena_mpc_core/mpc_benchmark" 10 >/dev/null
for binary in \
    "$STAGE/relocated/install/lib/libarena_mpc_core.so" \
    "$STAGE/relocated/install/lib/libarena_mpc_controller.so" \
    "$STAGE/relocated/install/lib/arena_mpc_controller/mpc_command_watchdog"; do
    if ldd "$binary" | grep -q 'not found'; then
        echo "unresolved dependency in $binary" >&2
        ldd "$binary" >&2
        exit 1
    fi
done

{
    printf 'release_id=%s\n' "$RELEASE_ID"
    printf 'source_commit=%s\n' "$(git -C "$MPC_WS" rev-parse HEAD)"
    printf 'controller_sha256=%s\n' "$(sha256sum "$MPC_WS/src/arena_mpc_controller/src/mpc_controller.cpp" | awk '{print $1}')"
    printf 'controller_config_sha256=%s\n' "$(sha256sum "$MPC_WS/src/arena_mpc_bringup/config/controller_model.yaml" | awk '{print $1}')"
    printf 'nav2_overrides_sha256=%s\n' "$(sha256sum "$MPC_WS/src/arena_mpc_bringup/config/nav2_overrides.yaml" | awk '{print $1}')"
    printf 'casadi_version=3.8.0\n'
    printf 'ros_distro=humble\n'
} >"$STAGE/relocated/RELEASE"
(cd "$STAGE/relocated" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum >SHA256SUMS)
(cd "$STAGE/relocated" && sha256sum --check SHA256SUMS)

mkdir -p "$(dirname "$DEST_ROOT")"
cp -a "$STAGE/relocated" "$DEST_ROOT"
mkdir -p "$STABLE_WS/optional/mpc"
: >"$STABLE_WS/optional/mpc/COLCON_IGNORE"
cat >"$STABLE_WS/scripts/run_six_behaviors_mpc.sh" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail
exec "\$(cd "\$(dirname "\${BASH_SOURCE[0]}")/.." && pwd)/optional/mpc/releases/$RELEASE_ID/run_mpc_release.sh" "\$@"
EOF
chmod +x "$STABLE_WS/scripts/run_six_behaviors_mpc.sh"

(cd "$DEST_ROOT" && sha256sum --check SHA256SUMS)
(cd "$STABLE_WS" && sha256sum --check "$MPC_WS/config/stable_protected.sha256")
"$STABLE_WS/scripts/run_six_behaviors_mpc.sh" --check-runtime-only
echo "MPC_RELEASE_CREATED $DEST_ROOT"
