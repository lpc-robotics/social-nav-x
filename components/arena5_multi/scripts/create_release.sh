#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STABLE="${ARENA_STABLE_WS:-/home/lpc/workspace/arena5_ws}"
if [[ -n "$(git -C "$ROOT" status --porcelain=v1 --untracked-files=all)" ]]; then
    echo "Refusing to release a dirty source tree." >&2
    exit 1
fi
COMMIT="$(git -C "$ROOT" rev-parse HEAD)"
RELEASE_ID="${1:-$(date +%Y%m%d)-$(git -C "$ROOT" rev-parse --short=8 HEAD)}"
TARGET="$STABLE/optional/multirobot/releases/$RELEASE_ID"
if [[ -e "$TARGET/RELEASE.json" ]]; then
    echo "Immutable release already exists: $TARGET" >&2
    exit 1
fi

"$SCRIPT_DIR/verify_baseline.py"
python3 "$SCRIPT_DIR/verify_multi_sfm_release.py"
mkdir -p "$TARGET" "$TARGET/scripts" "$TARGET/baseline" "$TARGET/docs" "$TARGET/evidence" \
    "$ROOT/.release_build/$RELEASE_ID/build" "$ROOT/.release_build/$RELEASE_ID/log"
cp -a "$ROOT/config" "$TARGET/config"
cp "$ROOT/baseline/stable_underlay.json" "$ROOT/baseline/multi_sfm_development.json" "$TARGET/baseline/"
cp "$ROOT/README.md" "$ROOT/MULTIROBOT_PLATFORM_DEVELOPMENT_PLAN.md" "$TARGET/"
cp -a "$ROOT/docs/." "$TARGET/docs/"
cp -a "$ROOT/evidence/." "$TARGET/evidence/"
for name in env.sh run_multirobot.sh cleanup.sh verify_baseline.py validate_runtime.py \
    validate_control.py validate_angular_sign.py validate_cancel_isolation.py validate_peer_obstacle.py \
    validate_hunav_runtime.py benchmark_navigation.py compare_algorithms.py \
    capture_resources.py capture_graph.sh validate_multi_sfm_runtime.py validate_multi_sfm_faults.py \
    validate_multi_sfm_clearing.py validate_multi_sfm_interaction.py wait_multi_sfm_ready.py; do
    cp "$ROOT/scripts/$name" "$TARGET/scripts/$name"
done
tar --exclude='*/__pycache__' --exclude='*/.pytest_cache' \
    -C "$ROOT" -czf "$TARGET/source_snapshot.tar.gz" src

export ARENA_MULTI_SKIP_OVERLAY=true
source "$SCRIPT_DIR/env.sh" >/dev/null
unset ARENA_MULTI_SKIP_OVERLAY
colcon --log-base "$ROOT/.release_build/$RELEASE_ID/log" build \
    --base-paths "$ROOT/src" \
    --build-base "$ROOT/.release_build/$RELEASE_ID/build" \
    --install-base "$TARGET/install" \
    --packages-select arena_multi_hunav_msgs arena_multi_hunav_core arena_isaac arena_multi_control arena_peer_costmap arena_multi_bringup arena_multi_hunav \
    --packages-ignore arena_people_msgs isaacsim_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

python3 - "$TARGET/RELEASE.json" "$RELEASE_ID" "$COMMIT" "$ROOT/evidence/multi_sfm/acceptance.json" <<'PY'
import json
from datetime import datetime
from pathlib import Path
import sys

path, release_id, commit, acceptance_path = sys.argv[1:]
acceptance = json.loads(Path(acceptance_path).read_text())
Path(path).write_text(json.dumps({
    "schema_version": 3,
    "release_id": release_id,
    "source_commit": commit,
    "built_at": datetime.now().astimezone().isoformat(),
    "ros_domain_default": 71,
    "gpu_validation": {
        "status": "passed_multi_sfm_acceptance",
        "validated_on": acceptance["validated_on"],
        "supported_robot_count": 2,
        "pedestrian_counts": [1, 6],
        "robot_max_linear_mps": 0.26,
        "robot_max_angular_radps": 1.0,
        "physics_dt": 1.0 / 20.0,
        "psychology": "noop",
        "evidence": "evidence/multi_sfm/acceptance.json",
        "validated_source_sha256": acceptance["source_sha256"],
    },
}, indent=2) + "\n", encoding="utf-8")
PY
(
    cd "$TARGET"
    find README.md MULTIROBOT_PLATFORM_DEVELOPMENT_PLAN.md RELEASE.json source_snapshot.tar.gz \
        baseline config docs evidence install scripts \
        -type f -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
)
chmod -R a-w "$TARGET/config" "$TARGET/baseline" "$TARGET/docs" "$TARGET/evidence" "$TARGET/install" \
    "$TARGET/scripts" "$TARGET/README.md" "$TARGET/MULTIROBOT_PLATFORM_DEVELOPMENT_PLAN.md" \
    "$TARGET/source_snapshot.tar.gz" "$TARGET/RELEASE.json" "$TARGET/SHA256SUMS"
chmod a-w "$TARGET"
echo "MULTIROBOT_RELEASE_OK id=$RELEASE_ID path=$TARGET"
