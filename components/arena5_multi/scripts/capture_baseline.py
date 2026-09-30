#!/usr/bin/env python3
"""Capture the non-destructive Arena5 underlay baseline as machine-readable evidence."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
from datetime import datetime


ROOT = Path(__file__).resolve().parents[1]
STABLE = Path(os.environ.get("ARENA_STABLE_WS", "/home/lpc/workspace/arena5_ws"))
OUTPUT = ROOT / "baseline/stable_underlay.json"


def run(*args: str, cwd: Path | None = None) -> tuple[int, str, str]:
    result = subprocess.run(args, cwd=cwd, text=True, capture_output=True, check=False)
    return result.returncode, result.stdout.rstrip(), result.stderr.rstrip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_record(repo: Path) -> dict:
    _, head, _ = run("git", "--no-optional-locks", "-C", str(repo), "rev-parse", "HEAD")
    _, branch, _ = run("git", "--no-optional-locks", "-C", str(repo), "branch", "--show-current")
    _, status, _ = run(
        "git", "--no-optional-locks", "-C", str(repo), "status", "--porcelain=v1", "--untracked-files=all"
    )
    return {"path": str(repo.relative_to(STABLE)), "head": head, "branch": branch, "status": status.splitlines()}


def main() -> int:
    repos = []
    for git_dir in sorted((STABLE / "src").glob("**/.git")):
        repos.append(git_record(git_dir.parent))

    protected_rel = [
        "scripts/run_six_behaviors.sh",
        "scripts/run_six_behaviors_dwb_08.sh",
        "scripts/run_six_behaviors_mpc.sh",
        "src/arena/simulation-setup/configs/nav2/nav2.yaml",
        "install/arena_simulation_setup/share/arena_simulation_setup/configs/nav2/nav2.yaml",
        "src/arena-isaac/arena_isaac/arena_isaac/run_isaacsim.py",
        "install/arena_isaac/lib/python3.11/site-packages/arena_isaac/run_isaacsim.py",
        "optional/mpc/releases/20260920-94f4de6/RELEASE",
    ]
    files = {}
    for rel in protected_rel:
        path = STABLE / rel
        files[rel] = {"exists": path.is_file(), "sha256": sha256(path) if path.is_file() else None}

    _, nav2_version, _ = run(
        "python3", "-c",
        "import xml.etree.ElementTree as E; print(E.parse('/home/lpc/workspace/arena5_ws/.conda/arena_ros/share/nav2_core/package.xml').getroot().findtext('version'))",
    )
    _, isaac_version, _ = run(
        "/home/lpc/miniforge3/envs/isaaclab/bin/python", "-c",
        "from importlib.metadata import version; print(version('isaacsim'))",
    )
    document = {
        "schema_version": 1,
        "captured_at": datetime.now().astimezone().isoformat(),
        "stable_workspace": str(STABLE),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "versions": {"ros": "humble", "nav2": nav2_version, "isaacsim": isaac_version},
        "repositories": repos,
        "protected_files": files,
        "normalized_scan_source": {
            "workspace": "/home/lpc/bk/arena5_ws_costmap_fix",
            "accepted_commit": "6709da14af280444ac90b616a800439de22b1869",
            "acceptance_manifest": "baseline/radar-input-accepted-v1.json",
        },
        "notes": [
            "The stable workspace root is not a Git repository.",
            "Source and installed Nav2 parameter hashes are intentionally recorded separately.",
            "Status lines are protected user state and must not be cleaned or reset.",
        ],
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n")
    print(OUTPUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

