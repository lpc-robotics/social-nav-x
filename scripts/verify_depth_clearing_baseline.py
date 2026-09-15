#!/usr/bin/env python3
"""Verify the frozen depth-clearing source, runtime install and recovery archives."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "baseline/depth-clearing-v1.json"
ARCHIVE = ROOT / "logs/baselines/depth-clearing-v1"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(repo: Path, *args: str, binary: bool = False):
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout if binary else result.stdout.decode().strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--runtime-only",
        action="store_true",
        help="skip hashing and verifying the large offline bundles",
    )
    args = parser.parse_args()
    manifest = json.loads(MANIFEST.read_text())
    failures = []
    checks = []

    isaac_guard = manifest["environment_guard"]["isaac_sim_source"]
    isaac_repo = ROOT / isaac_guard["path"]
    isaac_head = git(isaac_repo, "rev-parse", "HEAD")
    isaac_status = git(isaac_repo, "status", "--porcelain=v1", "--untracked-files=all")
    head_ok = isaac_head == isaac_guard["commit"]
    status_ok = isaac_status == isaac_guard["expected_status"]
    checks.append({"check": "isaac_source_head", "passed": head_ok})
    checks.append({"check": "isaac_source_status", "passed": status_ok})
    if not head_ok or not status_ok:
        failures.append(
            "Isaac Sim source guard failed: "
            f"head={isaac_head!r}, status={isaac_status!r}"
        )

    conda_guard = manifest["environment_guard"]["conda_history"]
    conda_history = Path(conda_guard["path"])
    conda_ok = conda_history.is_file() and sha256(conda_history) == conda_guard["sha256"]
    checks.append({"check": "isaac_conda_history", "passed": conda_ok})
    if not conda_ok:
        failures.append("Isaac conda environment history changed")

    for relative, expected in manifest["repositories"].items():
        repo = ROOT / relative
        actual = git(repo, "rev-list", "-n", "1", manifest["tag"])
        ok = actual == expected
        checks.append({"check": "tag", "path": relative, "passed": ok})
        if not ok:
            failures.append(f"tag target {relative}: expected {expected}, got {actual}")

        if relative != ".":
            head = git(repo, "rev-parse", "HEAD")
            ok = head == expected
            checks.append({"check": "stable_head", "path": relative, "passed": ok})
            if not ok:
                failures.append(f"stable source HEAD {relative}: expected {expected}, got {head}")

            status = git(repo, "status", "--porcelain=v1", "--untracked-files=all")
            expected_status = (
                "M configs/nav2/nav2.yaml"
                if relative == "src/arena/simulation-setup"
                else ""
            )
            ok = status == expected_status
            checks.append({"check": "stable_status", "path": relative, "passed": ok})
            if not ok:
                failures.append(f"stable source status changed in {relative}: {status!r}")

    patch_path = ARCHIVE / "user-state/simulation-setup-before-laserscan.patch"
    current_patch = git(
        ROOT / "src/arena/simulation-setup", "diff", "--binary", binary=True
    )
    ok = hashlib.sha256(current_patch).hexdigest() == sha256(patch_path)
    checks.append({"check": "preserved_simulation_setup_diff", "passed": ok})
    if not ok:
        failures.append("simulation-setup working diff no longer matches its protected copy")

    for name, expected in manifest["preserved_user_state"].items():
        path = ARCHIVE / "user-state" / name
        ok = path.is_file() and sha256(path) == expected
        checks.append({"check": "preserved_user_state", "path": name, "passed": ok})
        if not ok:
            failures.append(f"preserved user-state artifact failed: {name}")

    installed = {
        "src/arena-isaac/arena_isaac/isaac_utils/graphs/sensors/depth_clearing.py": (
            "install/arena_isaac/lib/python3.11/site-packages/"
            "isaac_utils/graphs/sensors/depth_clearing.py"
        ),
        "src/arena-isaac/arena_isaac/isaac_utils/clearing_geometry.py":
            "install/arena_isaac/lib/python3.11/site-packages/isaac_utils/clearing_geometry.py",
        "src/arena-isaac/arena_isaac/isaac_utils/graphs/sensors/lidar.py":
            "install/arena_isaac/lib/python3.11/site-packages/isaac_utils/graphs/sensors/lidar.py",
        "src/arena-isaac/arena_isaac/arena_isaac/run_isaacsim.py":
            "install/arena_isaac/lib/python3.11/site-packages/arena_isaac/run_isaacsim.py",
    }
    for source, expected in manifest["runtime_files"].items():
        source_path = ROOT / source
        install_path = ROOT / installed[source]
        source_ok = source_path.is_file() and sha256(source_path) == expected
        install_ok = install_path.is_file() and sha256(install_path) == expected
        checks.append({"check": "stable_runtime_source", "path": source, "passed": source_ok})
        checks.append(
            {
                "check": "stable_runtime_install",
                "path": installed[source],
                "passed": install_ok,
            }
        )
        if not source_ok or not install_ok:
            failures.append(f"stable runtime source/install mismatch: {source}")

    if not args.runtime_only:
        for name, expected in manifest["bundles"].items():
            path = ARCHIVE / "bundles" / name
            hash_ok = path.is_file() and sha256(path) == expected
            verify_ok = False
            if hash_ok:
                result = subprocess.run(
                    ["git", "bundle", "verify", str(path)],
                    cwd=ROOT,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                verify_ok = result.returncode == 0
            ok = hash_ok and verify_ok
            checks.append({"check": "offline_bundle", "path": name, "passed": ok})
            if not ok:
                failures.append(f"offline bundle failed: {name}")

    result = {
        "passed": not failures,
        "mode": "runtime" if args.runtime_only else "full",
        "checks": len(checks),
        "failures": failures,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
