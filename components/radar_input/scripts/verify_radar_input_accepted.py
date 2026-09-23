#!/usr/bin/env python3
"""Verify the accepted radar-input tags, installed overlay and recovery bundles."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "baseline/radar-input-accepted-v1.json"


def run(*args: str, cwd: Path = ROOT) -> subprocess.CompletedProcess:
    return subprocess.run(args, cwd=cwd, capture_output=True, check=False)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(repo: Path, *args: str) -> str:
    result = run("git", "-C", str(repo), *args)
    return result.stdout.decode().strip() if result.returncode == 0 else ""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime-only", action="store_true")
    args = parser.parse_args()
    manifest = json.loads(MANIFEST.read_text())
    failures = []
    checks = 0

    def check(name: str, passed: bool) -> None:
        nonlocal checks
        checks += 1
        if not passed:
            failures.append(name)

    root_tag = manifest["tag"]
    root_commit = git(ROOT, "rev-list", "-n", "1", root_tag)
    check("root tag and current HEAD", bool(root_commit) and root_commit == git(ROOT, "rev-parse", "HEAD"))
    check(
        "root tag parent",
        bool(root_commit)
        and git(ROOT, "rev-parse", f"{root_commit}^") == manifest["root_parent_commit"],
    )
    tagged_manifest = run("git", "show", f"{root_tag}:baseline/radar-input-accepted-v1.json")
    check(
        "tagged acceptance manifest",
        tagged_manifest.returncode == 0 and tagged_manifest.stdout == MANIFEST.read_bytes(),
    )

    normalized = manifest["accepted_normalized_nav2"]
    repos = {
        "arena_isaac": ROOT / normalized["arena_isaac"]["worktree"],
        "simulation_setup": ROOT / normalized["simulation_setup"]["worktree"],
    }
    for name, repo in repos.items():
        expected = normalized[name]["commit"]
        check(f"{name} accepted tag", git(repo, "rev-list", "-n", "1", root_tag) == expected)
        check(f"{name} current HEAD", git(repo, "rev-parse", "HEAD") == expected)
        check(
            f"{name} clean tracked source",
            run("git", "-C", str(repo), "diff", "--quiet").returncode == 0
            and run("git", "-C", str(repo), "diff", "--cached", "--quiet").returncode == 0,
        )

    overlay = ROOT / normalized["overlay"]
    pairs = [
        (
            repos["arena_isaac"] / "arena_isaac/isaac_utils/graphs/sensors/normalized_scan.py",
            overlay / "arena_isaac/lib/python3.11/site-packages/isaac_utils/graphs/sensors/normalized_scan.py",
        ),
        (
            repos["arena_isaac"] / "arena_isaac/isaac_utils/scan_geometry.py",
            overlay / "arena_isaac/lib/python3.11/site-packages/isaac_utils/scan_geometry.py",
        ),
        (
            repos["simulation_setup"] / "entities/robots/jackal/model_params.yaml",
            overlay / "arena_simulation_setup/share/arena_simulation_setup/entities/robots/jackal/model_params.yaml",
        ),
    ]
    for source, installed in pairs:
        check(
            f"overlay matches {source.name}",
            source.is_file() and installed.is_file() and sha256(source) == sha256(installed),
        )

    baseline_args = ["scripts/verify_depth_clearing_baseline.py"]
    if args.runtime_only:
        baseline_args.append("--runtime-only")
    check("frozen depth-clearing baseline", run(*baseline_args).returncode == 0)

    bundle_data = manifest["offline_bundles"]
    bundle_dir = ROOT / bundle_data["directory"]
    if not args.runtime_only:
        bundle_repos = {
            "arena_isaac_delta.bundle": repos["arena_isaac"],
            "simulation_setup_delta.bundle": repos["simulation_setup"],
        }
        for filename, expected_hash in bundle_data["sha256"].items():
            path = bundle_dir / filename
            check(
                f"offline bundle {filename}",
                path.is_file()
                and sha256(path) == expected_hash
                and run(
                    "git", "-C", str(bundle_repos[filename]), "bundle", "verify", str(path)
                ).returncode == 0,
            )
        root_bundle = bundle_dir / bundle_data["root_bundle"]
        check(
            "offline root bundle",
            root_bundle.is_file()
            and run("git", "bundle", "verify", str(root_bundle)).returncode == 0
            and root_tag
            in run("git", "bundle", "list-heads", str(root_bundle)).stdout.decode(),
        )

    print(json.dumps({"passed": not failures, "mode": "runtime" if args.runtime_only else "full", "checks": checks, "failures": failures}, ensure_ascii=False, indent=2))
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
