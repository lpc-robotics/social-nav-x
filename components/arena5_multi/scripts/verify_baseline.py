#!/usr/bin/env python3
"""Fail closed when the protected Arena5 underlay changed after P0."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "baseline/stable_underlay.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "--no-optional-locks", "-C", str(repo), *args],
        text=True, capture_output=True, check=False,
    )
    return result.stdout.rstrip() if result.returncode == 0 else ""


def main() -> int:
    data = json.loads(MANIFEST.read_text())
    stable = Path(data["stable_workspace"])
    failures = []
    checks = 0
    for record in data["repositories"]:
        repo = stable / record["path"]
        checks += 2
        if git(repo, "rev-parse", "HEAD") != record["head"]:
            failures.append(f"repository HEAD changed: {record['path']}")
        status = git(repo, "status", "--porcelain=v1", "--untracked-files=all").splitlines()
        if status != record["status"]:
            failures.append(f"repository status changed: {record['path']}")
    for rel, expected in data["protected_files"].items():
        checks += 1
        path = stable / rel
        actual = sha256(path) if path.is_file() else None
        if actual != expected["sha256"]:
            failures.append(f"protected file changed: {rel}")
    result = {"passed": not failures, "checks": checks, "failures": failures}
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())

