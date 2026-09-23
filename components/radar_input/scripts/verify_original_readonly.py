#!/usr/bin/env python3
"""Read-only verification of the original source tree and Git states."""
import hashlib
import json
import os
from pathlib import Path
import subprocess


def main():
    development = Path(__file__).resolve().parents[1]
    audit = development / "audit"
    record = json.loads((audit / "original_repositories.json").read_text())
    original = Path(record["workspace"])
    expected = json.loads((audit / "original_files.sha256.json").read_text())
    actual = {}
    for top in ("src", "scripts", "config"):
        for base, dirs, files in os.walk(original / top):
            dirs[:] = [d for d in dirs if d not in (".git", "__pycache__")]
            for name in files:
                path = Path(base) / name
                relative = str(path.relative_to(original))
                if path.is_symlink():
                    actual[relative] = {"symlink": os.readlink(path)}
                elif path.is_file():
                    digest = hashlib.sha256()
                    with path.open("rb") as stream:
                        for block in iter(lambda: stream.read(1024 * 1024), b""):
                            digest.update(block)
                    actual[relative] = digest.hexdigest()
    changes = [p for p in sorted(expected.keys() | actual.keys())
               if expected.get(p) != actual.get(p)]
    git_changes = []
    env = dict(os.environ, GIT_OPTIONAL_LOCKS="0")
    for repo in record["repositories"]:
        def git(*args):
            return subprocess.check_output(
                ["git", "-C", str(original / repo["path"]), *args], env=env
            ).decode()
        current = {
            "branch": git("branch", "--show-current").strip(),
            "commit": git("rev-parse", "HEAD").strip(),
            "status": git("status", "--porcelain=v1", "--untracked-files=all"),
        }
        for key, value in current.items():
            if value != repo[key]:
                git_changes.append({"path": repo["path"], "field": key})
        key = repo["path"].replace("/", "__")
        for label, args in (("unstaged", ("diff", "--binary")),
                            ("staged", ("diff", "--cached", "--binary"))):
            current_diff = subprocess.check_output(
                ["git", "-C", str(original / repo["path"]), *args], env=env
            )
            if current_diff != (audit / f"{key}.{label}.patch").read_bytes():
                git_changes.append({"path": repo["path"], "field": label})
    result = {
        "passed": not changes and not git_changes,
        "files_checked": len(actual),
        "repositories_checked": len(record["repositories"]),
        "changed_files": changes,
        "changed_git_states": git_changes,
        "scope": "src, scripts, config; Git branch, HEAD, status and both diffs; excludes logs/build/cache and __pycache__",
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
