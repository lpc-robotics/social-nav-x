"""Relocate the copied install tree; never follows symlinks into the original."""
import os
from pathlib import Path

root = Path(__file__).resolve().parents[1]
assert root.name == "arena5_ws_costmap_fix"
old = b"/home/lpc/workspace/arena5_ws"
new = str(root).encode()
counts = {"symlinks": 0, "text_files": 0}
for base, dirs, files in os.walk(root / "install", followlinks=False):
    for name in dirs + files:
        path = Path(base) / name
        if path.is_symlink():
            target = os.readlink(path)
            if target.startswith(old.decode() + "/"):
                path.unlink()
                path.symlink_to(str(root) + target[len(old):])
                counts["symlinks"] += 1
        elif path.is_file():
            data = path.read_bytes()
            if old in data and b"\0" not in data:
                path.write_bytes(data.replace(old + b"/", new + b"/"))
                counts["text_files"] += 1
print(counts)
