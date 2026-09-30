#!/usr/bin/env python3
"""The private LightSFM copy must match its pinned source manifest."""
import hashlib
import json
from pathlib import Path
root=Path(__file__).resolve().parents[1]/'src/arena_multi_hunav_core/vendor/lightsfm'
manifest=json.loads((root/'SOURCE.json').read_text())
failures=[name for name,digest in manifest['files'].items() if hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest]
print(json.dumps({'passed':not failures,'commit':manifest['commit'],'files':len(manifest['files']),'failures':failures}))
raise SystemExit(bool(failures))
