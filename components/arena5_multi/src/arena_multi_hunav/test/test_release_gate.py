"""A release must not inherit acceptance after code or evidence changes."""
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


@pytest.fixture
def gate(tmp_path, monkeypatch):
    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location("release_gate", root / "scripts/verify_multi_sfm_release.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "ROOT", tmp_path)
    monkeypatch.setattr(module, "source_digest", lambda: "validated-source")
    manifest = {"passed": True, "source_sha256": "validated-source", "reports": {}}
    for name in module.REQUIRED:
        data = {"passed": True}
        if name == "six_runtime_30min": data["duration_wall_s"] = 1800
        if name == "cpu_interactions": data["trials"] = [{}] * 120
        if name == "six_navigation": data["rounds_completed"] = 10
        path = tmp_path / (name + ".json")
        path.write_text(json.dumps(data))
        manifest["reports"][name] = {"path": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    return module, manifest, tmp_path / "acceptance.json"


@pytest.mark.parametrize("failure", [None, "source", "missing", "tampered", "failed", "short_run"])
def test_release_requires_current_complete_evidence(gate, failure):
    module, manifest, path = gate
    if failure == "source": manifest["source_sha256"] = "old-source"
    if failure == "missing": del manifest["reports"]["legacy_six"]
    if failure in ("tampered", "failed", "short_run"):
        name = "six_runtime_30min"
        report = path.parent / manifest["reports"][name]["path"]
        data = json.loads(report.read_text())
        if failure == "short_run": data["duration_wall_s"] = 1799
        else: data["passed"] = False
        report.write_text(json.dumps(data))
        if failure != "tampered":
            manifest["reports"][name]["sha256"] = hashlib.sha256(report.read_bytes()).hexdigest()
    path.write_text(json.dumps(manifest))
    assert module.verify(path)["passed"] is (failure is None)
