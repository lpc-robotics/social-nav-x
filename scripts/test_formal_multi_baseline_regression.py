#!/usr/bin/env python3
"""Unmodified shared/overlay six-behavior and Nav2 regression, fresh run each."""

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time

from test_formal_multi_simulation_matrix import ROOT, BASE, ERRORS, guard, stop_owned


def interval_metrics(text, previous=None):
    # Shared compat predates the steady_* fields. Derive the same independent
    # report-window rates from its ROS wall stamps and increasing counters.
    reports = []
    for line in text.splitlines():
        if "SIX_BEHAVIORS_RUNNING" not in line:
            continue
        stamp = re.search(r"\[([0-9]+\.[0-9]+)\]", line)
        values = dict(re.findall(r"\b(compute|updates|max_dt|lag)=([0-9.]+)", line))
        if stamp and len(values) == 4:
            reports.append({**{k: float(v) for k, v in values.items()}, "wall_stamp": float(stamp.group(1))})
    if len(reports) < 2:
        return None
    a, b = reports[-2:]
    dt = b["wall_stamp"] - a["wall_stamp"]
    if dt <= 0:
        return None
    b.update(steady_compute=(b["compute"] - a["compute"]) / dt,
             steady_display=(b["updates"] - a["updates"]) / dt)
    if b["steady_compute"] < 10 or b["steady_display"] < 4.5 or b["max_dt"] > .026 or b["lag"] > .1:
        return None
    if previous and any(b[k] <= previous[k] for k in ("compute", "updates")):
        return None
    return b


def healthy(process, path, previous=None):
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        text = path.read_text(errors="replace")
        if ERRORS.search(text) or process.poll() is not None:
            raise RuntimeError(f"baseline launch unhealthy: {path}")
        values = interval_metrics(text, previous)
        if values and "SIX_BEHAVIORS_READY pedestrians=6 behavior_types=1,2,3,4,5,6" in text:
            return values
        time.sleep(1)
    raise RuntimeError(f"baseline runtime gates timed out: {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gpu", type=int, default=1)
    parser.add_argument("--domain-base", type=int, default=210)
    args = parser.parse_args()
    if not 0 <= args.domain_base <= 228 or args.gpu < 0:
        parser.error("domain/GPU out of range")
    root = ROOT / "logs/formal_multi_baseline" / (time.strftime("%Y%m%d_%H%M%S") + f"_pid{os.getpid()}")
    root.mkdir(parents=True)
    print(f"FORMAL_MULTI_BASELINE_DIR={root}", flush=True)
    guard(root / "protected_before.txt")
    evidence = []
    for mode in ("shared", "overlay"):
        for test in ("six", "nav2"):
            case = root / f"{mode}_{test}"
            case.mkdir()
            env = os.environ.copy()
            for key in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH", "PYTHONPATH", "LD_LIBRARY_PATH"):
                env.pop(key, None)
            env.update(ROS_DOMAIN_ID=str(args.domain_base + len(evidence)), GPU_ID=str(args.gpu),
                       NAVIGATION="true" if test == "nav2" else "false", LIVESTREAM="false", DRL_VO_GUI="false",
                       ARENA_SIX_BEHAVIORS_USE_OVERLAY="true" if mode == "overlay" else "false",
                       ARENA_SIX_BEHAVIORS_OVERLAY_ROOT=str(ROOT / ".colcon-formal-phase4"))
            executable = "verify_six_behaviors" if test == "six" else "verify_runtime"
            marker = "SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6" if test == "six" else "SMOKE_NAVIGATION_OK"
            process = None
            print(f"FORMAL_MULTI_BASELINE_START mode={mode} test={test}", flush=True)
            try:
                with (case / "prefix.log").open("w") as stream:
                    subprocess.run([str(BASE / "scripts/run_six_behaviors.sh"), "--check-runtime-only"],
                                   env=env, stdout=stream, stderr=subprocess.STDOUT, check=True)
                log = case / "launch.log"
                with log.open("w") as stream:
                    process = subprocess.Popen([str(BASE / "scripts/run_six_behaviors.sh"), "foxglove:=false"],
                        env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                    before = healthy(process, log)
                    shell = 'source "$1/scripts/env.sh"\nif [[ "$2" == overlay ]]; then source "$3/install/local_setup.bash"; fi\nros2 run arena_humble_compat "$4"'
                    with (case / "verifier.log").open("w") as output:
                        subprocess.run(["bash", "-c", shell, "baseline-verifier", str(BASE), mode,
                            str(ROOT / ".colcon-formal-phase4"), executable], env=env, stdout=output,
                            stderr=subprocess.STDOUT, check=True, timeout=300)
                    assert marker in (case / "verifier.log").read_text(), "missing baseline success marker"
                    post_boundary = interval_metrics(log.read_text(errors="replace")) or before
                    after = healthy(process, log, post_boundary)
                stop_owned(process)
                process = None
                assert not ERRORS.search(log.read_text(errors="replace")), "post-cleanup baseline error"
                result = {"mode": mode, "test": test, "before": before, "after": after}
                evidence.append(result)
                (case / "evidence.json").write_text(json.dumps(result, indent=2) + "\n")
                print("FORMAL_MULTI_BASELINE_CASE_OK " + json.dumps(result, sort_keys=True), flush=True)
            finally:
                if process is not None:
                    stop_owned(process)
                guard(case / "protected_after.txt")
    guard(root / "protected_after.txt")
    (root / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print("FORMAL_MULTI_BASELINE_OK cases=4", flush=True)


if __name__ == "__main__":
    main()
