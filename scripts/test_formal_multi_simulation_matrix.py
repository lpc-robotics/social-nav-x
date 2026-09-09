#!/usr/bin/env python3
"""Run isolated Phase 4 GPU cases, replay each trace and verify guard hashes."""

import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
BASE = Path("/home/lpc/workspace/arena5_ws")
MANIFEST = ROOT / "docs/formal_social_automata/phase4_baseline.sha256"
CASES = ("formation", "near", "crossing", "asymmetric", "fast", "disabled")
ERRORS = re.compile(r"Traceback|process has died|formal multi (compute|telemetry) failed|"
                    r"Isaac pedestrian update (failed|returned errors)|"
                    r"HuNav compute (failed|returned an invalid response)|non-finite")


def runtime(text, previous=None):
    lines = [line for line in text.splitlines() if "SIX_BEHAVIORS_RUNNING" in line]
    if not lines:
        return None
    values = dict(re.findall(r"\b(compute|updates|steady_compute|steady_display|max_dt|lag)=([0-9.]+)", lines[-1]))
    if len(values) != 6:
        return None
    values = {key: float(value) for key, value in values.items()}
    if values["steady_compute"] < 10 or values["steady_display"] < 4.5 or values["max_dt"] > .026 or values["lag"] > .100:
        return None
    if previous and any(values[key] <= previous[key] for key in ("compute", "updates")):
        return None
    return values


def stop_owned(process):
    # Only the new session owned by this matrix; never kill by process name.
    for sig, seconds in ((signal.SIGINT, 20), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
        try:
            if sig == signal.SIGINT:
                # ros2 launch forwards SIGINT to its children. Signalling the
                # whole group here would deliver it twice and can turn a
                # healthy shared node's cleanup into exit -2/KeyboardInterrupt.
                process.send_signal(sig)
            else:
                os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(seconds)
            # Launch normally reaps all children. Send TERM to any remaining
            # members of this same, still-owned process group on the next pass.
            os.killpg(process.pid, 0)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            if process.poll() is not None:
                break


def await_health(process, path, *, previous=None, ready=False, timeout=180):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        text = path.read_text(errors="replace")
        if ERRORS.search(text):
            raise RuntimeError("launch error: " + next(line for line in text.splitlines() if ERRORS.search(line)))
        if process.poll() is not None:
            raise RuntimeError(f"launch exited {process.returncode}; inspect {path}")
        metrics = runtime(text, previous)
        if metrics and (not ready or "FORMAL_SOCIAL_MULTI_BRIDGE_READY" in text):
            return metrics
        time.sleep(1)
    raise RuntimeError(f"runtime readiness/freshness gates failed: {path}")


def guard(log):
    result = subprocess.run(["sha256sum", "--check", str(MANIFEST)], cwd=BASE,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    log.write_text(result.stdout)
    result.check_returncode()


def run_case(root, scenario, round_index, domain, gpu):
    case_dir = root / f"{scenario}_round{round_index}_domain{domain}"
    case_dir.mkdir()
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(domain), GPU_ID=str(gpu), NAVIGATION="false",
               FORMAL_MULTI_ENABLED="false" if scenario == "disabled" else "true",
               FORMAL_MULTI_SHARED_EVENTS="true", DRL_VO_GUI="false")
    launch_args = [str(ROOT / "scripts/run_formal_social_multi_demo.sh"),
                   "headless:=true", "livestream:=false", "foxglove:=false"]
    if scenario == "asymmetric":
        launch_args.append("robot_y:=2.4")
    launch_log = case_dir / "launch.log"
    print(f"FORMAL_MULTI_CASE_START scenario={scenario} round={round_index} domain={domain}", flush=True)
    process = None
    try:
        with launch_log.open("w") as stream:
            process = subprocess.Popen(launch_args, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            before = await_health(process, launch_log, ready=True)
            with (case_dir / "verifier.log").open("w") as output:
                result = subprocess.run(["python", "-m", "formal_social_behavior.multi_agent.scenario_verifier",
                    "--ros-args", "-p", f"scenario:={scenario}", "-p", f"round:={round_index}"],
                    env=env, stdout=output, stderr=subprocess.STDOUT, timeout=300)
            result.check_returncode()
            # A report seen during the action is not a post-action interval.
            post_boundary = runtime(launch_log.read_text(errors="replace")) or before
            after = await_health(process, launch_log, previous=post_boundary, timeout=90)
            match = re.search(r"FORMAL_SOCIAL_MULTI_RUN_DIR=(\S+)", launch_log.read_text())
            if not match:
                raise RuntimeError("missing run manifest path")
            run_dir = Path(match.group(1))
        stop_owned(process)
        process = None
        text = launch_log.read_text(errors="replace")
        if ERRORS.search(text):
            raise RuntimeError("post-cleanup launch health failed")
        raw_resets = text.count("=== RESET AGENTS SERVICE CALLED ===")
        if scenario == "disabled":
            if raw_resets or (run_dir / "steps.jsonl").exists():
                raise RuntimeError("disabled mode produced resets/formal trace")
            commits = 0
        else:
            from formal_social_behavior.multi_agent.replay import verify_replay
            with (run_dir / "steps.jsonl").open() as stream:
                commits = verify_replay((ROOT / "src/formal_social_behavior/config/formal_social_multi_automata.yaml").read_bytes(), stream)
            frames = [json.loads(line) for line in (run_dir / "steps.jsonl").read_text().splitlines()]
            if raw_resets != frames[-1]["reset_count"] or raw_resets > 4:
                raise RuntimeError("uncommitted reset/reset storm")
            formed = [e for frame in frames for e in frame["shared_events"] if e["name"] == "SOCIAL_SPACE_FORMED"]
            if len(formed) != 1:
                raise RuntimeError("formation missing or repeated unexpectedly")
        evidence = {"scenario": scenario, "round": round_index, "domain": domain, "gpu": gpu,
                    "before": before, "after": after, "raw_resets": raw_resets,
                    "replayed_commits": commits, "run_dir": str(run_dir)}
        (case_dir / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n")
        print("FORMAL_MULTI_CASE_OK " + json.dumps(evidence, sort_keys=True), flush=True)
        return evidence
    finally:
        if process is not None:
            stop_owned(process)
        guard(case_dir / "protected_after.txt")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--rounds", type=int, default=2)
    parser.add_argument("--round-start", type=int, default=1)
    parser.add_argument("--domain-base", type=int, default=180)
    parser.add_argument("--gpu", type=int, default=1)
    args = parser.parse_args()
    count = len(args.scenarios) * args.rounds
    if args.rounds < 1 or args.round_start < 1 or not 0 <= args.domain_base <= 232 - count or args.gpu < 0:
        parser.error("invalid rounds/domain/GPU range")
    root = ROOT / "logs/formal_multi_acceptance" / (time.strftime("%Y%m%d_%H%M%S") + f"_pid{os.getpid()}")
    root.mkdir(parents=True)
    print(f"FORMAL_MULTI_MATRIX_DIR={root}", flush=True)
    guard(root / "protected_before.txt")
    evidence = []
    for scenario in args.scenarios:
        for round_index in range(args.round_start, args.round_start + args.rounds):
            evidence.append(run_case(root, scenario, round_index, args.domain_base + len(evidence), args.gpu))
    guard(root / "protected_after.txt")
    (root / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(f"FORMAL_MULTI_MATRIX_OK cases={len(evidence)} rounds={args.rounds}", flush=True)


if __name__ == "__main__":
    main()
