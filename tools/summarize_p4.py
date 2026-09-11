#!/usr/bin/env python3
"""Validate and summarize the 40 formal P4 result files.

Files carrying calibration, rejected, or superseded suffixes are deliberately
outside this manifest.  A missing or inconsistent formal result makes the
command fail instead of silently reducing the sample count.
"""

import argparse
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path


SCENARIOS = (
    "crossing",
    "head_on",
    "same_direction",
    "multi_crossing",
    "stop_turn",
    "id_change",
    "backlog",
    "six_behaviors",
)
REPETITIONS = range(1, 6)
COMMON_HASH_KEYS = (
    "controller_source_sha256",
    "controller_config_sha256",
    "nav2_overrides_sha256",
    "probe_sha256",
    "launch_sha256",
)


def percentile(values, fraction):
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-dir", type=Path, default=Path("evidence/p4"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    failures = []
    records = []
    common_hashes = defaultdict(set)
    config_hashes = defaultdict(set)
    modes = Counter()
    wait_reasons = Counter()

    for repetition in REPETITIONS:
        for scenario in SCENARIOS:
            path = args.evidence_dir / f"{scenario}_run{repetition}.json"
            if not path.is_file():
                failures.append(f"missing {path}")
                continue
            try:
                report = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as error:
                failures.append(f"unreadable {path}: {error}")
                continue

            prefix = f"{scenario} run {repetition}"
            checks = {
                "scenario": report.get("scenario") == scenario,
                "pass": report.get("pass") is True,
                "action succeeded": report.get("action_status") == 4,
                "finite output": report.get("outputs_finite") is True,
                "BT timeout 500 ms": report.get("bt_default_server_timeout_ms") == 500,
                "progress allowance 30 s": report.get(
                    "progress_movement_time_allowance_s"
                )
                == 30.0,
                "measurement error <= 0.05 m": report.get("safety", {}).get(
                    "sampling_alignment_error_bound_m", math.inf
                )
                <= 0.05,
                "clearance lower bound >= 0.30 m": report.get("safety", {}).get(
                    "minimum_clearance_lower_bound_m", -math.inf
                )
                >= 0.30,
                "solver samples": report.get("solver_samples", 0) > 0,
            }
            if scenario == "id_change":
                checks["ID change injected"] = report.get("id_change_injections", 0) >= 1
                checks["ID change observed"] = (
                    report.get("safety", {}).get("id_set_change_count", 0) >= 1
                )
            if scenario == "backlog":
                checks["backlog injected"] = report.get("backlog_injections", 0) >= 1
            for label, passed in checks.items():
                if not passed:
                    failures.append(f"{prefix}: {label}")

            metadata = report.get("run_metadata", {})
            for key in COMMON_HASH_KEYS:
                value = metadata.get(key)
                if not value:
                    failures.append(f"{prefix}: missing {key}")
                else:
                    common_hashes[key].add(value)
            config_hashes[scenario].add(metadata.get("config_sha256"))
            modes.update(report.get("controller_mode_counts", {}))
            wait_reasons.update(report.get("controller_wait_reason_counts", {}))
            records.append(report)

    for key, values in common_hashes.items():
        if len(values) != 1:
            failures.append(f"formal runs use {len(values)} values for {key}")
    for scenario, values in config_hashes.items():
        if len(values) != 1 or None in values:
            failures.append(f"{scenario} uses inconsistent scenario config hashes")

    expected_count = len(SCENARIOS) * len(REPETITIONS)
    if len(records) != expected_count:
        failures.append(f"loaded {len(records)} of {expected_count} formal results")

    if records:
        cycle_p95_values = [item["cycle_ms_p95"] for item in records]
        cycle_max_values = [item["cycle_ms_max"] for item in records]
        solver_p95_values = [item["solver_ms_p95"] for item in records]
        solver_max_values = [item["solver_ms_max"] for item in records]
        lower_values = [
            item["safety"]["minimum_clearance_lower_bound_m"] for item in records
        ]
        error_values = [
            item["safety"]["sampling_alignment_error_bound_m"] for item in records
        ]
        summary = {
            "gate": "PASS" if not failures else "FAIL",
            "formal_result_count": len(records),
            "expected_formal_result_count": expected_count,
            "scenarios": list(SCENARIOS),
            "repetitions_per_scenario": len(REPETITIONS),
            "all_action_status_succeeded": all(
                item.get("action_status") == 4 for item in records
            ),
            "all_outputs_finite": all(item.get("outputs_finite") for item in records),
            "minimum_clearance_lower_bound_m": min(lower_values),
            "maximum_sampling_alignment_error_bound_m": max(error_values),
            "cycle_ms": {
                "maximum_run_p95": max(cycle_p95_values),
                "p95_of_run_maxima": percentile(cycle_max_values, 0.95),
                "maximum": max(cycle_max_values),
            },
            "solver_ms": {
                "maximum_run_p95": max(solver_p95_values),
                "p95_of_run_maxima": percentile(solver_max_values, 0.95),
                "maximum": max(solver_max_values),
            },
            "total_solver_samples": sum(item["solver_samples"] for item in records),
            "total_wall_duration_s": sum(item["wall_duration_s"] for item in records),
            "total_simulation_duration_s": sum(
                item["simulation_duration_s"] for item in records
            ),
            "common_hashes": {
                key: next(iter(values)) if len(values) == 1 else sorted(values)
                for key, values in sorted(common_hashes.items())
            },
            "scenario_config_hashes": {
                key: next(iter(values)) if len(values) == 1 else sorted(values)
                for key, values in sorted(config_hashes.items())
            },
            "controller_mode_counts": dict(sorted(modes.items())),
            "controller_wait_reason_counts": dict(sorted(wait_reasons.items())),
            "failures": failures,
        }
    else:
        summary = {
            "gate": "FAIL",
            "formal_result_count": 0,
            "expected_formal_result_count": expected_count,
            "failures": failures,
        }

    rendered = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    print(rendered, end="")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    raise SystemExit(0 if summary["gate"] == "PASS" else 1)


if __name__ == "__main__":
    main()
