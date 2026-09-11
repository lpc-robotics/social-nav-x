#!/usr/bin/env python3
"""Validate the five AB/BA paired DWB/MPC comparison trials."""

import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--evidence-dir", type=Path, default=Path("evidence/p5/comparison")
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    failures = []
    records = []
    expected_orders = {
        1: ("dwb", "mpc"),
        2: ("mpc", "dwb"),
        3: ("dwb", "mpc"),
        4: ("mpc", "dwb"),
        5: ("dwb", "mpc"),
    }
    for pair, methods in expected_orders.items():
        pair_records = []
        for order, method in enumerate(methods, start=1):
            path = args.evidence_dir / f"pair{pair}_{order}_{method}.json"
            if not path.is_file():
                failures.append(f"missing {path}")
                continue
            report = json.loads(path.read_text(encoding="utf-8"))
            prefix = f"pair {pair} order {order} {method}"
            if report.get("pair") != pair or report.get("order") != order:
                failures.append(f"{prefix}: metadata mismatch")
            if report.get("method") != method:
                failures.append(f"{prefix}: method/order mismatch")
            if report.get("pass") is not True:
                failures.append(f"{prefix}: probe gate failed")
            if report.get("action_status") != 4:
                failures.append(f"{prefix}: navigation did not succeed")
            if method == "mpc" and report.get("safety", {}).get(
                "minimum_clearance_lower_bound_m", -1
            ) < 0.30:
                failures.append(f"{prefix}: MPC clearance lower bound failed")
            if method == "mpc" and report.get("mpc_costmap_gate") is not True:
                failures.append(f"{prefix}: MPC costmap layer/footprint gate failed")
            if report.get("safety", {}).get("sampling_alignment_error_bound_m", 1) > 0.05:
                failures.append(f"{prefix}: sampling error bound failed")
            records.append(report)
            pair_records.append(report)
        if len(pair_records) == 2:
            if pair_records[0].get("target") != pair_records[1].get("target"):
                failures.append(f"pair {pair}: methods used different goals")
            first_meta = pair_records[0].get("run_metadata", {})
            second_meta = pair_records[1].get("run_metadata", {})
            if first_meta.get("config_sha256") != second_meta.get("config_sha256"):
                failures.append(f"pair {pair}: methods used different scene configs")
            if first_meta.get("gpu_index") != second_meta.get("gpu_index"):
                failures.append(f"pair {pair}: methods used different GPUs")
            if first_meta.get("physics_dt_s") != second_meta.get("physics_dt_s"):
                failures.append(f"pair {pair}: methods used different physics steps")
            if first_meta.get("ideal_chassis") != second_meta.get("ideal_chassis"):
                failures.append(f"pair {pair}: methods used different chassis modes")

    by_method = defaultdict(list)
    for report in records:
        by_method[report["method"]].append(report)
    method_summary = {}
    for method, items in sorted(by_method.items()):
        method_summary[method] = {
            "runs": len(items),
            "successes": sum(item.get("action_status") == 4 for item in items),
            "median_simulation_duration_s": statistics.median(
                item["simulation_duration_s"] for item in items
            ),
            "median_wall_duration_s": statistics.median(
                item["wall_duration_s"] for item in items
            ),
            "median_path_length_m": statistics.median(
                item["path_length_m"] for item in items
            ),
            "median_command_total_variation": statistics.median(
                item["command_total_variation"] for item in items
            ),
            "minimum_clearance_lower_bound_m": min(
                item["safety"]["minimum_clearance_lower_bound_m"] for item in items
            ),
        }
    config_hashes = {
        item.get("run_metadata", {}).get("config_sha256") for item in records
    }
    probe_hashes = {
        item.get("run_metadata", {}).get("probe_sha256") for item in records
    }
    if len(config_hashes) != 1:
        failures.append("comparison runs used inconsistent scene hashes")
    if len(probe_hashes) != 1:
        failures.append("comparison runs used inconsistent probe hashes")
    if len(records) != 10:
        failures.append(f"loaded {len(records)} of 10 comparison results")
    summary = {
        "gate": "PASS" if not failures else "FAIL",
        "design": "five fresh-start pairs ordered AB,BA,AB,BA,AB",
        "formal_result_count": len(records),
        "method_summary": method_summary,
        "config_sha256": next(iter(config_hashes)) if len(config_hashes) == 1 else None,
        "probe_sha256": next(iter(probe_hashes)) if len(probe_hashes) == 1 else None,
        "failures": failures,
    }
    rendered = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    print(rendered, end="")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    raise SystemExit(0 if not failures else 1)


if __name__ == "__main__":
    main()
