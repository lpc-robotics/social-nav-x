#!/usr/bin/env python3
"""Compare paired Dijkstra and A* benchmark reports without changing DWB."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics


def summarize(document: dict) -> dict:
    robot_results = [
        result
        for round_result in document.get("results", [])
        for result in round_result.get("robots", {}).values()
    ]
    durations = [item["wall_duration_s"] for item in document.get("results", [])]
    paths = [item["path_length_m"] for item in robot_results]
    return {
        "passed": bool(document.get("passed")),
        "rounds_completed": int(document.get("rounds_completed", 0)),
        "robot_success_rate": (
            sum(bool(item.get("passed")) for item in robot_results) / len(robot_results)
            if robot_results else 0.0
        ),
        "mean_round_wall_s": statistics.fmean(durations) if durations else None,
        "mean_path_length_m": statistics.fmean(paths) if paths else None,
        "minimum_center_separation_m": document.get("minimum_center_separation_m"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dijkstra")
    parser.add_argument("astar")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    documents = {
        "dijkstra": json.loads(Path(args.dijkstra).read_text(encoding="utf-8")),
        "astar": json.loads(Path(args.astar).read_text(encoding="utf-8")),
    }
    if documents["dijkstra"].get("planner_algorithm") != "dijkstra":
        raise SystemExit("first report is not Dijkstra")
    if documents["astar"].get("planner_algorithm") != "astar":
        raise SystemExit("second report is not A*")
    if documents["dijkstra"].get("rounds_requested") != documents["astar"].get("rounds_requested"):
        raise SystemExit("reports do not contain the same number of paired rounds")
    if documents["dijkstra"].get("tasks_sha256") != documents["astar"].get("tasks_sha256"):
        raise SystemExit("reports were not generated from the same paired task file")
    report = {name: summarize(document) for name, document in documents.items()}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if all(item["passed"] for item in report.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
