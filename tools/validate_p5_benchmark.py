#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = list(csv.DictReader(args.csv.open(encoding="utf-8")))
    selected = [
        row
        for row in rows
        if row["layout"] == "fixed_masked" and row["load"] == "maximum"
    ]
    failures = []
    if len(selected) != 3:
        failures.append(f"expected 3 fixed_masked maximum rows, got {len(selected)}")
    for row in selected:
        condition = row["condition"]
        if int(row["iterations"]) != 1000:
            failures.append(f"{condition}: iterations != 1000")
        if condition in ("feasible", "critical"):
            if int(row["success"]) != 1000 or int(row["timeout"]) != 0:
                failures.append(f"{condition}: not all solves succeeded")
            if float(row["warm_p99_ms"]) > 60.0:
                failures.append(f"{condition}: core p99 exceeds 60 ms")
        elif condition == "infeasible":
            if int(row["success"]) != 0 or int(row["timeout"]) != 1000:
                failures.append("infeasible: fail-safe timeout classification mismatch")
            if float(row["warm_p99_ms"]) > 90.0:
                failures.append("infeasible: core p99 exceeds plugin commit budget")
    report = {
        "gate": "PASS" if not failures else "FAIL",
        "input": str(args.csv.resolve()),
        "fixed_masked_maximum_rows": selected,
        "failures": failures,
    }
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    print(rendered, end="")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered, encoding="utf-8")
    raise SystemExit(0 if not failures else 1)


if __name__ == "__main__":
    main()
