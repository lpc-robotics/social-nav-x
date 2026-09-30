#!/usr/bin/env python3
"""Sample CPU, resident memory, threads, and GPU memory for one registered run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import subprocess
import time

import psutil


def gpu_memory_by_pid() -> dict[int, float]:
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-compute-apps=pid,used_memory",
             "--format=csv,noheader,nounits"],
            check=True, capture_output=True, text=True, timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError):
        return {}
    values: dict[int, float] = {}
    for line in result.stdout.splitlines():
        fields = [item.strip() for item in line.split(",")]
        if len(fields) == 2 and all(item.isdigit() for item in fields):
            values[int(fields[0])] = float(fields[1])
    return values


def process_tree(root: psutil.Process) -> list[psutil.Process]:
    result = [root]
    try:
        result.extend(root.children(recursive=True))
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        pass
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    run_dir = Path(args.run_dir).resolve()
    pid = int((run_dir / "launch.pid").read_text(encoding="utf-8").strip())
    root = psutil.Process(pid)
    process_cache: dict[int, psutil.Process] = {}
    for process in process_tree(root):
        try:
            process_cache[process.pid] = process
            process.cpu_percent(None)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    samples = []
    started = time.monotonic()
    deadline = started + args.duration
    interrupted = False
    try:
        while time.monotonic() < deadline:
            discovered = process_tree(root)
            processes = []
            for process in discovered:
                cached = process_cache.get(process.pid)
                if cached is None:
                    process_cache[process.pid] = process
                    process.cpu_percent(None)
                    cached = process
                processes.append(cached)
            gpu = gpu_memory_by_pid()
            cpu = 0.0
            rss = 0
            threads = 0
            alive_pids = []
            for process in processes:
                try:
                    cpu += process.cpu_percent(None)
                    rss += process.memory_info().rss
                    threads += process.num_threads()
                    alive_pids.append(process.pid)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            samples.append({
                "wall_time": time.time(), "processes": len(alive_pids), "cpu_percent": cpu,
                "rss_mib": rss / 1024.0 / 1024.0, "threads": threads,
                "gpu_memory_mib": sum(gpu.get(pid, 0.0) for pid in alive_pids),
            })
            time.sleep(max(0.1, args.interval))
    except KeyboardInterrupt:
        interrupted = True


    def aggregate(field: str) -> dict:
        values = [float(sample[field]) for sample in samples]
        return {"mean": statistics.fmean(values), "max": max(values)} if values else {}

    report = {
        "duration_wall_s": time.monotonic() - started,
        "requested_duration_wall_s": args.duration, "interrupted": interrupted,
        "sample_count": len(samples),
        "cpu_percent": aggregate("cpu_percent"), "rss_mib": aggregate("rss_mib"),
        "threads": aggregate("threads"), "gpu_memory_mib": aggregate("gpu_memory_mib"),
        "samples": samples,
    }
    output = Path(args.output) if args.output else run_dir / "resource_usage.json"
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "samples"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
