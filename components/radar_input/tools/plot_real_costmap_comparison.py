"""Summarize and plot the recorded full Arena/HuNav Master Costmaps."""
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
AUDIT = ROOT / "audit"
RESOLUTION = 0.1
WIDTH_METERS = 15.0

fixed = json.loads((AUDIT / "real_pedestrian_costmap.json").read_text())
baseline = json.loads((AUDIT / "real_pedestrian_baseline.json").read_text())
robot = np.asarray(fixed["robot_initial"], dtype=float)
origin = robot - WIDTH_METERS / 2
extent = [origin[0], origin[0] + WIDTH_METERS,
          origin[1], origin[1] + WIDTH_METERS]


items = [
    ("Baseline: 7.5 s after departure",
     np.load(AUDIT / "real_pedestrian_baseline_final_master.npy"),
     baseline["marked"]["position"], baseline["final"]),
    ("Fixed: 2.0 s later",
     np.load(AUDIT / "real_pedestrian_cleared_master.npy"),
     fixed["marked"]["position"], fixed["cleared"]),
]

controlled = json.loads((AUDIT / "depth_clearing_comparison.json").read_text())
stream = json.loads((AUDIT / "live_clearing_stream.json").read_text())
summary = {
    "real_baseline": {
        "after_departure_sim_seconds": baseline["final"]["since_departure"],
        "movement_m": baseline["final"]["movement"],
        "old_lethal_cells": baseline["final"]["old_lethal_cells"],
        "old_max_cost": baseline["final"]["old_max_cost"],
    },
    "real_fixed": {
        "clearing_latency_sim_seconds": fixed["clearing_latency_sim_seconds"],
        "movement_m": fixed["cleared"]["movement"],
        "old_lethal_cells": fixed["cleared"]["old_lethal_cells"],
        "old_max_cost": fixed["cleared"]["old_max_cost"],
    },
    "controlled_final": {
        "baseline": controlled[-1]["masters"]["baseline"],
        "fixed": controlled[-1]["masters"]["fixed"],
    },
    "stream": {
        "pointcloud_points": stream["samples"][0]["points"],
        "sim_frequency": stream["sim_frequency"],
        "lidar_sim_frequency": stream["lidar_sim_frequency"],
        "all_frames_lidar_link": stream["all_frames_lidar_link"],
        "all_points_finite": stream["all_points_finite"],
        "all_ranges_safe": stream["all_ranges_safe"],
    },
}

fig, axes = plt.subplots(1, 2, figsize=(9.4, 4.5), constrained_layout=True)
for axis, (title, grid, old_position, record) in zip(axes, items):
    image = axis.imshow(grid, origin="lower", extent=extent, vmin=0, vmax=254,
                        cmap="magma", interpolation="nearest")
    circle = plt.Circle(old_position, 0.35, fill=False, color="cyan", linewidth=2)
    axis.add_patch(circle)
    axis.plot(*old_position, marker="+", color="cyan", markersize=9)
    lethal = record.get("old_lethal_cells", record.get("lethal_cells"))
    maximum = record.get("old_max_cost", record.get("max_cost"))
    axis.set_title(f"{title}\nold lethal={lethal}, max={maximum}")
    axis.set_xlabel("map x [m]")
    axis.set_ylabel("map y [m]")
    axis.set_xlim(old_position[0] - 1.2, old_position[0] + 1.2)
    axis.set_ylim(old_position[1] - 1.2, old_position[1] + 1.2)
fig.colorbar(image, ax=axes, label="Master Costmap cost", shrink=0.88)
fig.suptitle("Real Arena/HuNav pedestrian: old-position costmap evidence")
fig.savefig(AUDIT / "real_pedestrian_before_after.png", dpi=180)

summary["metadata"] = {"resolution": RESOLUTION, "robot_position": robot.tolist()}
(AUDIT / "real_pedestrian_comparison.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
