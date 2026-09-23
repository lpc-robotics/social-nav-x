#!/usr/bin/env python3
"""Independent NumPy/Python evaluation of the deterministic C++ fixture."""

import math
import os
import subprocess

import numpy as np


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def step(state, control, dt):
    x, y, yaw = state
    linear, angular = control
    return np.array(
        [x + dt * linear * math.cos(yaw), y + dt * linear * math.sin(yaw), yaw + dt * angular]
    )


def clearance(state, obstacle, safe):
    ox, oy, major, minor, yaw = obstacle
    dx, dy = state[0] - ox, state[1] - oy
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    regularizer = 1.0e-12
    normalized = math.sqrt(
        (local_x / major) ** 2 + (local_y / minor) ** 2 + regularizer
    ) - math.sqrt(regularizer)
    return minor * normalized - minor - safe


def main():
    executable = os.environ["MODEL_FIXTURE"]
    output = subprocess.check_output([executable], text=True)
    actual = {line.split("=", 1)[0]: float(line.split("=", 1)[1]) for line in output.splitlines()}

    dt = 0.1
    gamma = 0.2
    safe = 0.3
    initial = np.array([0.2, -0.1, math.pi - 0.01])
    measured = np.array([0.05, -0.1])
    references = np.array(
        [
            [0.2, -0.1, -math.pi + 0.01],
            [0.18, -0.1, -math.pi + 0.02],
            [0.16, -0.101, -math.pi + 0.03],
            [0.14, -0.102, -math.pi + 0.04],
        ]
    )
    controls = np.array([[0.10, 0.05], [0.12, 0.08], [0.11, -0.02]])
    obstacles = np.array(
        [
            [1.0, 0.5, 0.45, 0.25, 0.4],
            [0.99, 0.5, 0.45, 0.25, 0.4],
            [0.98, 0.5, 0.45, 0.25, 0.4],
            [0.97, 0.5, 0.45, 0.25, 0.4],
        ]
    )
    states = [initial]
    for control in controls:
        states.append(step(states[-1], control, dt))
    states = np.array(states)
    states[2, 0] += 2.5e-5
    slack = np.array([0.01, 0.02, 0.03])

    objective = 0.0
    max_dynamics = 0.0
    max_bounds = 0.0
    max_accel = 0.0
    previous = measured
    for k in range(3):
        dp = states[k, :2] - references[k, :2]
        dyaw = wrap(states[k, 2] - references[k, 2])
        pw = 1.0 + 0.05 * k
        yw = 0.02 + 0.005 * k
        objective += 0.1 * (pw * np.dot(dp, dp) + yw * dyaw * dyaw)
        objective += 0.1 * controls[k, 0] ** 2 + 0.02 * controls[k, 1] ** 2
        expected = step(states[k], controls[k], dt)
        residual = np.array(
            [states[k + 1, 0] - expected[0], states[k + 1, 1] - expected[1], wrap(states[k + 1, 2] - expected[2])]
        )
        max_dynamics = max(max_dynamics, float(np.max(np.abs(residual))))
        max_bounds = max(max_bounds, -controls[k, 0], controls[k, 0] - 0.8, abs(controls[k, 1]) - 1.5)
        interval = 0.08 if k == 0 else dt
        max_accel = max(
            max_accel,
            abs(controls[k, 0] - previous[0]) - 2.0 * interval,
            abs(controls[k, 1] - previous[1]) - 3.2 * interval,
        )
        previous = controls[k]
    terminal_dp = states[-1, :2] - references[-1, :2]
    terminal_yaw = wrap(states[-1, 2] - references[-1, 2])
    objective += 1.7 * (np.dot(terminal_dp, terminal_dp) + 0.02 * terminal_yaw**2)
    objective += 50.0 * float(np.dot(slack, slack))

    clearances = np.array([clearance(states[k], obstacles[k], safe) for k in range(4)])
    geometry = max(0.0, float(np.max(-clearances)))
    barrier = max(
        0.0,
        max(gamma * clearances[k] - clearances[k + 1] - slack[k] for k in range(3)),
    )

    expected = {
        "wrap": -0.03,
        "clearance": clearances[1],
        "objective": objective,
        "initial": 0.0,
        "dynamics": max_dynamics,
        "geometry": geometry,
        "barrier": barrier,
        "acceleration": max_accel,
        "bounds": max_bounds,
    }
    differences = {key: abs(actual[key] - value) for key, value in expected.items()}
    worst = max(differences.values())
    if worst > 1.0e-6:
        raise SystemExit(f"reference mismatch: worst={worst:.12g}, differences={differences}")
    print(f"PYTHON_REFERENCE_OK worst_difference={worst:.12g}")


if __name__ == "__main__":
    main()
