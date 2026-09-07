#!/usr/bin/env python3
"""Independent CasADi Opti formulation for the fixed no-obstacle fixture."""

import os
import subprocess

import casadi as ca
import numpy as np


def main():
    output = subprocess.check_output([os.environ["SOLVER_FIXTURE"]], text=True)
    cpp = {line.split("=", 1)[0]: float(line.split("=", 1)[1]) for line in output.splitlines()}

    horizon = 25
    dt = 0.1
    reference = np.zeros((horizon + 1, 3))
    reference[:, 0] = 0.02 * np.arange(horizon + 1)
    opti = ca.Opti()
    states = opti.variable(horizon + 1, 3)
    controls = opti.variable(horizon, 2)
    opti.subject_to(states[0, :] == np.zeros((1, 3)))
    opti.subject_to(opti.bounded(0.0, controls[:, 0], 0.26))
    opti.subject_to(opti.bounded(-1.0, controls[:, 1], 1.0))
    for k in range(horizon):
        expected = ca.hcat(
            [
                states[k, 0] + dt * controls[k, 0] * ca.cos(states[k, 2]),
                states[k, 1] + dt * controls[k, 0] * ca.sin(states[k, 2]),
                states[k, 2] + dt * controls[k, 1],
            ]
        )
        opti.subject_to(states[k + 1, :] == expected)
        previous = np.zeros(2) if k == 0 else controls[k - 1, :]
        opti.subject_to(opti.bounded(-0.2, controls[k, 0] - previous[0], 0.2))
        opti.subject_to(opti.bounded(-0.32, controls[k, 1] - previous[1], 0.32))

    objective = 0
    for k in range(horizon):
        dx = states[k, 0] - reference[k, 0]
        dy = states[k, 1] - reference[k, 1]
        yaw_delta = states[k, 2] - reference[k, 2]
        yaw_error = ca.atan2(ca.sin(yaw_delta), ca.cos(yaw_delta))
        position_weight = 1.0 + 0.05 * k
        yaw_weight = 0.02 + 0.005 * k
        objective += 0.1 * (
            position_weight * dx**2 + position_weight * dy**2 + yaw_weight * yaw_error**2
        )
        objective += 0.1 * controls[k, 0] ** 2 + 0.02 * controls[k, 1] ** 2
    dx = states[horizon, 0] - reference[horizon, 0]
    dy = states[horizon, 1] - reference[horizon, 1]
    yaw_delta = states[horizon, 2] - reference[horizon, 2]
    yaw_error = ca.atan2(ca.sin(yaw_delta), ca.cos(yaw_delta))
    objective += dx**2 + dy**2 + 0.02 * yaw_error**2
    opti.minimize(objective)

    opti.set_initial(states, reference)
    opti.set_initial(controls[:, 0], 0.2)
    opti.set_initial(controls[:, 1], 0.0)
    opti.solver(
        "ipopt",
        {
            "print_time": False,
            "error_on_fail": False,
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "ipopt.max_iter": 100,
            "ipopt.max_cpu_time": 0.060,
            "ipopt.tol": 1.0e-3,
            "ipopt.acceptable_tol": 1.0e-3,
            "ipopt.acceptable_obj_change_tol": 1.0e-3,
        },
    )
    solution = opti.solve()
    python_control = np.asarray(solution.value(controls[0, :])).reshape(-1)
    python_objective = float(solution.value(objective))
    command_difference = max(
        abs(cpp["linear"] - python_control[0]), abs(cpp["angular"] - python_control[1])
    )
    objective_difference = abs(cpp["objective"] - python_objective)
    if command_difference > 1.0e-3:
        raise SystemExit(
            f"first-control mismatch: cpp={cpp}, python={python_control}, diff={command_difference}"
        )
    if objective_difference > 1.0e-6:
        raise SystemExit(
            f"objective mismatch: cpp={cpp['objective']}, python={python_objective}, diff={objective_difference}"
        )
    print(
        f"PYTHON_SOLVER_REFERENCE_OK command_difference={command_difference:.12g} "
        f"objective_difference={objective_difference:.12g}"
    )


if __name__ == "__main__":
    main()
