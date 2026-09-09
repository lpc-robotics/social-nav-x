"""The GPU runner must reject stale/partial runtime evidence."""

import importlib.util
from pathlib import Path

import pytest


path = Path(__file__).resolve().parents[3] / "scripts/test_formal_multi_simulation_matrix.py"
spec = importlib.util.spec_from_file_location("multi_matrix", path)
matrix = importlib.util.module_from_spec(spec)
spec.loader.exec_module(matrix)

LINE = "SIX_BEHAVIORS_RUNNING compute=100 updates=30 steady_compute=14.5 steady_display=5.5 max_dt=0.025 lag=0.01"


@pytest.mark.parametrize("line", ["", "SIX_BEHAVIORS_RUNNING compute=100",
    LINE.replace("14.5", "9.99"), LINE.replace("5.5", "4.49"),
    LINE.replace("0.025", "0.027"), LINE.replace("lag=0.01", "lag=0.101")])
def test_runtime_gate_rejects_missing_and_slow_metrics(line):
    assert matrix.runtime(line) is None


def test_runtime_gate_requires_fresh_counters_and_uses_latest_interval():
    good = matrix.runtime(LINE)
    assert good and matrix.runtime(LINE, good) is None
    assert matrix.runtime(LINE.replace("100", "101").replace("30", "31"), good)
    assert matrix.runtime(LINE + "\n" + LINE.replace("14.5", "0.0")) is None
