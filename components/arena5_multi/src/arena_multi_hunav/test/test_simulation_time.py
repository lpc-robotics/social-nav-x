from pathlib import Path
from types import SimpleNamespace
import importlib.util
import sys

import pytest


@pytest.fixture
def clock_module():
    root=Path(__file__).resolve().parents[3]
    spec=importlib.util.spec_from_file_location('simulation_time',root/'src/arena_isaac/isaac_utils/simulation_time.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_legacy_keeps_world_clock(clock_module,monkeypatch):
    monkeypatch.delenv('ARENA_MULTI_CLOCK_ALIGNMENT',raising=False)
    assert clock_module.observation_time(10.)==10.
    assert clock_module.command_age(10.,9.8)==pytest.approx(.2)


def test_new_mode_matches_ros_epoch_after_world_reset(clock_module,monkeypatch):
    monkeypatch.setenv('ARENA_MULTI_CLOCK_ALIGNMENT','true')
    attribute=SimpleNamespace(get=lambda:10.6)
    core=SimpleNamespace(Controller=SimpleNamespace(attribute=lambda path:attribute))
    monkeypatch.setitem(sys.modules,'omni',SimpleNamespace(graph=SimpleNamespace(core=core)))
    monkeypatch.setitem(sys.modules,'omni.graph',SimpleNamespace(core=core))
    monkeypatch.setitem(sys.modules,'omni.graph.core',core)
    assert clock_module.observation_time(10.)==10.6
    assert clock_module.command_age(10.,10.4)==pytest.approx(.2)
    assert clock_module.command_age(10.,0.)==0.
    attribute.get=lambda:float('nan')
    with pytest.raises(ValueError):clock_module.observation_time(10.)
