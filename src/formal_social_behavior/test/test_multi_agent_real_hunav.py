"""The real pinned HuNav v1 manager, with both humans in the same SFM world."""

import copy
import math
import os
import subprocess
import uuid

import pytest

from test_multi_agent_core import scene
from test_multi_agent_service import Harness, request_for, wait


@pytest.fixture
def real_hunav(tmp_path):
    root = "/multi_real_" + uuid.uuid4().hex[:10]
    path = tmp_path / "manager.log"
    with path.open("w") as log:
        manager = subprocess.Popen([
            "ros2", "run", "hunav_agent_manager", "hunav_agent_manager", "--ros-args",
            "-r", "__node:=" + root[1:], "-r", "compute_agents:=" + root + "/raw_compute",
            "-r", "reset_agents:=" + root + "/raw_reset",
            "-p", "publish_tf:=false", "-p", "publish_sfm_forces:=false",
            "-p", "hunav_loader.publish_people:=false"], env=os.environ.copy(),
            stdout=log, stderr=subprocess.STDOUT)
        harness = None
        try:
            harness = Harness(tmp_path, raw_root=root, timeout=5)
            yield harness, path
        finally:
            if harness:
                harness.close()
            if manager.poll() is None:
                manager.terminate()
                try:
                    manager.wait(5)
                except subprocess.TimeoutExpired:
                    manager.kill()
                    manager.wait(5)


def feedback(harness, previous, snapshot):
    request = request_for(snapshot)
    request.current_agents.agents = copy.deepcopy(previous.updated_agents.agents)
    return wait(harness.client.call_async(request))


def test_real_pair_formation_intrusion_look_at_and_recovery(real_hunav):
    h, log = real_hunav
    h.form()
    assert h.proxy.reset_count == 0
    result = h.call(scene(2, 5.21, vx=.15))
    assert h.states() == ("SURPRISED", "SURPRISED")
    assert [a.behavior.type for a in result.updated_agents.agents] == [3, 3]
    # Post-reset compute only initializes trees; subsequent ticks execute them.
    for index in range(1, 61):
        result = feedback(h, result, scene(2 + .025 * index, 5.21))
        assert len(result.updated_agents.agents) == 2
    for agent in result.updated_agents.agents:
        assert agent.radius == pytest.approx(.4) and agent.goal_radius == pytest.approx(.3)
        assert math.hypot(agent.velocity.linear.x, agent.velocity.linear.y) <= .02
        desired = math.atan2(3 - agent.position.position.y, 5.21 - agent.position.position.x)
        error = (agent.yaw - desired + math.pi) % (2 * math.pi) - math.pi
        assert abs(math.degrees(error)) <= 3
    # Continuous safety recovers the formal state, not the restarted BT timer.
    result = feedback(h, result, scene(5.1, 2, vx=-.3))
    assert h.states() == ("NORMAL", "NORMAL")
    assert h.proxy.reset_count == 2
    assert all(a.behavior.type == 1 and a.linear_vel == 0 for a in result.updated_agents.agents)
    assert log.read_text().count("RESET AGENTS SERVICE CALLED") == 2


def test_real_asymmetric_intrusion_has_outward_motion_and_one_reset(real_hunav):
    h, log = real_hunav
    h.form()
    result = h.call(scene(2, 5.21, 2.4, .15))
    assert h.states() == ("SCARED", "SURPRISED")
    assert [a.behavior.type for a in result.updated_agents.agents] == [4, 3]
    outward = []
    for index in range(1, 16):
        result = feedback(h, result, scene(2 + .025 * index, 5.21, 2.4))
        a, b = result.updated_agents.agents
        dx, dy = a.position.position.x - 5.21, a.position.position.y - 2.4
        outward.append((a.velocity.linear.x * dx + a.velocity.linear.y * dy) / math.hypot(dx, dy))
        assert math.hypot(b.velocity.linear.x, b.velocity.linear.y) <= .02
    assert max(outward) > .01
    assert h.proxy.reset_count == 1
    assert log.read_text().count("RESET AGENTS SERVICE CALLED") == 1
