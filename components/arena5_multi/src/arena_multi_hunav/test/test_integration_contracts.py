from pathlib import Path
import importlib.util
import threading
from types import SimpleNamespace
import pytest
import yaml
from geometry_msgs.msg import Twist
from arena_multi_control.scenario import load_scenario
from arena_multi_control.command_guard import CommandGuard
from arena_multi_hunav.multi_adapter import load_people

ROOT=Path(__file__).resolve().parents[3]

def test_new_scenarios_and_legacy_defaults():
    old=load_scenario(ROOT/'config/scenarios/two_robots_hunav_six.yaml')
    assert old.pedestrian_backend=='legacy_hunav'
    assert (old.max_linear,old.max_angular)==(.5,1.0)
    for count in (1,6):
        new=load_scenario(ROOT/f'config/scenarios/two_robots_multi_sfm_{count}_nav2.yaml')
        people,models=load_people(new.pedestrian_config)
        assert len(people.agents)==count and len(models)==count
        assert all(p.behavior.type==1 and p.desired_velocity==1.0 for p in people.agents)
        expected_linear=.5 if count==1 else .26
        assert (new.max_linear,new.max_angular)==(expected_linear,1.0)

def test_generated_limits_cover_controller_smoother_and_recovery(tmp_path,monkeypatch):
    spec=importlib.util.spec_from_file_location('multi_launch',ROOT/'src/arena_multi_bringup/launch/multirobot.launch.py')
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    monkeypatch.setenv('ARENA_MULTI_RUN_DIR',str(tmp_path))
    scenario=load_scenario(ROOT/'config/scenarios/two_robots_multi_sfm_1_nav2.yaml')
    path=module._write_robot_params(ROOT/'src/arena_multi_bringup/config/nav2_multirobot.yaml',scenario.robots[0],['robot_2'],'dijkstra',.26,1.0)
    data=yaml.safe_load(Path(path).read_text())['robot_1']
    assert data['controller_server']['ros__parameters']['FollowPath']['max_vel_x']==.26
    assert data['velocity_smoother']['ros__parameters']['max_velocity']==[.26,0.0,1.0]
    assert data['behavior_server']['ros__parameters']['max_rotational_vel']==1.0
    path=module._write_robot_params(ROOT/'src/arena_multi_bringup/config/nav2_multirobot.yaml',scenario.robots[0],['robot_2'],'dijkstra',.26,1.0,'multi_sfm')
    local=yaml.safe_load(Path(path).read_text())['robot_1']['local_costmap']['local_costmap']['ros__parameters']
    assert local['voxel_layer']['plugin']=='nav2_costmap_2d::ObstacleLayer'
    assert local['voxel_layer']['normalized']['clearing'] is True
    assert local['voxel_layer']['normalized']['max_obstacle_height']==2.0

def test_heartbeat_required_and_expiration_blocks_live_command():
    state=SimpleNamespace(_lock=threading.RLock(),_clock_rewind_latched=False,_emergency_latched=False,
                          require_pedestrian_health=True,_pedestrian_healthy=True,_pedestrian_health_seen=1.0)
    command,reason=CommandGuard._decision(state,1.51)
    assert command==Twist() and reason=='pedestrian_unhealthy'
    state._pedestrian_healthy=False
    assert CommandGuard._decision(state,1.01)[1]=='pedestrian_unhealthy'

@pytest.mark.parametrize('change',[{'max_linear':1.0},{'hunav_profile':'six'},{'psychology_model':'sir'}])
def test_invalid_multi_configuration(tmp_path,change):
    original=ROOT/'config/scenarios/two_robots_multi_sfm_1_nav2.yaml'
    data=yaml.safe_load(original.read_text()); data['pedestrian_config']=str(ROOT/'config/hunav/multi_regular_1.yaml'); data.update(change)
    path=tmp_path/'bad.yaml'; path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError): load_scenario(path)

def test_late_and_invalid_compute_response_never_commit():
    import copy
    from arena_multi_hunav_msgs.srv import ComputeMultiAgents
    from arena_multi_hunav.multi_adapter import validate_response
    req=ComputeMultiAgents.Request(epoch=5,step=9,dt=.025)
    req.current_agents,_=load_people(ROOT/'config/hunav/multi_regular_1.yaml')
    res=ComputeMultiAgents.Response(epoch=5,step=9,success=True,updated_agents=copy.deepcopy(req.current_agents))
    res.updated_agents.header.stamp.nanosec=25_000_000
    assert validate_response(req,res)==25_000_000
    res.step=8
    with pytest.raises(ValueError,match='late'):validate_response(req,res)
    res.step=9;res.updated_agents.agents[0].velocity.linear.x=float('nan')
    with pytest.raises(ValueError,match='invalid_compute_motion'):validate_response(req,res)


def test_float_encoded_frame_lead_does_not_latch_false_fault():
    import time
    from arena_multi_hunav.multi_adapter import MultiSfmAdapter
    history=SimpleNamespace(samples=[SimpleNamespace(stamp_ns=1_050_000_001)],wall_seen=time.monotonic())
    state=SimpleNamespace(history={'robot_1':history},clock_ns=1_000_000_000,
        get_parameter=lambda name:SimpleNamespace(value=.6 if name=='observation_timeout' else 1.0))
    assert MultiSfmAdapter._fresh(state)
    history.samples[0].stamp_ns=1_051_000_000
    with pytest.raises(ValueError,match='robot_stamp_stale'):MultiSfmAdapter._fresh(state)
    history.samples[0].stamp_ns=0
    state.clock_ns=1_001_000_000
    with pytest.raises(ValueError,match='robot_stamp_stale'):MultiSfmAdapter._fresh(state)


def test_guard_tolerates_frame_roundoff_but_preserves_watchdogs():
    state=SimpleNamespace(_lock=threading.RLock(),_clock_rewind_latched=False,_emergency_latched=False,
        require_pedestrian_health=False,_started=0.,startup_grace=0.,_lidar_seen=2.,
        observation_timeout=.6,_clock_ns=1_000_000_000,_lidar_stamp_ns=1_050_000_001,
        simulation_stamp_timeout=1.,peer_seen={'robot_2':2.},peer_stamps_ns={'robot_2':1_050_000_001},
        _command_seen=2.,command_timeout=.5,_command_valid=True,_command=Twist())
    assert CommandGuard._decision(state,2.1)[1]=='ok'
    state.peer_stamps_ns['robot_2']=1_051_000_000
    assert CommandGuard._decision(state,2.1)[1]=='peer_stamp_stale:robot_2'
    state.peer_stamps_ns['robot_2']=1_000_000_000
    state._lidar_stamp_ns=1_051_000_000
    assert CommandGuard._decision(state,2.1)[1]=='lidar_stamp_stale'
    state._lidar_stamp_ns=1_000_000_000
    assert CommandGuard._decision(state,2.601)[1]=='lidar_stale'
