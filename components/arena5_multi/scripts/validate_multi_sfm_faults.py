#!/usr/bin/env python3
"""Real ROS adapter/core/guard tests against a synthetic clock and Isaac services.

Use a dedicated ROS domain. Only processes created by this script are signalled.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from arena_people_msgs.srv import SpawnPedestrians, UpdatePedestrians

ROOT=Path(__file__).resolve().parents[1]

class World(Node):
    def __init__(self):
        super().__init__('multi_sfm_test_world')
        self.health=False; self.status={}; self.commands=[]; self.fail_update=False
        self.stop_updates=[]
        self.create_service(SpawnPedestrians,'/isaac/SpawnPedestrians',self.spawn)
        self.create_service(UpdatePedestrians,'/isaac/UpdatePedestrians',self.update)
        self.create_subscription(Bool,'/multirobot/hunav/health',lambda m:setattr(self,'health',m.data),10)
        self.create_subscription(String,'/multirobot/hunav/status',self.on_status,10)
        self.create_subscription(Twist,'/robot_1/cmd_vel',lambda m:self.commands.append((time.monotonic(),m.linear.x)),10)
        self.clock=self.create_publisher(Clock,'/clock',qos_profile_sensor_data)
        self.ready=self.create_publisher(String,'/multirobot/status',10)
        self.odoms={n:self.create_publisher(Odometry,f'/{n}/odom',qos_profile_sensor_data) for n in ('robot_1','robot_2')}
        self.scan=self.create_publisher(LaserScan,'/robot_1/lidar_normalized',qos_profile_sensor_data)
        self.cmd=self.create_publisher(Twist,'/robot_1/cmd_vel_external',10)

    def spawn(self,req,res): res.results=[0]*len(req.pedestrians); return res
    def update(self,req,res):
        if all(p.twist.linear.x==0 and p.twist.linear.y==0 for p in req.pedestrians): self.stop_updates.append(time.monotonic())
        res.results=[1 if self.fail_update else 0]*len(req.pedestrians); return res
    def on_status(self,msg):
        try: self.status=json.loads(msg.data)
        except ValueError: pass
    def tick(self,sim,omit=None,frozen=None):
        ns=round(sim*1e9); clock=Clock(); clock.clock.sec,clock.clock.nanosec=divmod(ns,10**9); self.clock.publish(clock)
        self.ready.publish(String(data='MULTIROBOT_SCENE_READY count=2 robots=robot_1,robot_2'))
        for n,pub in self.odoms.items():
            if omit==n: continue
            msg=Odometry(); msg.header.frame_id=f'{n}/odom'; msg.child_frame_id=f'{n}/base_link'
            msg.header.stamp=clock.clock
            if frozen is not None and n=='robot_2': msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(round(frozen*1e9),10**9)
            msg.pose.pose.position.x=3.0; msg.pose.pose.position.y=3.0 if n=='robot_1' else 7.0
            msg.pose.pose.orientation.w=1.0; pub.publish(msg)
        scan=LaserScan(); scan.header.stamp=clock.clock; scan.header.frame_id='robot_1/lidar'; self.scan.publish(scan)
        cmd=Twist(); cmd.linear.x=.1; self.cmd.publish(cmd)


def run_case(case,outdir):
    children=[]; logs=[]; node=None
    try:
        for name,command in [
            ('core',['ros2','run','arena_multi_hunav_core','multi_sfm_server']),
            ('adapter',['ros2','run','arena_multi_hunav','multi_sfm_adapter','--ros-args','-p','use_sim_time:=true','-p',f'agent_config_file:={ROOT}/config/hunav/multi_regular_1.yaml']),
            ('guard',['ros2','run','arena_multi_control','command_guard','--ros-args','-p','robot_name:=robot_1','-p','control_mode:=external','-p','peer_names:=[robot_2]','-p','require_pedestrian_health:=true'])]:
            log=open(outdir/f'{case}_{name}.log','w'); logs.append(log)
            children.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        node=World(); sim=10.0; started=time.monotonic(); injected=None; frozen=None; resumed=False; before_compute=0
        zero_time=None; detected=None
        while time.monotonic()-started<20:
            now=time.monotonic(); sim+=.005
            if injected is None and node.health and any(abs(v)>.05 for t,v in node.commands):
                injected=now; before_compute=node.status.get('compute',0); frozen=sim-.005
                if case=='compute_late': os.killpg(children[0].pid,signal.SIGSTOP)
                if case=='update_failure': node.fail_update=True
                if case=='clock_rewind': sim-=2
            omit='robot_2' if injected and case=='odom_loss' else None
            freeze=frozen if injected and case=='duplicate_odom' else None
            node.tick(sim,omit,freeze)
            for _ in range(6): rclpy.spin_once(node,timeout_sec=0)
            if injected:
                if case=='compute_late' and now-injected>1.6 and not resumed:
                    os.killpg(children[0].pid,signal.SIGCONT); resumed=True
                fault=node.status.get('fault','')
                if fault and detected is None: detected=now
                zeros=[t for t,v in node.commands if t>=injected and abs(v)<1e-8]
                if zeros and zero_time is None: zero_time=zeros[0]
                if now-injected>3.0: break
            time.sleep(.01)
        errors=[]; fault=node.status.get('fault','')
        if injected is None: errors.append('did not reach moving healthy state')
        expected={'odom_loss':'robot_stale:robot_2','duplicate_odom':'robot_stale:robot_2',
                  'compute_late':'compute_timeout','update_failure':'isaac_update_failed','clock_rewind':'clock_rewind'}
        if fault!=expected[case]: errors.append('unexpected or missing latched fault')
        if node.health: errors.append('healthy after fault')
        if zero_time is None or detected is None or zero_time-detected>.6: errors.append('guard stop missing or late')
        if injected is None or not any(t>injected for t in node.stop_updates): errors.append('no pedestrian zero update')
        if detected and any(abs(v)>1e-8 for t,v in node.commands if t>detected+.6): errors.append('motion resumed after latch')
        return {'case':case,'passed':not errors,'fault':fault,'stop_from_injection_s':None if zero_time is None else zero_time-injected,
                'late_response_resumed':resumed,'failures':errors}
    finally:
        if node: node.destroy_node()
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGCONT); os.killpg(child.pid,signal.SIGINT)
        for child in children:
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(child.pid,signal.SIGKILL); child.wait()
        for log in logs: log.close()


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output-dir',required=True); args=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in ('51','71',None): raise SystemExit('set a dedicated ROS_DOMAIN_ID (e.g. 73)')
    out=Path(args.output_dir); out.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    results=[run_case(case,out) for case in ('odom_loss','duplicate_odom','compute_late','update_failure','clock_rewind')]
    rclpy.shutdown(); report={'passed':all(v['passed'] for v in results),'scope':'real ROS nodes, synthetic world','cases':results}
    (out/'faults.json').write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1
if __name__=='__main__': raise SystemExit(main())
