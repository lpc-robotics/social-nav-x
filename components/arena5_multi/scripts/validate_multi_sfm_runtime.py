#!/usr/bin/env python3
"""Read-only checks of the multi-SFM chain, including rendered Character poses.

This is a health/geometry probe, not a task-completion or obstacle-clearing test.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from hunav_msgs.msg import Agents
from arena_people_msgs.msg import Pedestrians
from arena_multi_hunav_msgs.msg import InteractionForces
from arena_multi_control.scenario import load_scenario
from arena_multi_hunav.multi_adapter import load_people
from arena_multi_hunav.timeline import Timeline, Sample
from diagnostic_msgs.msg import DiagnosticArray


def stamp(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class FailureLog(set):
    def __init__(self):
        super().__init__(); self.events=[]

    def add(self, reason):
        if reason not in self:
            event={'wall_time':time.time(),'reason':reason}
            self.events.append(event)
            print(json.dumps(event),flush=True)
        super().add(reason)


class Probe(Node):
    def __init__(self, scenario):
        super().__init__("multi_sfm_runtime_probe")
        people, _ = load_people(scenario.pedestrian_config)
        self.radii = {p.name: p.radius for p in people.agents}
        self.names = [r.name for r in scenario.robots]
        self.robot_history = {n: Timeline() for n in self.names}
        self.person_history = {n: Timeline() for n in self.radii}
        self.pending = deque(); self.min_clearance = math.inf; self.max_display_error = 0.0
        self.maximum_error_sample = None
        self.actual_samples = 0; self.agent_samples = 0; self.checked_samples = 0
        self.skipped_samples = 0; self.failures = FailureLog(); self.status = []; self.healthy = False
        self.influences = set(); self.health_seen = 0; self.started = None
        self.actual_times = []; self.guard_samples = {n: 0 for n in self.names}
        self.create_subscription(Bool, "/multirobot/hunav/health", self.health, 10)
        self.create_subscription(String, "/multirobot/hunav/status", self.on_status, 10)
        self.create_subscription(Agents, "/multirobot/hunav/agents", self.people, 10)
        self.create_subscription(Pedestrians, "/multirobot/hunav/actual_people", self.actual, 10)
        self.create_subscription(InteractionForces, "/multirobot/hunav/interactions", self.forces, 10)
        self.create_subscription(DiagnosticArray, "/multirobot/diagnostics", self._on_guard_diagnostics, 10)
        for name in self.names:
            self.create_subscription(Odometry, f"/{name}/odom", lambda m,n=name:self.robot(n,m), qos_profile_sensor_data)

    def ready(self):
        histories=(*self.robot_history.values(),*self.person_history.values())
        return (self.healthy and self.status and self.status[-1].get('updates',0)>=10
                and all(h.samples and h.samples[-1].stamp_ns-h.samples[0].stamp_ns>=1_000_000_000 for h in histories))

    def health(self,msg):
        self.healthy = msg.data; self.health_seen = time.monotonic()
        if self.started is not None and not msg.data: self.failures.add("unhealthy")

    def on_status(self,msg):
        try: data=json.loads(msg.data)
        except ValueError: return
        if data.get('backend') != 'multi_sfm': return
        self.status.append(data)
        if data.get('fault'): self.failures.add(data['fault'])

    def robot(self,name,msg):
        p=msg.pose.pose.position
        try: self.robot_history[name].append(Sample(stamp(msg),p.x,p.y,0,0,0,0),time.monotonic())
        except ValueError as e: self.failures.add(str(e))

    def people(self,msg):
        if set(a.name for a in msg.agents) != set(self.radii): self.failures.add('agent identity mismatch')
        self.agent_samples += 1
        for a in msg.agents:
            if a.name not in self.person_history: continue
            p=a.position.position
            try: self.person_history[a.name].append(Sample(stamp(msg),p.x,p.y,a.yaw,a.velocity.linear.x,a.velocity.linear.y,0),time.monotonic())
            except ValueError as e: self.failures.add(str(e))

    def actual(self,msg):
        if self.started is None: return
        if set(p.name for p in msg.pedestrians) != set(self.radii): self.failures.add('actual identity mismatch')
        self.actual_samples += 1; self.pending.append((time.monotonic(),msg))
        self.actual_times.append((stamp(msg)/1e9,time.monotonic()))

    def _on_guard_diagnostics(self,msg):
        if self.started is None: return
        for status in msg.status:
            for name in self.names:
                if status.name==f'multirobot/{name}/command_guard':
                    self.guard_samples[name]+=1
                    if status.message not in ('ok','command_stale'):
                        self.failures.add(f'{name} guard: {status.message}')

    def forces(self,msg):
        for f in msg.influences:
            if not all(math.isfinite(v) for v in (f.force.x,f.force.y,f.clearance)): self.failures.add('nonfinite influence')
            self.influences.add((f.agent_id,f.robot_name))

    def check_geometry(self):
        while self.pending:
            received,msg=self.pending[0]; t=stamp(msg)
            timelines=[*self.robot_history.values(),*self.person_history.values()]
            if not all(h.samples and h.samples[-1].stamp_ns>=t for h in timelines):
                if time.monotonic()-received < 4: return
                self.failures.add('geometry timeline unavailable'); self.skipped_samples+=1; self.pending.popleft(); continue
            self.pending.popleft()
            try:
                for a in msg.pedestrians:
                    if a.name not in self.radii: continue
                    p=a.pose.position; predicted=self.person_history[a.name].at(t)
                    if not all(math.isfinite(v) for v in (p.x,p.y)):
                        self.failures.add('nonfinite actual position'); continue
                    error=math.hypot(p.x-predicted.x,p.y-predicted.y)
                    if error>self.max_display_error:
                        self.max_display_error=error
                        self.maximum_error_sample={'stamp_ns':t,'name':a.name,'actual':[p.x,p.y],
                            'predicted':[predicted.x,predicted.y],'velocity':[predicted.vx,predicted.vy],
                            'status':self.status[-1] if self.status else None}
                    for history in self.robot_history.values():
                        r=history.at(t)
                        clearance=math.hypot(p.x-r.x,p.y-r.y)-self.radii[a.name]-.35
                        self.min_clearance=min(self.min_clearance,clearance)
                        if clearance < .1-1e-5: self.failures.add('actual clearance below 0.10 m')
                self.checked_samples+=1
            except ValueError as e:
                self.skipped_samples+=1; self.failures.add(str(e))


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('scenario'); ap.add_argument('--duration',type=float,default=60)
    ap.add_argument('--ready-timeout',type=float,default=300); ap.add_argument('--output',required=True)
    args=ap.parse_args(); scenario=load_scenario(args.scenario)
    if scenario.pedestrian_backend!='multi_sfm': raise SystemExit('multi_sfm scenario required')
    rclpy.init(); node=Probe(scenario); wait_until=time.monotonic()+args.ready_timeout
    ready=node.ready
    while time.monotonic()<wait_until and not ready():
        rclpy.spin_once(node,timeout_sec=.05)
        if node.failures: break
    if not ready(): node.failures.add('readiness timeout')
    else:
        node.started=time.monotonic()
        while time.monotonic()-node.started<args.duration:
            rclpy.spin_once(node,timeout_sec=.05); node.check_geometry()
            if time.monotonic()-node.health_seen>.6: node.failures.add('health heartbeat missing')
    # Allow trailing rendered samples to acquire a matching computation timestamp.
    drain=time.monotonic()+1
    node.destroy_subscription(next(s for s in node.subscriptions if s.topic_name=='/multirobot/hunav/actual_people'))
    while node.pending and time.monotonic()<drain:
        rclpy.spin_once(node,timeout_sec=.02); node.check_geometry()
    if node.pending: node.failures.add('unmatched actual samples')
    if node.agent_samples<2 or node.checked_samples<2 or len(node.status)<2: node.failures.add('insufficient samples')
    if len(node.influences)!=len(node.radii)*len(node.names): node.failures.add('missing robot-person pair')
    if node.max_display_error>.25: node.failures.add('display error above 0.25 m')
    if any(v<2 for v in node.guard_samples.values()): node.failures.add('guard diagnostics missing')
    if len(node.status)>=2 and node.status[-1]['compute']<=node.status[0]['compute']: node.failures.add('integration stopped')
    graph=[name for name,ns in node.get_node_names_and_namespaces()]
    if graph.count('multi_sfm_server')!=1 or graph.count('multi_sfm_adapter')!=1: node.failures.add('nonunique backend')
    if 'hunav_agent_manager' in graph: node.failures.add('legacy manager also active')
    if len(node.get_publishers_info_by_topic('/multirobot/hunav/agents'))!=1: node.failures.add('nonunique state publisher')
    report={'passed':not node.failures,'scope':'runtime health and synchronized rendered geometry; not task acceptance',
            'duration_wall_s':args.duration,'agent_samples':node.agent_samples,'actual_samples':node.actual_samples,
            'checked_samples':node.checked_samples,'skipped_samples':node.skipped_samples,
            'minimum_actual_clearance_m':node.min_clearance if math.isfinite(node.min_clearance) else None,
            'maximum_display_error_m':node.max_display_error,'maximum_error_sample':node.maximum_error_sample,'influence_pairs':sorted(node.influences),
            'guard_samples':node.guard_samples,
            'real_time_factor': ((node.actual_times[-1][0]-node.actual_times[0][0])/(node.actual_times[-1][1]-node.actual_times[0][1])) if len(node.actual_times)>1 else None,
            'last_status':node.status[-1] if node.status else None,'failures':sorted(node.failures),'failure_events':node.failures.events}
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    Path(args.output).write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2))
    node.destroy_node(); rclpy.shutdown(); return 0 if report['passed'] else 1

if __name__=='__main__': raise SystemExit(main())
