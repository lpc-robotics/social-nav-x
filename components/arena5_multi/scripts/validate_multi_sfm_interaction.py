#!/usr/bin/env python3
"""Drive both external-mode robots through ordinary pedestrians at <=0.26 m/s.

Uses the dedicated external scenarios; sends commands only after checking spawn
locations. Robots travel three metres on separated opposing lanes, then stop.
All geometry checks use Character Graph poses, synchronized with robot odometry.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from arena_multi_control.scenario import load_scenario
from arena_multi_hunav.multi_adapter import load_people
from arena_multi_hunav_msgs.msg import InteractionForces
from validate_multi_sfm_runtime import Probe


def main():
    ap=argparse.ArgumentParser();ap.add_argument('scenario');ap.add_argument('--duration-sim',type=float,default=60)
    ap.add_argument('--output',required=True);args=ap.parse_args();scenario=load_scenario(args.scenario)
    if any(r.control_mode!='external' for r in scenario.robots) or len(scenario.robots)!=2:
        raise SystemExit('dedicated two-robot external scenario required')
    rclpy.init();node=Probe(scenario);publishers={n:node.create_publisher(Twist,f'/{n}/cmd_vel_external',10) for n in node.names}
    people,_=load_people(scenario.pedestrian_config)
    first_goals={p.name:(p.goals[0].position.x,p.goals[0].position.y) for p in people.agents}
    reached=set();force_peak={n:0. for n in node.names};initial={};travel={n:0. for n in node.names}
    original_forces=node.forces
    def forces(msg):
        original_forces(msg)
        for f in msg.influences:
            if f.agent_id==people.agents[0].id:
                force_peak[f.robot_name]=max(force_peak[f.robot_name],math.hypot(f.force.x,f.force.y))
    node.create_subscription(InteractionForces,'/multirobot/hunav/interactions',forces,10)
    try:
        deadline=time.monotonic()+180
        while time.monotonic()<deadline and not node.ready():
            rclpy.spin_once(node,timeout_sec=.02)
        if not node.ready():raise RuntimeError('backend not ready')
        for robot in scenario.robots:
            p=node.robot_history[robot.name].samples[-1]
            if math.hypot(p.x-robot.x,p.y-robot.y)>.05:raise RuntimeError('robot differs from configured spawn; refusing motion')
            initial[robot.name]=(p.x,p.y)
        node.started=time.monotonic();start_sim=min(h.samples[-1].stamp_ns for h in node.robot_history.values())/1e9
        last_send=0.;elapsed=0.
        while time.monotonic()-node.started<600:
            rclpy.spin_once(node,timeout_sec=.01);node.check_geometry()
            now=time.monotonic();elapsed=min(h.samples[-1].stamp_ns for h in node.robot_history.values())/1e9-start_sim
            for name,h in node.robot_history.items():
                p=h.samples[-1];travel[name]=math.hypot(p.x-initial[name][0],p.y-initial[name][1])
            for name,h in node.person_history.items():
                if h.samples and math.hypot(h.samples[-1].x-first_goals[name][0],h.samples[-1].y-first_goals[name][1])<.35:reached.add(name)
            if now-last_send>=.05:
                for name,pub in publishers.items():
                    cmd=Twist()
                    if not node.failures and node.healthy and travel[name]<3.0 and elapsed<20:cmd.linear.x=.26
                    pub.publish(cmd)
                last_send=now
            if elapsed>=args.duration_sim or node.failures:break
        if elapsed<args.duration_sim:node.failures.add('interaction ended early')
        if any(v<2.8 for v in travel.values()):node.failures.add('both robots did not traverse interaction lanes')
        if reached!=set(first_goals):node.failures.add('some pedestrians did not reach their first waypoint')
        if any(v<.1 for v in force_peak.values()):node.failures.add('first pedestrian did not react measurably to both robots')
        if node.checked_samples<10:node.failures.add('insufficient actual geometry')
        if node.max_display_error>.25:node.failures.add('display error above 0.25 m')
    except Exception as e:node.failures.add(str(e))
    finally:
        end=time.monotonic()+.6
        while time.monotonic()<end:
            for pub in publishers.values():pub.publish(Twist())
            rclpy.spin_once(node,timeout_sec=.05)
    report={'passed':not node.failures,'scope':'GPU dual moving robots and rendered pedestrian geometry',
            'robot_travel_m':travel,'first_pedestrian_peak_force_by_robot':force_peak,'people_reached_first_goal':sorted(reached),
            'minimum_actual_clearance_m':node.min_clearance if math.isfinite(node.min_clearance) else None,
            'maximum_display_error_m':node.max_display_error,'maximum_error_sample':node.maximum_error_sample,'checked_samples':node.checked_samples,'failures':sorted(node.failures)}
    Path(args.output).parent.mkdir(parents=True,exist_ok=True);Path(args.output).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    node.destroy_node();rclpy.shutdown();return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
