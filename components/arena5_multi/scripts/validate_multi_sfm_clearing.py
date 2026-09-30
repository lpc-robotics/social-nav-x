#!/usr/bin/env python3
"""Verify moving pedestrians mark and clear local costmaps after free ray coverage.

Use stationary robots and a clear interior test area. Candidate cells must be
lethal near an actual pedestrian, later outside all people/robot inflation areas,
and traversed by a normalized lidar ray before the two-simulation-second gate.
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
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from arena_people_msgs.msg import Pedestrians
from tf2_ros import Buffer, TransformListener

def stamp(msg): return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def cell(grid,x,y):
    ix=math.floor((x-grid.info.origin.position.x)/grid.info.resolution)
    iy=math.floor((y-grid.info.origin.position.y)/grid.info.resolution)
    if not 0<=ix<grid.info.width or not 0<=iy<grid.info.height:return None
    return grid.data[iy*grid.info.width+ix]

class Probe(Node):
    def __init__(self):
        super().__init__('multi_sfm_clearing_probe')
        self.people=None;self.grids={};self.scans={};self.robots={};self.samples={};self.grid_history={};self.scan_history={};self.processed={};self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_subscription(Pedestrians,'/multirobot/hunav/actual_people',lambda m:setattr(self,'people',m),10)
        for name in ('robot_1','robot_2'):
            self.grid_history[name]=deque(maxlen=100);self.scan_history[name]=deque(maxlen=100)
            self.samples[name]={'marked':0,'cleared':0,'timeout':0,'max_clear_delay_sim_s':0.,'timeout_examples':[],'unobserved_neighborhood_samples':0,'candidates':{}}
            self.create_subscription(OccupancyGrid,f'/{name}/local_costmap/costmap',lambda m,n=name:self.grid_history[n].append(m),qos_profile_sensor_data)
            self.create_subscription(LaserScan,f'/{name}/lidar_normalized',lambda m,n=name:self.scan_history[n].append(m),qos_profile_sensor_data)
            self.create_subscription(Odometry,f'/{name}/odom',lambda m,n=name:self.robots.__setitem__(n,m),qos_profile_sensor_data)

    def ray_passes(self,scan,x,y,grid):
        resolution=grid.info.resolution
        try:t=self.buffer.lookup_transform('map',scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform
        except Exception:return False
        dx=x-t.translation.x;dy=y-t.translation.y;distance=math.hypot(dx,dy)
        angle=math.atan2(dy,dx)-yaw(t.rotation);angle=math.atan2(math.sin(angle),math.cos(angle))
        index=round((angle-scan.angle_min)/scan.angle_increment)
        if not 0<=index<len(scan.ranges) or distance>=12:return False
        # Nav2 clears Bresenham raster cells, which differ from geometric
        # ray/AABB intersections near cell corners. Match the installed
        # ObstacleLayer clipping and Costmap2D rasterization before starting
        # the clearing deadline. This does not relax the two-second deadline.
        measured=scan.ranges[index]
        if math.isnan(measured) or measured<=distance+.15:return False
        if math.isinf(measured):measured=scan.range_max-.0001
        beam=scan.angle_min+index*scan.angle_increment+yaw(t.rotation)
        ox,oy=t.translation.x,t.translation.y
        a,b=measured*math.cos(beam),measured*math.sin(beam)
        wx,wy=ox+a,oy+b
        gx,gy=grid.info.origin.position.x,grid.info.origin.position.y
        ex,ey=gx+grid.info.width*resolution,gy+grid.info.height*resolution
        if wx<gx:wx,wy=gx,oy+b*(gx-ox)/a
        if wy<gy:wx,wy=ox+a*(gy-oy)/b,gy
        if wx>ex:wx,wy=ex-.001,oy+b*(ex-ox)/a
        if wy>ey:wx,wy=ox+a*(ey-oy)/b,ey-.001
        x0,y0=math.floor((ox-gx)/resolution),math.floor((oy-gy)/resolution)
        x1,y1=math.floor((wx-gx)/resolution),math.floor((wy-gy)/resolution)
        target=(math.floor((x-gx)/resolution),math.floor((y-gy)/resolution))
        dx,dy=x1-x0,y1-y0;ax,ay=abs(dx),abs(dy)
        sx,sy=(1 if dx>0 else -1),(1 if dy>0 else -1)
        length=math.hypot(dx,dy);scale=min(1.,math.ceil(12/resolution)/length) if length else 1.
        if ax>=ay:
            error=ax//2
            for _ in range(int(scale*ax)+1):
                if (x0,y0)==target:return True
                x0+=sx;error+=ay
                if error>=ax:y0+=sy;error-=ax
        else:
            error=ay//2
            for _ in range(int(scale*ay)+1):
                if (x0,y0)==target:return True
                y0+=sy;error+=ax
                if error>=ay:x0+=sx;error-=ay
        return False

    def tick(self):
        if not self.people:return
        occupied=[(p.pose.position.x,p.pose.position.y) for p in self.people.pedestrians]
        occupied += [(r.pose.pose.position.x,r.pose.pose.position.y) for r in self.robots.values()]
        for name,record in self.samples.items():
            if not self.grid_history[name] or not self.scan_history[name]:continue
            grid=min(self.grid_history[name],key=lambda m:abs(stamp(m)-stamp(self.people)))
            scan=min(self.scan_history[name],key=lambda m:abs(stamp(m)-stamp(self.people)))
            if abs(stamp(grid)-stamp(self.people))>.15 or abs(stamp(scan)-stamp(self.people))>.15:continue
            if self.processed.get(name)==stamp(grid):continue
            self.processed[name]=stamp(grid)
            if grid.header.frame_id not in ('map',name+'/odom'):continue
            if any(abs(r.twist.twist.linear.x)>.02 or abs(r.twist.twist.angular.z)>.02 for r in self.robots.values()):continue
            for p in self.people.pedestrians:
                x=p.pose.position.x;y=p.pose.position.y
                # Reject walls and static robot regions so nearby lethal cells
                # cannot be attributed to a person just by proximity to a wall.
                if not 2<x<28 or not 2<y<21:continue
                for dx in (-.2,0,.2):
                    for dy in (-.2,0,.2):
                        cx=grid.info.origin.position.x+(math.floor((x+dx-grid.info.origin.position.x)/grid.info.resolution)+.5)*grid.info.resolution
                        cy=grid.info.origin.position.y+(math.floor((y+dy-grid.info.origin.position.y)/grid.info.resolution)+.5)*grid.info.resolution
                        key=(cx,cy)
                        if cell(grid,cx,cy)==100 and key not in record['candidates']:
                            record['marked']+=1;record['candidates'][key]={'observed':stamp(grid),'ray_time':None}
            for (x,y),candidate in list(record['candidates'].items()):
                if any(math.hypot(x-px,y-py)<1.4 for px,py in occupied):
                    candidate['ray_time']=None;candidate['observed']=stamp(grid);continue
                if candidate['ray_time'] is None:
                    if stamp(scan)<candidate['observed'] or not self.ray_passes(scan,x,y,grid):continue
                    # The published master grid includes a 0.55 m inflation
                    # layer. A free ray through this cell cannot remove costs
                    # from a neighboring obstacle that the ray did not cover.
                    # Require clearing coverage for those lethal neighbors too.
                    resolution=grid.info.resolution
                    ix=math.floor((x-grid.info.origin.position.x)/resolution)
                    iy=math.floor((y-grid.info.origin.position.y)/resolution)
                    radius=math.ceil(.55/resolution)+1
                    neighborhood=[]
                    for yy in range(max(0,iy-radius),min(grid.info.height,iy+radius+1)):
                        for xx in range(max(0,ix-radius),min(grid.info.width,ix+radius+1)):
                            if grid.data[yy*grid.info.width+xx]!=100:continue
                            cx=grid.info.origin.position.x+(xx+.5)*resolution
                            cy=grid.info.origin.position.y+(yy+.5)*resolution
                            if math.hypot(cx-x,cy-y)<=.55+resolution:
                                neighborhood.append((cx,cy))
                    if not all(self.ray_passes(scan,cx,cy,grid) for cx,cy in neighborhood):
                        record['unobserved_neighborhood_samples']+=1;continue
                    candidate['ray_time']=stamp(scan)
                age=stamp(grid)-candidate['ray_time']
                if age<0:continue
                value=cell(grid,x,y)
                if value is None:continue
                if value==0:
                    record['cleared']+=1;record['max_clear_delay_sim_s']=max(record['max_clear_delay_sim_s'],age)
                    del record['candidates'][(x,y)]
                elif age>2:
                    record['timeout']+=1
                    if len(record['timeout_examples'])<5:
                        record['timeout_examples'].append({'x':x,'y':y,'cost':int(value),'grid_time':stamp(grid),'ray_time':candidate['ray_time'],'nearest_body_m':min(math.hypot(x-px,y-py) for px,py in occupied)})
                    del record['candidates'][(x,y)]

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--duration',type=float,default=120);ap.add_argument('--output',required=True);args=ap.parse_args()
    rclpy.init();node=Probe();end=time.monotonic()+args.duration
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05);node.tick()
    rows={n:{k:v for k,v in r.items() if k!='candidates'} for n,r in node.samples.items()}
    passed=all(r['marked']>0 and r['cleared']>=3 and r['timeout']==0 and r['max_clear_delay_sim_s']<=2 for r in rows.values())
    report={'passed':passed,'scope':'actual people and free-ray-conditioned clearing, stationary robots','robots':rows}
    Path(args.output).parent.mkdir(parents=True,exist_ok=True);Path(args.output).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    node.destroy_node();rclpy.shutdown();return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
