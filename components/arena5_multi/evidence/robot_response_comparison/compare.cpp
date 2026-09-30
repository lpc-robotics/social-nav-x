// CPU-only controlled experiment. No ROS nodes, services or robot commands.
#include "arena_multi_hunav_core/core.hpp"
#include "sfm.hpp"
#include <rclcpp/time.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace arena_multi_hunav_core;
using Agent = hunav_msgs::msg::Agent;

Compute::Request initial(bool staggered, bool wide) {
  Compute::Request q; q.dt=.025; q.epoch=1;
  q.current_agents.header.frame_id=q.robots.header.frame_id="map";
  Agent p; p.id=1; p.name="person"; p.type=Agent::PERSON; p.group_id=-1;
  p.radius=.4; p.desired_velocity=1; p.goal_radius=.3; p.behavior.type=1;
  p.behavior.goal_force_factor=2; p.behavior.obstacle_force_factor=10; p.behavior.social_force_factor=5;
  p.position.position.x=10; p.position.position.y=10; p.position.orientation.w=1;
  geometry_msgs::msg::Pose g; g.position.x=16; g.position.y=10; g.orientation.w=1;
  p.goals.push_back(g); q.current_agents.agents.push_back(p);
  arena_multi_hunav_msgs::msg::BehaviorModifiers m; m.agent_id=1; q.modifiers.push_back(m);
  for(int i=0;i<2;i++) {
    Agent r; r.id=-1-i; r.name="robot_"+std::to_string(i+1); r.type=Agent::ROBOT;
    r.group_id=-1; r.radius=.35; r.position.orientation.w=1;
    r.position.position.x=11+(staggered && i==1?.6:0); r.position.position.y=i==0?9.2:(wide?11.2:10.8);
    q.robots.agents.push_back(r);
  }
  return q;
}

// Reproduce the original Regular single-agent LightSFM compute/update branch.
// Match speed, walls, timestep and goal to the new core; no new near-force or acceleration cap.
void legacy_step(Compute::Request &q, int robot) {
  auto &p=q.current_agents.agents[0]; sfm::Agent a; a.id=p.id;
  a.radius=p.radius; a.desiredVelocity=p.desired_velocity;
  a.position.set(p.position.position.x,p.position.position.y);
  a.velocity.set(p.velocity.linear.x,p.velocity.linear.y); a.yaw.setRadian(p.yaw);
  a.params.forceFactorDesired=2; a.params.forceFactorObstacle=10; a.params.forceFactorSocial=5;
  for(auto &g:p.goals) {sfm::Goal goal; goal.center.set(g.position.x,g.position.y); goal.radius=.3; a.goals.push_back(goal);}
  double x=a.position.getX(),y=a.position.getY();
  a.obstacles1={{0,y},{30,y},{x,0},{x,23}};
  sfm::Agent r; r.id=q.robots.agents[robot].id; r.radius=.35;
  r.position.set(q.robots.agents[robot].position.position.x,q.robots.agents[robot].position.position.y); r.velocity.set(0,0);
  std::vector<sfm::Agent> others{r}; sfm::SFM.computeForces(a,others); sfm::SFM.updatePosition(a,q.dt);
  p.position.position.x=a.position.getX(); p.position.position.y=a.position.getY();
  p.velocity.linear.x=a.velocity.getX(); p.velocity.linear.y=a.velocity.getY(); p.yaw=a.yaw.toRadian();
  if(a.goals.empty()) p.goals.clear();
}

int main(int argc,char**argv) {
  if(argc!=2 && argc!=3) return 2;
  bool staggered=argc==3;
  std::ofstream csv(std::string(argv[1])+"/trajectories.csv"); csv<<std::setprecision(12);
  csv<<"mode,t,x,y,vx,vy,gap_r1,gap_r2,force_r1,force_r2,arrived\n";
  for(std::string mode:{"legacy_r1","legacy_r2","current_r1","current_r2","current_nearest","current_both","current_none"}) {
    auto q=initial(staggered,argc==3 && std::string(argv[2])=="wide"); bool arrived=false;
    for(int k=0;k<=1200;k++) {
      auto &p=q.current_agents.agents[0]; auto pos=p.position.position;
      double gaps[2]; for(int r=0;r<2;r++) gaps[r]=std::hypot(pos.x-q.robots.agents[r].position.position.x,pos.y-q.robots.agents[r].position.position.y)-.75;
      double forces[2]={0,0}; Compute::Response result;
      bool legacy=mode.find("legacy")==0;
      if(!legacy && !arrived && k<1200) {
        auto input=q; input.step=k+1; input.robots.header.stamp=input.current_agents.header.stamp;
        if(mode=="current_r1") input.robots.agents.erase(input.robots.agents.begin()+1);
        if(mode=="current_r2") input.robots.agents.erase(input.robots.agents.begin());
        if(mode=="current_nearest") input.robots.agents.erase(input.robots.agents.begin()+(gaps[0]<=gaps[1]?1:0));
        if(mode=="current_none") input.robots.agents.clear();
        result=integrate(input,{});
        if(!result.success) {std::cerr<<mode<<": "<<result.error<<"\n";return 1;}
        for(auto &f:result.influences) forces[f.robot_name=="robot_1"?0:1]=std::hypot(f.force.x,f.force.y);
      }
      csv<<mode<<","<<k*q.dt<<","<<pos.x<<","<<pos.y<<","<<p.velocity.linear.x<<","<<p.velocity.linear.y
         <<","<<gaps[0]<<","<<gaps[1]<<","<<forces[0]<<","<<forces[1]<<","<<arrived<<"\n";
      if(k==1200 || arrived) continue;
      if(legacy) legacy_step(q,mode=="legacy_r1"?0:1); else q.current_agents=result.updated_agents;
      if(q.current_agents.agents[0].goals.empty()) {arrived=true; q.current_agents.agents[0].velocity=geometry_msgs::msg::Twist();}
    }
  }
}
