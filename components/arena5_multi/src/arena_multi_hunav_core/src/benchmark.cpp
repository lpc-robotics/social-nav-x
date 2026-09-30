// Deterministic CPU interaction acceptance. This does not replace Isaac acceptance.
#include "arena_multi_hunav_core/core.hpp"
#include <rclcpp/time.hpp>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
using namespace arena_multi_hunav_core;
using Agent = hunav_msgs::msg::Agent;

struct Trial { bool success=true; double clearance=1e9, elapsed=0; std::string error; };
Compute::Request initial(int count, int seed, const std::string & mode) {
  Compute::Request q; q.epoch=1; q.dt=.025; q.current_agents.header.frame_id="map"; q.robots.header.frame_id="map";
  std::mt19937 rng(seed); std::uniform_real_distribution<double> offset(-.08,.08);
  for(int i=0;i<count;i++) {
    Agent p; p.id=i+1; p.name="person_"+std::to_string(i+1); p.type=Agent::PERSON; p.group_id=-1;
    p.radius=.4; p.desired_velocity=1; p.goal_radius=.3; p.behavior.type=1;
    p.behavior.goal_force_factor=2; p.behavior.obstacle_force_factor=10; p.behavior.social_force_factor=5;
    p.position.position.x=6-(i%3)*1.5; p.position.position.y=10+(i/3)*2.0+offset(rng); p.position.orientation.w=1;
    geometry_msgs::msg::Pose g; g.position.x=16; g.position.y=p.position.position.y; g.orientation.w=1;
    p.goals.push_back(g); q.current_agents.agents.push_back(p);
    arena_multi_hunav_msgs::msg::BehaviorModifiers m; m.agent_id=p.id; q.modifiers.push_back(m);
  }
  for(int i=0;i<2;i++) {
    Agent r; r.id=-i-1; r.name="robot_"+std::to_string(i+1); r.type=Agent::ROBOT; r.radius=.35; r.group_id=-1; r.position.orientation.w=1;
    r.position.position.x=10; r.position.position.y=i==0 ? 10:13;
    if(mode=="robot_1_only") r.position.position.y=i==0 ? 10:20;
    if(mode=="robot_2_only") r.position.position.y=i==0 ? 20:10;
    if(mode=="gap") {r.position.position.x=10; r.position.position.y=i==0 ? 8.7:11.3;}
    if(mode=="crossing" || mode=="simultaneous") {
      r.position.position.x=i==0 ? 9:11;
      r.position.position.y=i==0 ? 9:11;
      r.velocity.linear.y=i==0 ? .26:-.26;
      if(mode=="simultaneous") r.velocity.linear.x=-.10;
    }
    q.robots.agents.push_back(r);
  }
  return q;
}

Trial run(Compute::Request q) {
  Trial trial;
  for(int k=0;k<4800;k++) {
    q.step=k+1; q.robots.header.stamp=q.current_agents.header.stamp;
    auto out=integrate(q,{});
    if(!out.success) {trial.success=false; trial.error=out.error; break;}
    for(const auto & p:out.updated_agents.agents) for(const auto & r:q.robots.agents)
      trial.clearance=std::min(trial.clearance,std::hypot(p.position.position.x-r.position.position.x,p.position.position.y-r.position.position.y)-p.radius-r.radius);
    q.current_agents=out.updated_agents;
    for(auto & r:q.robots.agents) {
      if(k*.025<12) {r.position.position.x+=r.velocity.linear.x*.025; r.position.position.y+=r.velocity.linear.y*.025;}
      else r.velocity=geometry_msgs::msg::Twist();
    }
    trial.elapsed=(k+1)*.025;
    bool finished=true; for(const auto & p:q.current_agents.agents) if(!p.goals.empty()) finished=false;
    if(finished) break;
    if(k==4799) {trial.success=false; trial.error="completion_timeout";}
  }
  if(trial.clearance<.10-1e-6) {trial.success=false; trial.error="clearance_below_0.10";}
  return trial;
}

int main(int argc, char **argv) {
  int start=11, end=20; if(argc==3) {start=std::stoi(argv[1]); end=std::stoi(argv[2]);}
  bool passed=true, first=true;
  std::cout<<"{\"scope\":\"CPU prescribed robot trajectories; not Isaac acceptance\",\"trials\":[";
  for(int count:{1,6}) for(const std::string mode:{"robot_1_only","robot_2_only","static","gap","crossing","simultaneous"}) for(int seed=start;seed<=end;seed++) {
    auto q=initial(count,seed,mode); const auto t=run(q); passed &= t.success;
    if(!first) std::cout<<","; first=false;
    std::cout<<"{\"people\":"<<count<<",\"mode\":\""<<mode<<"\",\"seed\":"<<seed<<",\"passed\":"<<(t.success?"true":"false")
      <<",\"min_clearance\":"<<t.clearance<<",\"elapsed_sim_s\":"<<t.elapsed<<",\"error\":\""<<t.error<<"\"}";
  }
  std::cout<<"],\"passed\":"<<(passed?"true":"false")<<"}\n"; return passed?0:1;
}
