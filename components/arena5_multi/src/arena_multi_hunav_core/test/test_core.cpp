#include "arena_multi_hunav_core/core.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
using namespace arena_multi_hunav_core;
using Agent = hunav_msgs::msg::Agent;

Compute::Request fixture() {
  Compute::Request q;
  q.epoch=7; q.step=1; q.dt=0.025;
  q.current_agents.header.frame_id="map"; q.robots.header.frame_id="map";
  Agent a; a.id=1; a.name="person"; a.type=Agent::PERSON; a.behavior.type=1;
  a.group_id=-1; a.radius=0.4; a.desired_velocity=1; a.goal_radius=0.3;
  a.behavior.goal_force_factor=2; a.behavior.obstacle_force_factor=10; a.behavior.social_force_factor=5;
  a.position.position.x=10; a.position.position.y=10; a.position.orientation.w=1;
  geometry_msgs::msg::Pose goal; goal.position.x=16; goal.position.y=10; goal.orientation.w=1;
  a.goals.push_back(goal); q.current_agents.agents.push_back(a);
  arena_multi_hunav_msgs::msg::BehaviorModifiers m; m.agent_id=1; q.modifiers.push_back(m);
  for(int i=1;i<=2;i++) {
    Agent r; r.id=-i; r.name="robot_"+std::to_string(i); r.type=Agent::ROBOT; r.group_id=-1;
    r.radius=0.35; r.position.position.x=11; r.position.position.y=10+(i==1 ? -0.8:0.8);
    r.position.orientation.w=1; q.robots.agents.push_back(r);
  }
  return q;
}

TEST(Core, AllRobotsHaveIndependentContributions) {
  auto q=fixture(); auto both=integrate(q, {}); ASSERT_TRUE(both.success)<<both.error;
  ASSERT_EQ(both.influences.size(),2u);
  for(const auto & f:both.influences) EXPECT_GT(std::hypot(f.force.x,f.force.y),0.01);
  auto one=q; one.robots.agents.erase(one.robots.agents.begin());
  auto only=integrate(one, {}); ASSERT_TRUE(only.success)<<only.error;
  EXPECT_NE(both.updated_agents.agents[0].velocity.linear.y, only.updated_agents.agents[0].velocity.linear.y);
  for(const auto & f:both.influences) if(f.robot_name==only.influences[0].robot_name) {
    EXPECT_DOUBLE_EQ(f.force.x,only.influences[0].force.x);
    EXPECT_DOUBLE_EQ(f.force.y,only.influences[0].force.y);
  }
}
TEST(Core, RobotOrderAndNamesDoNotChangeMotion) {
  auto q=fixture(); auto a=integrate(q, {});
  std::reverse(q.robots.agents.begin(), q.robots.agents.end());
  std::swap(q.robots.agents[0].name,q.robots.agents[1].name);
  auto b=integrate(q, {}); ASSERT_TRUE(b.success);
  EXPECT_EQ(a.updated_agents,b.updated_agents);
}
TEST(Core, EmptyRobotsAndModifiers) {
  auto q=fixture(); q.robots.agents.clear();
  auto r=integrate(q, {}); ASSERT_TRUE(r.success)<<r.error;
  EXPECT_GT(r.updated_agents.agents[0].position.position.x,10);
  q.modifiers.clear(); EXPECT_FALSE(integrate(q, {}).success);
}
TEST(Core, GeometryAndInputFailuresAreAtomic) {
  auto q=fixture(); q.robots.agents[0].id=1; auto r=integrate(q, {});
  EXPECT_FALSE(r.success); EXPECT_EQ(r.updated_agents,q.current_agents);
  q=fixture(); q.dt=0.026; EXPECT_FALSE(integrate(q, {}).success);
  q=fixture(); q.robots.header.frame_id="odom"; EXPECT_FALSE(integrate(q, {}).success);
  q=fixture(); q.robots.agents[0].velocity.linear.x=NAN; EXPECT_FALSE(integrate(q, {}).success);
  q=fixture(); q.modifiers[0].space_scale=0.5; EXPECT_FALSE(integrate(q, {}).success);
}
TEST(Core, CoincidentEntitiesRemainFinite) {
  auto q=fixture(); q.robots.agents[0].position=q.current_agents.agents[0].position;
  auto r=integrate(q, {}); ASSERT_TRUE(r.success)<<r.error;
  EXPECT_TRUE(std::isfinite(r.updated_agents.agents[0].velocity.linear.x));
}
TEST(Core, PsychologyAppliesWithoutMutatingBaseSpeed) {
  auto q=fixture(); q.robots.agents.clear(); q.modifiers[0].speed_scale=0;
  auto r=integrate(q, {}); ASSERT_TRUE(r.success);
  EXPECT_DOUBLE_EQ(r.updated_agents.agents[0].velocity.linear.x,0);
  EXPECT_DOUBLE_EQ(r.updated_agents.agents[0].desired_velocity,1);
}
TEST(Core, GoalsAdvanceAndRobotsNeverIntegrated) {
  auto q=fixture(); q.robots.agents.clear();
  q.current_agents.agents[0].goals[0].position=q.current_agents.agents[0].position.position;
  auto r=integrate(q, {}); ASSERT_TRUE(r.success); EXPECT_TRUE(r.updated_agents.agents[0].goals.empty());
}
TEST(Core, ExactHeadOnStaticRobotDoesNotCausePermanentStop) {
  auto q=fixture(); q.robots.agents.resize(1); q.robots.agents[0].position.position.y=10;
  for(int i=0;i<4800 && !q.current_agents.agents[0].goals.empty();i++) {
    q.robots.header.stamp=q.current_agents.header.stamp;
    auto r=integrate(q,{}); ASSERT_TRUE(r.success)<<r.error;
    const auto & p=r.updated_agents.agents[0].position.position;
    EXPECT_GE(std::hypot(p.x-11,p.y-10)-.75,.1-1e-5);
    q.current_agents=r.updated_agents;
  }
  EXPECT_TRUE(q.current_agents.agents[0].goals.empty());
}
TEST(Core, AblatingEitherRobotChangesTrajectoryByTenCentimeters) {
  for(int omitted=0;omitted<2;omitted++) {
    auto full=fixture(), ablated=full;
    ablated.robots.agents.erase(ablated.robots.agents.begin()+omitted);
    double maximum=0;
    for(int i=0;i<320;i++) {
      full.robots.header.stamp=full.current_agents.header.stamp;
      ablated.robots.header.stamp=ablated.current_agents.header.stamp;
      auto a=integrate(full,{}), b=integrate(ablated,{});
      ASSERT_TRUE(a.success); ASSERT_TRUE(b.success);
      auto pa=a.updated_agents.agents[0].position.position, pb=b.updated_agents.agents[0].position.position;
      maximum=std::max(maximum,std::hypot(pa.x-pb.x,pa.y-pb.y));
      full.current_agents=a.updated_agents; ablated.current_agents=b.updated_agents;
    }
    EXPECT_GE(maximum,.10)<<"robot index="<<omitted;
  }
}
