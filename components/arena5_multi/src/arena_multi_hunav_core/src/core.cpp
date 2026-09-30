#include "arena_multi_hunav_core/core.hpp"
#include "sfm.hpp"
#include <algorithm>
#include <set>
#include <map>
#include <stdexcept>
#include <rclcpp/time.hpp>

namespace arena_multi_hunav_core {
namespace {
using Agent = hunav_msgs::msg::Agent;
using Modifier = arena_multi_hunav_msgs::msg::BehaviorModifiers;
void require(bool valid, const char * reason) { if (!valid) throw std::invalid_argument(reason); }
void finite(double value) { require(std::isfinite(value), "non-finite input"); }
sfm::Agent convert(const Agent & a) {
  for (double v : {a.position.position.x, a.position.position.y, double(a.yaw),
      a.velocity.linear.x, a.velocity.linear.y, double(a.radius), double(a.desired_velocity)}) finite(v);
  require(a.radius > 0 && a.desired_velocity >= 0, "invalid radius or speed");
  sfm::Agent s;
  s.id = a.id; s.groupId = -1; s.radius = a.radius; s.desiredVelocity = a.desired_velocity;
  s.position.set(a.position.position.x, a.position.position.y);
  s.velocity.set(a.velocity.linear.x, a.velocity.linear.y); s.yaw.setRadian(a.yaw);
  s.cyclicGoals = a.cyclic_goals;
  require(a.type == Agent::ROBOT || a.goal_radius > 0, "invalid goal radius");
  for (const auto & g : a.goals) {
    finite(g.position.x); finite(g.position.y);
    sfm::Goal goal; goal.center.set(g.position.x, g.position.y); goal.radius = a.goal_radius;
    s.goals.push_back(goal);
  }
  for (double v : {double(a.behavior.goal_force_factor), double(a.behavior.obstacle_force_factor), double(a.behavior.social_force_factor)}) {
    finite(v); require(v >= 0, "negative force coefficient");
  }
  s.params.forceFactorDesired = a.behavior.goal_force_factor;
  s.params.forceFactorObstacle = a.behavior.obstacle_force_factor;
  s.params.forceFactorSocial = a.behavior.social_force_factor;
  return s;
}
// Deterministic pair direction also covers exact overlaps without division by zero.
utils::Vector2d direction(const sfm::Agent & a, const sfm::Agent & b) {
  auto d = a.position - b.position;
  if (d.norm() < 1e-9) return utils::Vector2d(a.id < b.id ? -1.0 : 1.0, 0.0);
  return d / d.norm();
}
utils::Vector2d pair_force(const sfm::Agent & a, const sfm::Agent & b) {
  // Use the pinned LightSFM single-agent kernel. Subtract its non-social terms
  // by reading socialForce only; perturb singular inputs in this private copy.
  auto me = a; auto other = b;
  if ((other.position - me.position).norm() < 1e-6)
    other.position = me.position - direction(a, b) * 1e-6;
  auto diff = other.position - me.position;
  auto interaction = me.params.lambda * (me.velocity - other.velocity) + diff.normalized();
  if (interaction.norm() < 1e-6) other.velocity += utils::Vector2d(1e-6, 0);
  std::vector<sfm::Agent> neighbours{other};
  sfm::SFM.computeForces(me, neighbours);
  finite(me.forces.socialForce.getX()); finite(me.forces.socialForce.getY());
  return me.forces.socialForce;
}
}

Compute::Response integrate(const Compute::Request & req, const Config & cfg) {
  Compute::Response out; out.epoch = req.epoch; out.step = req.step;
  try {
    finite(req.dt); require(req.dt > 0 && req.dt <= 0.025000001, "dt outside (0, 0.025]");
    require(req.current_agents.header.frame_id == "map" && req.robots.header.frame_id == "map", "frame must be map");
    require(req.current_agents.header.stamp == req.robots.header.stamp, "snapshot timestamps differ");
    require(!req.current_agents.agents.empty(), "no pedestrians");
    for (double v : {cfg.width, cfg.height, cfg.near_gain, cfg.near_sigma, cfg.max_acceleration, cfg.max_speed}) {
      finite(v); require(v > 0, "invalid core configuration");
    }
    finite(cfg.robot_clearance); require(cfg.robot_clearance >= 0, "negative clearance");
    std::set<int> ids; std::set<std::string> names;
    std::map<int, sfm::Agent> people, robots;
    for (const auto & a : req.current_agents.agents) {
      require(a.id > 0 && a.type == Agent::PERSON && a.behavior.type == 1, "multi_sfm requires positive-id regular people");
      require(a.group_id == -1, "group behaviors are not supported by multi_sfm v1");
      require(ids.insert(a.id).second && !a.name.empty() && names.insert(a.name).second, "duplicate identity");
      people.emplace(a.id, convert(a));
    }
    for (const auto & a : req.robots.agents) {
      require(a.id < 0 && a.type == Agent::ROBOT, "robots require negative ids");
      require(ids.insert(a.id).second && !a.name.empty() && names.insert(a.name).second, "duplicate identity");
      robots.emplace(a.id, convert(a));
    }
    std::map<int, Modifier> mods;
    for (const auto & m : req.modifiers) {
      require(people.count(m.agent_id) && mods.emplace(m.agent_id, m).second, "invalid modifier identity");
      for (double v : {m.speed_scale, m.social_scale, m.robot_scale, m.space_scale}) {
        finite(v); require(v >= 0 && v <= 4, "modifier outside [0,4]");
      }
      require(m.space_scale >= 1, "personal space cannot shrink below physical envelope");
    }
    require(mods.size() == people.size(), "missing modifiers");
    out.updated_agents = req.current_agents;
    for (auto & msg : out.updated_agents.agents) {
      auto me = people.at(msg.id); const auto & m = mods.at(msg.id);
      me.desiredVelocity = std::min(cfg.max_speed, me.desiredVelocity * m.speed_scale);
      const double x = me.position.getX(), y = me.position.getY();
      require(x >= 0 && x <= cfg.width && y >= 0 && y <= cfg.height, "pedestrian outside world");
      me.obstacles1 = {{0, std::clamp(y, 0.0, cfg.height)}, {cfg.width, std::clamp(y, 0.0, cfg.height)},
        {std::clamp(x, 0.0, cfg.width), 0}, {std::clamp(x, 0.0, cfg.width), cfg.height}};
      for (auto & p : me.obstacles1) if ((p - me.position).norm() < 1e-6) p += utils::Vector2d(1e-6, 1e-6);
      std::vector<sfm::Agent> empty;
      sfm::SFM.computeForces(me, empty);
      auto total = me.forces.desiredForce + me.forces.obstacleForce;
      for (const auto & [id, other] : people) if (id != me.id)
        total += pair_force(me, other) * m.social_scale;
      for (const auto & [id, robot] : robots) {
        const double gap = (me.position - robot.position).norm() - me.radius - robot.radius;
        const double space = cfg.robot_clearance * m.space_scale;
        auto force = pair_force(me, robot) * m.robot_scale;
        // Short-range geometry term is always present, including when psychology
        // suppresses social avoidance. It is not a hard collision guarantee.
        force += direction(me, robot) * (cfg.near_gain * std::exp(std::clamp((space - gap) / cfg.near_sigma, -50.0, 10.0)));
        total += force;
        arena_multi_hunav_msgs::msg::RobotInfluence info;
        info.agent_id = me.id;
        const auto found = std::find_if(req.robots.agents.begin(), req.robots.agents.end(), [id](const auto & r){return r.id == id;});
        info.robot_name = found->name; info.force.x = force.getX(); info.force.y = force.getY(); info.clearance = gap;
        out.influences.push_back(info);
      }
      finite(total.getX()); finite(total.getY());
      if (total.norm() > cfg.max_acceleration) total *= cfg.max_acceleration / total.norm();
      me.forces.globalForce = total;
      const auto goals_before = me.goals.size();
      sfm::SFM.updatePosition(me, req.dt);
      finite(me.position.getX()); finite(me.position.getY());
      require(me.position.getX() >= me.radius && me.position.getX() <= cfg.width - me.radius &&
        me.position.getY() >= me.radius && me.position.getY() <= cfg.height - me.radius, "wall envelope violation");
      msg.position.position.x = me.position.getX(); msg.position.position.y = me.position.getY();
      msg.yaw = me.yaw.toRadian(); msg.position.orientation.x = 0; msg.position.orientation.y = 0;
      msg.position.orientation.z = std::sin(msg.yaw / 2); msg.position.orientation.w = std::cos(msg.yaw / 2);
      msg.velocity.linear.x = me.velocity.getX(); msg.velocity.linear.y = me.velocity.getY();
      msg.velocity.angular.z = me.angularVelocity; msg.linear_vel = me.linearVelocity; msg.angular_vel = me.angularVelocity;
      // Preserve the base desired_velocity. Modifiers are applied afresh next tick.
      if (me.goals.size() < goals_before) msg.goals.erase(msg.goals.begin());
      else if (msg.cyclic_goals && !msg.goals.empty() &&
          (people.at(msg.id).goals.front().center - me.position).norm() <= msg.goal_radius)
        std::rotate(msg.goals.begin(), msg.goals.begin() + 1, msg.goals.end());
    }
    out.updated_agents.header.stamp = (rclcpp::Time(req.current_agents.header.stamp) + rclcpp::Duration::from_seconds(req.dt));
    out.success = true;
  } catch (const std::exception & e) {
    out.success = false; out.error = e.what(); out.updated_agents = req.current_agents; out.influences.clear();
  }
  return out;
}
}
