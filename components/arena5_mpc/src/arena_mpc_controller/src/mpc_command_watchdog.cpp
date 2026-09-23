#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <hunav_msgs/msg/agents.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

namespace arena_mpc_controller
{
namespace
{

using SteadyClock = std::chrono::steady_clock;

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

bool finite_twist(const geometry_msgs::msg::Twist & command)
{
  return std::isfinite(command.linear.x) && std::isfinite(command.linear.y) &&
         std::isfinite(command.linear.z) && std::isfinite(command.angular.x) &&
         std::isfinite(command.angular.y) && std::isfinite(command.angular.z);
}

}  // namespace

class MpcCommandWatchdog : public rclcpp::Node
{
public:
  MpcCommandWatchdog()
  : Node("mpc_command_watchdog")
  {
    raw_topic_ = declare_parameter<std::string>("raw_topic", "/cmd_vel_nav");
    smooth_topic_ =
      declare_parameter<std::string>("smooth_topic", "/cmd_vel_watchdog_in");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel");
    status_topic_ = declare_parameter<std::string>("status_topic", "/FollowPath/status");
    human_topic_ = declare_parameter<std::string>("human_topic", "/human_states");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/lidar");
    costmap_topic_ =
      declare_parameter<std::string>("costmap_topic", "/local_costmap/costmap_raw");

    command_lease_ = seconds("command_lease", 0.25);
    human_wall_limit_ = seconds("human_wall_timeout", 0.60);
    odom_wall_limit_ = seconds("odom_wall_timeout", 0.40);
    lidar_wall_limit_ = seconds("lidar_wall_timeout", 1.55);
    costmap_wall_limit_ = seconds("costmap_wall_timeout", 1.55);
    ros_age_limit_ = declare_parameter<double>("ros_age_timeout", 0.30);
    future_tolerance_ = declare_parameter<double>("future_tolerance", 0.05);
    if (ros_age_limit_ <= 0.0 || future_tolerance_ < 0.0) {
      throw std::invalid_argument("watchdog ROS-time limits must be non-negative");
    }

    const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    const auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    output_publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, volatile_qos);
    diagnostic_publisher_ =
      create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(1).reliable());
    raw_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      raw_topic_, volatile_qos,
      std::bind(&MpcCommandWatchdog::on_raw, this, std::placeholders::_1));
    smooth_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      smooth_topic_, volatile_qos,
      std::bind(&MpcCommandWatchdog::on_smooth, this, std::placeholders::_1));
    status_subscription_ = create_subscription<std_msgs::msg::String>(
      status_topic_, volatile_qos,
      std::bind(&MpcCommandWatchdog::on_status, this, std::placeholders::_1));
    human_subscription_ = create_subscription<hunav_msgs::msg::Agents>(
      human_topic_, volatile_qos,
      [this](const hunav_msgs::msg::Agents::SharedPtr message) {
        on_stamped(message->header.stamp, human_input_);
      });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, volatile_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        on_stamped(message->header.stamp, odom_input_);
      });
    lidar_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_topic_, volatile_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
        on_stamped(message->header.stamp, lidar_input_);
      });
    costmap_subscription_ = create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic_, costmap_qos,
      [this](const nav2_msgs::msg::Costmap::SharedPtr message) {
        on_stamped(message->header.stamp, costmap_input_);
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&MpcCommandWatchdog::tick, this));
  }

private:
  struct StampedInput
  {
    bool received{false};
    std::int64_t stamp_ns{0};
    SteadyClock::time_point wall_time{};
  };

  std::chrono::duration<double> seconds(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!(value > 0.0) || !std::isfinite(value)) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return std::chrono::duration<double>(value);
  }

  void invalidate_command_locked()
  {
    status_ok_ = false;
    raw_after_status_ = false;
    smooth_after_raw_ = false;
  }

  void on_stamped(const builtin_interfaces::msg::Time & stamp, StampedInput & input)
  {
    const auto stamp_ns = stamp_nanoseconds(stamp);
    const auto ros_now_ns = get_clock()->now().nanoseconds();
    const auto wall_now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    const double age = static_cast<double>(ros_now_ns - stamp_ns) * 1.0e-9;
    if (stamp_ns <= 0 || ros_now_ns <= 0 || !std::isfinite(age) ||
      age < -future_tolerance_ || age > ros_age_limit_)
    {
      invalidate_command_locked();
      return;
    }
    if (input.received && stamp_ns < input.stamp_ns) {
      invalidate_command_locked();
      return;
    }
    input.received = true;
    input.stamp_ns = stamp_ns;
    input.wall_time = wall_now;
  }

  void on_status(const std_msgs::msg::String::SharedPtr message)
  {
    const auto wall_now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    status_ok_ = message->data.rfind("ok ", 0) == 0;
    status_wall_time_ = wall_now;
    raw_after_status_ = false;
    // A new solve starts the next candidate chain.  Keep the last fully
    // validated chain alive during the normal status -> raw -> smooth phase
    // offset; it remains bounded by the command lease below.  A failure status
    // still revokes the chain immediately.
    if (!status_ok_) {
      invalidate_command_locked();
    }
  }

  void on_raw(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const auto wall_now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    raw_received_ = finite_twist(*message);
    raw_wall_time_ = wall_now;
    raw_after_status_ = raw_received_ && status_ok_ && wall_now >= status_wall_time_;
    if (!raw_received_) {
      invalidate_command_locked();
    }
  }

  void on_smooth(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const auto wall_now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!finite_twist(*message)) {
      smooth_received_ = false;
      invalidate_command_locked();
      return;
    }
    smooth_command_ = *message;
    smooth_received_ = true;
    smooth_wall_time_ = wall_now;
    smooth_after_raw_ = smooth_after_raw_ ||
      (raw_after_status_ && wall_now >= raw_wall_time_);
  }

  bool wall_fresh(
    bool received, const SteadyClock::time_point & then,
    const std::chrono::duration<double> & limit, const SteadyClock::time_point & now) const
  {
    return received && now >= then && (now - then) <= limit;
  }

  bool stamp_fresh(const StampedInput & input, std::int64_t ros_now_ns) const
  {
    if (!input.received || ros_now_ns <= 0) {
      return false;
    }
    const double age = static_cast<double>(ros_now_ns - input.stamp_ns) * 1.0e-9;
    return age >= -future_tolerance_ && age <= ros_age_limit_;
  }

  void tick()
  {
    const auto wall_now = SteadyClock::now();
    const auto ros_now_ns = get_clock()->now().nanoseconds();
    geometry_msgs::msg::Twist output;
    std::string reason = "ok";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_ros_now_ns_ > 0 && ros_now_ns < last_ros_now_ns_) {
        invalidate_command_locked();
        reset_latched_ = true;
      }
      last_ros_now_ns_ = ros_now_ns;

      const bool status_fresh =
        wall_fresh(status_ok_, status_wall_time_, command_lease_, wall_now);
      const bool raw_fresh =
        wall_fresh(raw_received_, raw_wall_time_, command_lease_, wall_now);
      const bool smooth_fresh =
        wall_fresh(smooth_received_, smooth_wall_time_, command_lease_, wall_now);
      const bool command_chain_complete = smooth_after_raw_;
      const bool human_fresh =
        wall_fresh(human_input_.received, human_input_.wall_time, human_wall_limit_, wall_now) &&
        stamp_fresh(human_input_, ros_now_ns);
      const bool odom_fresh =
        wall_fresh(odom_input_.received, odom_input_.wall_time, odom_wall_limit_, wall_now) &&
        stamp_fresh(odom_input_, ros_now_ns);
      const bool lidar_fresh =
        wall_fresh(lidar_input_.received, lidar_input_.wall_time, lidar_wall_limit_, wall_now) &&
        stamp_fresh(lidar_input_, ros_now_ns);
      const bool costmap_fresh =
        wall_fresh(
          costmap_input_.received, costmap_input_.wall_time, costmap_wall_limit_, wall_now) &&
        stamp_fresh(costmap_input_, ros_now_ns);
      const bool unique_output_publisher = count_publishers(output_topic_) <= 1U;

      if (reset_latched_) {
        reason = "clock_reset_latched";
      } else if (!unique_output_publisher) {
        reason = "multiple_cmd_vel_publishers";
      } else if (!status_ok_) {
        reason = "controller_status";
      } else if (!command_chain_complete) {
        reason = "command_sequence";
      } else if (!status_fresh) {
        reason = "status_lease";
      } else if (!raw_fresh) {
        reason = "raw_lease";
      } else if (!smooth_fresh) {
        reason = "smooth_lease";
      } else if (!human_fresh) {
        reason = "human_input";
      } else if (!odom_fresh) {
        reason = "odom_input";
      } else if (!lidar_fresh) {
        reason = "lidar_input";
      } else if (!costmap_fresh) {
        reason = "costmap_input";
      } else {
        output = smooth_command_;
      }
    }

    output_publisher_->publish(output);
    if (reason != last_reason_) {
      std_msgs::msg::String diagnostic;
      diagnostic.data = reason == "ok" ? "ok forwarding" : "stop reason=" + reason;
      diagnostic_publisher_->publish(diagnostic);
      if (reason == "ok") {
        RCLCPP_INFO(get_logger(), "command forwarding enabled");
      } else {
        RCLCPP_WARN(get_logger(), "publishing zero command: %s", reason.c_str());
      }
      last_reason_ = reason;
    }
  }

  std::mutex mutex_;
  geometry_msgs::msg::Twist smooth_command_;
  bool status_ok_{false};
  bool raw_received_{false};
  bool smooth_received_{false};
  bool raw_after_status_{false};
  bool smooth_after_raw_{false};
  SteadyClock::time_point status_wall_time_{};
  SteadyClock::time_point raw_wall_time_{};
  SteadyClock::time_point smooth_wall_time_{};
  StampedInput human_input_;
  StampedInput odom_input_;
  StampedInput lidar_input_;
  StampedInput costmap_input_;
  std::int64_t last_ros_now_ns_{0};
  bool reset_latched_{false};

  std::string raw_topic_;
  std::string smooth_topic_;
  std::string output_topic_;
  std::string status_topic_;
  std::string human_topic_;
  std::string odom_topic_;
  std::string lidar_topic_;
  std::string costmap_topic_;
  std::string last_reason_;
  std::chrono::duration<double> command_lease_;
  std::chrono::duration<double> human_wall_limit_;
  std::chrono::duration<double> odom_wall_limit_;
  std::chrono::duration<double> lidar_wall_limit_;
  std::chrono::duration<double> costmap_wall_limit_;
  double ros_age_limit_{0.3};
  double future_tolerance_{0.05};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr smooth_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Subscription<hunav_msgs::msg::Agents>::SharedPtr human_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace arena_mpc_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arena_mpc_controller::MpcCommandWatchdog>());
  rclcpp::shutdown();
  return 0;
}
