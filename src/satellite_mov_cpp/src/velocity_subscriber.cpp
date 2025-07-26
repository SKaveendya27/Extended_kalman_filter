#include "satellite_mov_cpp/velocity_subscriber.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <sstream>
#include <iomanip>

using std::placeholders::_1;

VelocitySubscriber::VelocitySubscriber()
: Node("minimal_subscriber")
{
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered",
    rclcpp::QoS(10),
    std::bind(&VelocitySubscriber::topic_callback, this, _1));
}

void VelocitySubscriber::topic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  latest_velocity_ = msg->twist.twist;

  const auto & linear = latest_velocity_.linear;
  const auto & angular = latest_velocity_.angular;

  std::ostringstream oss;
  oss << std::setprecision(17);
  oss << "twist:\n"
      << "  linear:\n"
      << "    x: " << linear.x << "\n"
      << "    y: " << linear.y << "\n"
      << "    z: " << linear.z << "\n"
      << "  angular:\n"
      << "    x: " << angular.x << "\n"
      << "    y: " << angular.y << "\n"
      << "    z: " << angular.z;

  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}

// ✅ THIS WAS MISSING
std::vector<double> VelocitySubscriber::get_velocity()
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  return {
    latest_velocity_.linear.x,
    latest_velocity_.linear.y,
    latest_velocity_.linear.z,
    latest_velocity_.angular.x,
    latest_velocity_.angular.y,
    latest_velocity_.angular.z
  };
}
