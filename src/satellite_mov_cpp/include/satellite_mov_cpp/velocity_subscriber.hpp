#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <vector>

class VelocitySubscriber : public rclcpp::Node
{
public:
  VelocitySubscriber();
  std::vector<double> get_velocity();

private:
  void topic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  geometry_msgs::msg::Twist latest_velocity_;
  std::mutex velocity_mutex_;
};
