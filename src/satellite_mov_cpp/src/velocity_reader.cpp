// src/velocity_reader.cpp
#include "satellite_mov_cpp/velocity_subscriber.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <chrono>
#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<VelocitySubscriber>();

  // Spin in a separate thread so callbacks can run
  std::thread spin_thread([&]() {
    rclcpp::spin(node);
  });

  // Periodically read velocity
  for (int i = 0; i < 5; ++i)
  {
    auto vec = node->get_velocity();
    std::cout << "Velocity: ";
    for (auto val : vec)
      std::cout << val << " ";
    std::cout << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
