cmake_minimum_required(VERSION 3.8)
project(satellite_mov_cpp)

include_directories(
  include
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)


# Ros velocity subscriber executable
add_executable(ros-velocity-subscriber
  src/velocity_subscriber.cpp src/main.cpp
)

add_executable(ros-velocity-reader
  src/velocity_reader.cpp
  src/velocity_subscriber.cpp
)

ament_target_dependencies(ros-velocity-subscriber
  rclcpp
  nav_msgs
  geometry_msgs
)

ament_target_dependencies(ros-velocity-reader
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS
  ros-velocity-subscriber
  ros-velocity-reader
  DESTINATION lib/${PROJECT_NAME}
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
