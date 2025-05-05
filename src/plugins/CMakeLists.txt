cmake_minimum_required(VERSION 3.8)
project(plugins)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-sim7 REQUIRED)

add_library(color_changer_plugin SHARED plugins/color_changer_plugin.cpp)
ament_target_dependencies(color_changer_plugin rclcpp std_msgs gz-sim7)

install(TARGETS color_changer_plugin
  DESTINATION lib
)

ament_package()
