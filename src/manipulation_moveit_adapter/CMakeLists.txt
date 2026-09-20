cmake_minimum_required(VERSION 3.16)
project(manipulation_moveit_adapter)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(target_pose_executor src/target_pose_executor.cpp)
ament_target_dependencies(target_pose_executor geometry_msgs moveit_ros_planning_interface rclcpp)
target_compile_features(target_pose_executor PUBLIC cxx_std_17)
target_compile_options(target_pose_executor PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS target_pose_executor DESTINATION lib/${PROJECT_NAME})

ament_package()
