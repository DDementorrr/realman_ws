cmake_minimum_required(VERSION 3.16)
project(pick_place_mtc)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_task_constructor_core QUIET)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
option(PICK_PLACE_REQUIRE_MTC "Fail configuration when MTC is unavailable" OFF)

if(PICK_PLACE_REQUIRE_MTC AND NOT moveit_task_constructor_core_FOUND)
  message(FATAL_ERROR "MTC core unavailable: install ros-humble-moveit-task-constructor-core and its dependencies")
endif()

# Stage 1 must remain buildable before MTC's optional Humble packages are installed.
if(moveit_task_constructor_core_FOUND)
  find_package(moveit_task_constructor_msgs REQUIRED)
  # MoveIt's exported link interface references the system tl::expected target.
  find_package(tl-expected REQUIRED CONFIG)
  add_executable(plan_pick_place src/plan_pick_place.cpp)
  ament_target_dependencies(plan_pick_place geometry_msgs moveit_msgs moveit_task_constructor_core moveit_task_constructor_msgs rclcpp std_msgs)
  target_compile_features(plan_pick_place PUBLIC cxx_std_17)
  target_compile_options(plan_pick_place PRIVATE -Wall -Wextra -Wpedantic)
  install(TARGETS plan_pick_place DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "MTC core unavailable: install ros-humble-moveit-task-constructor-core to build plan_pick_place")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
