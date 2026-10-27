"""Stage 3: execute the known-pose MTC task with Gazebo attachment feedback."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("pick_place_sim")) / "launch" / "plan_pick_place_sim.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "wrist_camera_sensor": LaunchConfiguration("wrist_camera_sensor"),
            "pick_place_attachment": "true",
            "enable_mtc_execution": "true",
            "world_file": LaunchConfiguration("world_file"),
            "scene_file": LaunchConfiguration("scene_file"),
            "publish_table": LaunchConfiguration("publish_table"),
            "environment_ready_topic": LaunchConfiguration("environment_ready_topic"),
            "grasp_spin": LaunchConfiguration("grasp_spin"),
            **{key: LaunchConfiguration(key) for key in ("candidate_config", "candidate_archive", "candidate_binding_files", "planning_candidates")},
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("wrist_camera_sensor", default_value="true"),
        DeclareLaunchArgument("auto_execute", default_value="false"),
        DeclareLaunchArgument("perception_ready_topic", default_value=""),
        DeclareLaunchArgument("perception_heartbeat_timeout", default_value="1.0"),
        DeclareLaunchArgument("initial_state_ready_topic", default_value=""),
        DeclareLaunchArgument("publish_table", default_value="true"),
        DeclareLaunchArgument("environment_ready_topic", default_value=""),
        DeclareLaunchArgument("grasp_spin", default_value="0.0"),
        DeclareLaunchArgument("planning_candidates", default_value="5"),
        DeclareLaunchArgument("candidate_config", default_value=str(
            Path(get_package_share_directory("pick_place_mtc"))/"config/baseline_candidates.yaml")),
        DeclareLaunchArgument("candidate_archive", default_value=""),
        DeclareLaunchArgument("candidate_binding_files", default_value="[]"),
        DeclareLaunchArgument("scene_file", default_value=str(
            Path(get_package_share_directory("pick_place_demo")) / "config/known_scene.yaml")),
        DeclareLaunchArgument(
            "execution_fault_latch_file",
            default_value=os.path.expanduser("~/.ros/pick_place_execution_fault.json")),
        DeclareLaunchArgument(
            "world_file",
            default_value=str(Path(get_package_share_directory("robot_gazebo")) /
                              "worlds" / "rm65b_pick_place_fixed.sdf")),
        planner,
        Node(package="pick_place_sim", executable="attachment_transport",
             parameters=[{"use_sim_time": True}], output="screen"),
        Node(package="pick_place_sim", executable="execute_pick_place.py",
             parameters=[{"use_sim_time": True,
                          "auto_execute": LaunchConfiguration("auto_execute"),
                          "perception_ready_topic": LaunchConfiguration("perception_ready_topic"),
                          "perception_heartbeat_timeout": LaunchConfiguration("perception_heartbeat_timeout"),
                          "initial_state_ready_topic": LaunchConfiguration("initial_state_ready_topic"),
                          "execution_fault_latch_file":
                              LaunchConfiguration("execution_fault_latch_file")}],
             output="screen"),
    ])
