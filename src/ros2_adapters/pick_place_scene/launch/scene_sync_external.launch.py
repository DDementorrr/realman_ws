"""Run PlanningScene synchronization against externally supplied MoveIt services."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene_config",
            description="ROS parameter YAML with observation topic, service names, IDs and freshness limit",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="pick_place_scene",
            executable="scene_sync",
            parameters=[LaunchConfiguration("scene_config"),
                        {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        ),
    ])
