"""Run RGB-D pose estimation using externally supplied sensor and model config."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "perception_config",
            description="ROS parameter YAML with camera topics, frames, marker and object geometry",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="pick_place_perception",
            executable="aruco_object_pose",
            parameters=[LaunchConfiguration("perception_config"),
                        {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        ),
    ])
