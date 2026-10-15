"""Start the known scene with the optional Gazebo fixed-joint attachment adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    known_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("pick_place_demo")) / "launch" / "known_scene.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "with_gripper": "true",
            "pick_place_attachment": "true",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        known_scene,
        Node(
            package="pick_place_sim",
            executable="attachment_transport",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ])
