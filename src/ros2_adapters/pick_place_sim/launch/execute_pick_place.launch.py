"""Stage 3: execute the known-pose MTC task with Gazebo attachment feedback."""

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
            str(Path(get_package_share_directory("pick_place_mtc")) / "launch" / "plan_pick_place.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "pick_place_attachment": "true",
            "enable_mtc_execution": "true",
            "world_file": LaunchConfiguration("world_file"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("auto_execute", default_value="false"),
        DeclareLaunchArgument(
            "world_file",
            default_value=str(Path(get_package_share_directory("robot_gazebo")) /
                              "worlds" / "rm65b_pick_place_fixed.sdf")),
        planner,
        Node(package="pick_place_sim", executable="attachment_transport",
             parameters=[{"use_sim_time": True}], output="screen"),
        Node(package="pick_place_sim", executable="execute_pick_place.py",
             parameters=[{"use_sim_time": True, "auto_execute": LaunchConfiguration("auto_execute")}],
             output="screen"),
    ])
