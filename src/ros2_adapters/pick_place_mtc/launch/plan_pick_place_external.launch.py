"""Start MTC planning with externally provided MoveIt parameters and inputs."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import Parameter


def generate_launch_description():
    planning_config = LaunchConfiguration("planning_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "planning_config",
            description="ROS parameter YAML containing the external robot and MoveIt configuration",
        ),
        DeclareLaunchArgument("candidate_config", default_value=str(
            Path(get_package_share_directory("pick_place_mtc"))/"config/baseline_candidates.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("task_config", description="Required robot/task parameter YAML"),
        Node(
            package="pick_place_mtc",
            executable="plan_pick_place",
            parameters=[planning_config, LaunchConfiguration("task_config"), LaunchConfiguration("candidate_config"),
                        Parameter("use_sim_time", value=use_sim_time, value_type=bool)],
            output="screen",
        ),
    ])
