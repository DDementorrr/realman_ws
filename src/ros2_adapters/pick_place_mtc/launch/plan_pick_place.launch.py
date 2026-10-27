"""Start MTC planning with externally supplied MoveIt and task parameters."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    external = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("pick_place_mtc")) / "launch" /
                "plan_pick_place_external.launch.py")
        ),
        launch_arguments={
            "planning_config": LaunchConfiguration("planning_config"),
            "task_config": LaunchConfiguration("task_config"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "candidate_config": LaunchConfiguration("candidate_config"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("planning_config", description="External MoveIt parameter YAML"),
        DeclareLaunchArgument("task_config", description="External robot/task parameter YAML"),
        DeclareLaunchArgument("candidate_config", default_value=str(
            Path(get_package_share_directory("pick_place_mtc"))/"config/baseline_candidates.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        external,
    ])
