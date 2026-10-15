from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scene_world = Path(get_package_share_directory("robot_gazebo")) / "worlds" / "rm65b_pick_place.sdf"
    scene_config = Path(get_package_share_directory("pick_place_demo")) / "config" / "known_scene.yaml"
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("manipulation_bringup"), "launch", "realman_moveit_gz.launch.py"])
        ),
        launch_arguments={
            "with_gripper": LaunchConfiguration("with_gripper"),
            "pick_place_attachment": LaunchConfiguration("pick_place_attachment"),
            "physical_grasp": LaunchConfiguration("physical_grasp"),
            "wrist_camera": LaunchConfiguration("wrist_camera"),
            "pad_compliance": LaunchConfiguration("pad_compliance"),
            "enable_mtc_execution": LaunchConfiguration("enable_mtc_execution"),
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "world_file": LaunchConfiguration("world_file"),
        }.items(),
    )
    scene = Node(
        package="pick_place_demo",
        executable="known_scene",
        parameters=[LaunchConfiguration("scene_file"), {"use_sim_time": True}],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("with_gripper", default_value="true"),
        DeclareLaunchArgument("pick_place_attachment", default_value="false"),
        DeclareLaunchArgument("physical_grasp", default_value="false"),
        DeclareLaunchArgument("wrist_camera", default_value="false"),
        DeclareLaunchArgument("pad_compliance", default_value="false"),
        DeclareLaunchArgument("enable_mtc_execution", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("world_file", default_value=str(scene_world)),
        DeclareLaunchArgument("scene_file", default_value=str(scene_config)),
        base,
        scene,
    ])
