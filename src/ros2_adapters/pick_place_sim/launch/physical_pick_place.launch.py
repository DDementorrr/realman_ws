"""Stage 4: execute the same MTC task using contact alone, without a fixed joint."""

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
            "pick_place_attachment": LaunchConfiguration("pick_place_attachment"),
            "physical_grasp": LaunchConfiguration("physical_grasp"),
            "wrist_camera": LaunchConfiguration("wrist_camera"),
            "pad_compliance": LaunchConfiguration("pad_compliance"),
            "enable_mtc_execution": "true",
            "world_file": LaunchConfiguration("world_file"),
            "scene_file": LaunchConfiguration("scene_file"),
            "grasp_spin": "1.57079632679",
            "grasp_depth": "0.02",
            "grasp_lateral_offset": "-0.003",
            "grip_target": LaunchConfiguration("grip_target"),
            "open_target": "1.0",
            "gripper_motion_scaling": "0.15",
            "motion_velocity_scaling": "1.0",
            "motion_acceleration_scaling": "1.0",
            # Reduce inertial load on the contact-only grasp during the
            # horizontal transport segment.
            "transport_velocity_scaling": LaunchConfiguration("transport_velocity_scaling"),
            "transport_acceleration_scaling": LaunchConfiguration("transport_acceleration_scaling"),
            "transport_keep_orientation": LaunchConfiguration("transport_keep_orientation"),
            "transport_orientation_tolerance": LaunchConfiguration("transport_orientation_tolerance"),
            "place_lower_min": "0.01",
            "place_lower_max": "0.02",
            "place_height_offset": "0.02",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("auto_execute", default_value="false"),
        DeclareLaunchArgument("pick_place_attachment", default_value="false"),
        DeclareLaunchArgument("physical_grasp", default_value="true"),
        DeclareLaunchArgument("wrist_camera", default_value="false"),
        DeclareLaunchArgument("pad_compliance", default_value="false"),
        DeclareLaunchArgument("transport_velocity_scaling", default_value="0.15"),
        DeclareLaunchArgument("transport_acceleration_scaling", default_value="0.10"),
        DeclareLaunchArgument("transport_keep_orientation", default_value="false"),
        DeclareLaunchArgument("transport_orientation_tolerance", default_value="0.2"),
        DeclareLaunchArgument("physical_contact", default_value="true"),
        DeclareLaunchArgument("grip_target", default_value="0.40"),
        DeclareLaunchArgument(
            "world_file",
            default_value=str(Path(get_package_share_directory("robot_gazebo")) /
                              "worlds" / "rm65b_pick_place_physical.sdf")),
        DeclareLaunchArgument(
            "scene_file",
            default_value=str(Path(get_package_share_directory("pick_place_demo")) /
                              "config" / "physical_scene.yaml")),
        planner,
        Node(package="pick_place_sim", executable="attachment_transport",
             parameters=[{"use_sim_time": True}], output="screen"),
        Node(package="pick_place_sim", executable="execute_pick_place.py",
             parameters=[{"use_sim_time": True, "auto_execute": LaunchConfiguration("auto_execute"),
                          "physical_contact": LaunchConfiguration("physical_contact")}], output="screen"),
    ])
