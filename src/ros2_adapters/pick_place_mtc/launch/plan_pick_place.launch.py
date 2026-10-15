"""Bring up the known scene and plan the MTC task in one launch."""

from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_share = Path(get_package_share_directory("robot_realman_moveit_config"))
    moveit_config = (
        MoveItConfigsBuilder("rm65b", package_name="robot_realman_moveit_config")
        .robot_description(
            file_path=str(moveit_share / "urdf" / "rm65b_ctm2f110.urdf.xacro"),
            mappings={"link6_type": "Link6", "wrist_camera": LaunchConfiguration("wrist_camera"),
                      "pad_compliance": LaunchConfiguration("pad_compliance")},
        )
        .robot_description_semantic(file_path="config/rm65b_ctm2f110.srdf")
        .robot_description_kinematics(file_path="config/rm65b_kinematics.yaml")
        .joint_limits(file_path="config/rm65b_ctm2f110_joint_limits.yaml")
        .trajectory_execution(file_path="config/rm65b_ctm2f110_moveit_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    ompl_file = moveit_share / "config" / "rm65b_ctm2f110_ompl_planning.yaml"
    moveit_config.planning_pipelines["ompl"] = yaml.safe_load(ompl_file.read_text())
    # Pose-space interpolation can jump between IK branches and fail final
    # path validation. Check carry constraints along joint-space interpolation.
    moveit_config.planning_pipelines["ompl"]["rm65_arm"]["enforce_joint_model_state_space"] = ParameterValue(
        LaunchConfiguration("transport_keep_orientation"), value_type=bool)
    known_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("pick_place_demo")) / "launch" / "known_scene.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "with_gripper": "true",
            "pick_place_attachment": LaunchConfiguration("pick_place_attachment"),
            "physical_grasp": LaunchConfiguration("physical_grasp"),
            "wrist_camera": LaunchConfiguration("wrist_camera"),
            "pad_compliance": LaunchConfiguration("pad_compliance"),
            "enable_mtc_execution": LaunchConfiguration("enable_mtc_execution"),
            "world_file": LaunchConfiguration("world_file"),
            "scene_file": LaunchConfiguration("scene_file"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("pick_place_attachment", default_value="false"),
        DeclareLaunchArgument("physical_grasp", default_value="false"),
        DeclareLaunchArgument("wrist_camera", default_value="false"),
        DeclareLaunchArgument("pad_compliance", default_value="false"),
        DeclareLaunchArgument("enable_mtc_execution", default_value="false"),
        DeclareLaunchArgument("grasp_spin", default_value="0.0"),
        DeclareLaunchArgument("planning_candidates", default_value="5"),
        DeclareLaunchArgument("grasp_depth", default_value="0.0"),
        DeclareLaunchArgument("grasp_lateral_offset", default_value="0.0"),
        DeclareLaunchArgument("grip_target", default_value="0.6"),
        DeclareLaunchArgument("open_target", default_value="0.8"),
        DeclareLaunchArgument("gripper_motion_scaling", default_value="1.0"),
        DeclareLaunchArgument("motion_velocity_scaling", default_value="1.0"),
        DeclareLaunchArgument("motion_acceleration_scaling", default_value="1.0"),
        DeclareLaunchArgument("transport_velocity_scaling", default_value="1.0"),
        DeclareLaunchArgument("transport_acceleration_scaling", default_value="1.0"),
        DeclareLaunchArgument("transport_keep_orientation", default_value="false"),
        DeclareLaunchArgument("transport_orientation_tolerance", default_value="0.2"),
        DeclareLaunchArgument("place_lower_min", default_value="0.08"),
        DeclareLaunchArgument("place_lower_max", default_value="0.12"),
        DeclareLaunchArgument("place_height_offset", default_value="0.0"),
        DeclareLaunchArgument(
            "world_file",
            default_value=str(Path(get_package_share_directory("robot_gazebo")) / "worlds" / "rm65b_pick_place.sdf"),
        ),
        DeclareLaunchArgument(
            "scene_file",
            default_value=str(Path(get_package_share_directory("pick_place_demo")) / "config" / "known_scene.yaml"),
        ),
        known_scene,
        Node(
            package="pick_place_mtc",
            executable="plan_pick_place",
            arguments=["--ros-args", "--log-level",
                       "moveit_kinematic_constraints.kinematic_constraints:=warn"],
            parameters=[moveit_config.to_dict(), {
                "use_sim_time": True,
                "planning_candidates": ParameterValue(LaunchConfiguration("planning_candidates"), value_type=int),
                "joint_motion_weights": [5.0, 4.0, 3.0, 2.0, 1.0, 1.0],
                "grasp_spin": ParameterValue(LaunchConfiguration("grasp_spin"), value_type=float),
                "grasp_depth": ParameterValue(LaunchConfiguration("grasp_depth"), value_type=float),
                "grasp_lateral_offset": ParameterValue(
                    LaunchConfiguration("grasp_lateral_offset"), value_type=float),
                "grip_target": ParameterValue(LaunchConfiguration("grip_target"), value_type=float),
                "open_target": ParameterValue(LaunchConfiguration("open_target"), value_type=float),
                "gripper_motion_scaling": ParameterValue(LaunchConfiguration("gripper_motion_scaling"), value_type=float),
                "motion_velocity_scaling": ParameterValue(
                    LaunchConfiguration("motion_velocity_scaling"), value_type=float),
                "motion_acceleration_scaling": ParameterValue(
                    LaunchConfiguration("motion_acceleration_scaling"), value_type=float),
                "transport_velocity_scaling": ParameterValue(
                    LaunchConfiguration("transport_velocity_scaling"), value_type=float),
                "transport_acceleration_scaling": ParameterValue(
                    LaunchConfiguration("transport_acceleration_scaling"), value_type=float),
                "transport_keep_orientation": ParameterValue(
                    LaunchConfiguration("transport_keep_orientation"), value_type=bool),
                "transport_orientation_tolerance": ParameterValue(
                    LaunchConfiguration("transport_orientation_tolerance"), value_type=float),
                "place_lower_min": ParameterValue(LaunchConfiguration("place_lower_min"), value_type=float),
                "place_lower_max": ParameterValue(LaunchConfiguration("place_lower_max"), value_type=float),
                "place_height_offset": ParameterValue(
                    LaunchConfiguration("place_height_offset"), value_type=float),
            }],
            output="screen",
        )
    ])
