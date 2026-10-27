"""Simulation orchestration for the known scene and MTC planning node."""

from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import Parameter, ParameterValue
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
            "wrist_camera_sensor": LaunchConfiguration("wrist_camera_sensor"),
            "pad_compliance": LaunchConfiguration("pad_compliance"),
            "enable_mtc_execution": LaunchConfiguration("enable_mtc_execution"),
            "world_file": LaunchConfiguration("world_file"),
            "scene_file": LaunchConfiguration("scene_file"),
            "publish_table": LaunchConfiguration("publish_table"),
            "environment_ready_topic": LaunchConfiguration("environment_ready_topic"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("pick_place_attachment", default_value="false"),
        DeclareLaunchArgument("physical_grasp", default_value="false"),
        DeclareLaunchArgument("wrist_camera", default_value="true"),
        DeclareLaunchArgument("wrist_camera_sensor", default_value="true"),
        DeclareLaunchArgument("pad_compliance", default_value="false"),
        DeclareLaunchArgument("enable_mtc_execution", default_value="false"),
        DeclareLaunchArgument("publish_table", default_value="true"),
        DeclareLaunchArgument("environment_ready_topic", default_value=""),
        DeclareLaunchArgument("grasp_spin", default_value="0.0"),
        DeclareLaunchArgument("planning_candidates", default_value="5"),
        DeclareLaunchArgument("candidate_config", default_value=str(
            Path(get_package_share_directory("pick_place_mtc"))/"config/baseline_candidates.yaml")),
        DeclareLaunchArgument("candidate_archive", default_value=""),
        DeclareLaunchArgument("candidate_binding_files", default_value="[]"),
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
        DeclareLaunchArgument("world_file", default_value=str(
            Path(get_package_share_directory("robot_gazebo")) / "worlds" / "rm65b_pick_place.sdf")),
        DeclareLaunchArgument("scene_file", default_value=str(
            Path(get_package_share_directory("pick_place_demo")) / "config" / "known_scene.yaml")),
        known_scene,
        Node(
            package="pick_place_mtc", executable="plan_pick_place",
            arguments=["--ros-args", "--log-level",
                       "moveit_kinematic_constraints.kinematic_constraints:=warn"],
            parameters=[moveit_config.to_dict(),
                        str(Path(get_package_share_directory("pick_place_demo")) / "config" / "mtc_demo.yaml"),
                        LaunchConfiguration("candidate_config"),
                        Parameter("plan_pick_place:candidate_archive", value=LaunchConfiguration("candidate_archive"), value_type=str),
                        Parameter("plan_pick_place:candidate_binding_files", value=LaunchConfiguration("candidate_binding_files"), value_type=str),
                        # Bind overrides to this planner; unqualified rules
                        # can lose to name-scoped task YAML on Humble.
                        Parameter("plan_pick_place:use_sim_time", value=True),
                        Parameter("plan_pick_place:trajectory_execution.allowed_execution_duration_scaling", value=5.0),
                        *[Parameter('plan_pick_place:'+name, value=LaunchConfiguration(name), value_type=kind) for name, kind in {
                "planning_candidates": int,
                "grasp_spin": float,
                "grasp_depth": float,
                "grasp_lateral_offset": float,
                "grip_target": float,
                "open_target": float,
                "gripper_motion_scaling": float,
                "motion_velocity_scaling": float,
                "motion_acceleration_scaling": float,
                "transport_velocity_scaling": float,
                "transport_acceleration_scaling": float,
                "transport_keep_orientation": bool,
                "transport_orientation_tolerance": float,
                "place_lower_min": float,
                "place_lower_max": float,
                "place_height_offset": float,
            }.items()]], output="screen")
    ])
