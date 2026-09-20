import os
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def continue_on_success(label, next_actions):
    def handle_exit(event, _context):
        if event.returncode == 0:
            return next_actions
        reason = f"{label} failed with exit code {event.returncode}"
        return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]

    return handle_exit


def launch_setup(context):
    with_gripper = LaunchConfiguration("with_gripper").perform(context).lower() in {"1", "true", "yes"}
    planner_id = LaunchConfiguration("planner_id").perform(context)
    allowed_planners = {
        "RRTConnectkConfigDefault",
        "RRTkConfigDefault",
        "RRTstarkConfigDefault",
        "PRMkConfigDefault",
    }
    if planner_id not in allowed_planners:
        raise RuntimeError("planner_id must be one of: " + ", ".join(sorted(allowed_planners)))

    planning_time = float(LaunchConfiguration("planning_time").perform(context))
    velocity_scaling = float(LaunchConfiguration("velocity_scaling").perform(context))
    acceleration_scaling = float(LaunchConfiguration("acceleration_scaling").perform(context))
    if planning_time <= 0.0:
        raise RuntimeError("planning_time must be positive")
    if not 0.0 < velocity_scaling <= 1.0 or not 0.0 < acceleration_scaling <= 1.0:
        raise RuntimeError("velocity_scaling and acceleration_scaling must be in (0, 1]")

    description_share = Path(get_package_share_directory("rm_description"))
    gripper_share = Path(get_package_share_directory("robot_changingtek_description")) if with_gripper else None
    gazebo_share = Path(get_package_share_directory("robot_gazebo"))
    moveit_share = Path(get_package_share_directory("robot_realman_moveit_config"))
    planning_urdf = (
        moveit_share / "urdf" / "rm65b_ctm2f110.urdf.xacro"
        if with_gripper else description_share / "urdf" / "rm_65.urdf.xacro"
    )
    gazebo_xacro = gazebo_share / "urdf" / (
        "rm65b_ctm2f110_gz.urdf.xacro" if with_gripper else "rm65b_gz.urdf.xacro"
    )
    world = gazebo_share / "worlds" / (
        "empty_mimic.sdf" if with_gripper else "empty.sdf"
    )

    moveit_config = (
        MoveItConfigsBuilder("rm65b", package_name="robot_realman_moveit_config")
        .robot_description(file_path=str(planning_urdf), mappings={"link6_type": "Link6"})
        .robot_description_semantic(file_path=(
            "config/rm65b_ctm2f110.srdf" if with_gripper else "config/rm65b.srdf"
        ))
        .robot_description_kinematics(file_path="config/rm65b_kinematics.yaml")
        .joint_limits(file_path=(
            "config/rm65b_ctm2f110_joint_limits.yaml" if with_gripper
            else "config/rm65b_joint_limits.yaml"
        ))
        .trajectory_execution(file_path=(
            "config/rm65b_ctm2f110_moveit_controllers.yaml"
            if with_gripper else "config/rm65b_moveit_controllers.yaml"
        ))
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    if with_gripper:
        # Keep the accepted bare-arm OMPL configuration unchanged.
        ompl_file = moveit_share / "config" / "rm65b_ctm2f110_ompl_planning.yaml"
        moveit_config.planning_pipelines["ompl"] = yaml.safe_load(ompl_file.read_text())

    gazebo_xml = Command([FindExecutable(name="xacro"), " ", str(gazebo_xacro)])
    gazebo_description = ParameterValue(gazebo_xml, value_type=str)
    headless = LaunchConfiguration("headless").perform(context).lower() in {"1", "true", "yes"}
    # This migration deliberately selects Gazebo Harmonic (gz-sim 8) on Humble.
    # The Harmonic package set must be installed before launching this file.
    gz_args = f"-v 3 {'-s ' if headless else ''}{world} --force-version 8"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        # Deliberately no -r: configure first, then unpause and activate.
        launch_arguments={"gz_args": gz_args}.items(),
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-world", "empty", "-name", "rm65b", "-string", gazebo_xml],
        output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="screen",
    )
    gazebo_description_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="gazebo_description_publisher",
        parameters=[{"robot_description": gazebo_description, "use_sim_time": True}],
        remappings=[
            ("robot_description", "/robot_description_gazebo"),
            ("joint_states", "/gazebo_description_unused/joint_states"),
            ("/tf", "/gazebo_description_unused/tf"),
            ("/tf_static", "/gazebo_description_unused/tf_static"),
        ],
        output="screen",
    )
    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    controllers = ["joint_state_broadcaster", "rm65b_arm_controller"]
    if with_gripper:
        controllers.append("ctm2f110_hand_controller")
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[*controllers, "--inactive", "--controller-manager-timeout", "90"],
        output="screen",
    )
    unpause = ExecuteProcess(
        cmd=[
            "gz", "service", "-s", "/world/empty/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000", "--req", "pause: false",
        ],
        output="screen",
    )
    activate = ExecuteProcess(
        cmd=[
            "ros2", "control", "switch_controllers", "--strict", "--activate-asap",
            "--activate", *controllers,
        ],
        output="screen",
    )
    stop_gazebo = ExecuteProcess(
        cmd=[
            "gz", "service", "-s", "/server_control",
            "--reqtype", "gz.msgs.ServerControl", "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000", "--req", "stop: true",
        ],
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(moveit_share / "rviz" / "rm65b.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )
    target_executor = Node(
        package="manipulation_moveit_adapter",
        executable="target_pose_executor",
        name="rm65b_target_pose_executor",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "use_sim_time": True,
                "planning_group": "rm65_arm",
                "target_topic": "/manipulation/rm65b/target_pose",
                "planner_id": planner_id,
                "planning_time": planning_time,
                "velocity_scaling": velocity_scaling,
                "acceleration_scaling": acceleration_scaling,
            },
        ],
        output="screen",
    )

    return [
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.pathsep.join(
                value for value in [
                    str(description_share.parent),
                    str(gripper_share.parent) if gripper_share else "",
                    os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                ] if value
            ),
        ),
        gazebo,
        clock_bridge,
        robot_state_publisher,
        gazebo_description_publisher,
        world_to_base,
        spawn_robot,
        RegisterEventHandler(OnShutdown(on_shutdown=[stop_gazebo])),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=continue_on_success("RM65B spawn", [controller_spawner]),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_spawner,
                on_exit=continue_on_success("controller configuration", [unpause]),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=unpause,
                on_exit=continue_on_success("Gazebo unpause", [activate]),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=activate,
                on_exit=continue_on_success(
                    "controller activation", [move_group, rviz, target_executor]
                ),
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("with_gripper", default_value="false"),
            DeclareLaunchArgument("planner_id", default_value="RRTConnectkConfigDefault"),
            DeclareLaunchArgument("planning_time", default_value="5.0"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.15"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.15"),
            OpaqueFunction(function=launch_setup),
        ]
    )
