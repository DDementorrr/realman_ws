import os
from pathlib import Path
import xml.etree.ElementTree as ET
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
from launch_ros.utilities import evaluate_parameters, normalize_parameters
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
    physical_grasp = LaunchConfiguration("physical_grasp").perform(context).lower() in {"1", "true", "yes"}
    enable_mtc_execution = LaunchConfiguration("enable_mtc_execution").perform(context).lower() in {"1", "true", "yes"}
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
    world_arg = LaunchConfiguration("world_file").perform(context).strip()
    world = Path(world_arg) if world_arg else gazebo_share / "worlds" / (
        "empty_mimic.sdf" if with_gripper else "empty.sdf"
    )
    if not world.is_file():
        raise RuntimeError(f"Gazebo world file does not exist: {world}")

    moveit_config = (
        MoveItConfigsBuilder("rm65b", package_name="robot_realman_moveit_config")
        .robot_description(file_path=str(planning_urdf), mappings={"link6_type": "Link6", "wrist_camera": LaunchConfiguration("wrist_camera"), "pad_compliance": LaunchConfiguration("pad_compliance")})
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

    gazebo_xml = Command([
        FindExecutable(name="xacro"), " ", str(gazebo_xacro),
        *([" wrist_camera:=", LaunchConfiguration("wrist_camera")] if with_gripper else []),
        *([" wrist_camera_sensor:=", LaunchConfiguration("wrist_camera_sensor")] if with_gripper else []),
        *([" pad_compliance:=", LaunchConfiguration("pad_compliance")] if with_gripper else []),
        *([" pick_place_attachment:=", LaunchConfiguration("pick_place_attachment")]
          if with_gripper else []),
        *([" physical_grasp:=", LaunchConfiguration("physical_grasp")]
          if with_gripper else []),
    ])
    gazebo_description = ParameterValue(gazebo_xml, value_type=str)
    headless = LaunchConfiguration("headless").perform(context).lower() in {"1", "true", "yes"}
    # This migration deliberately selects Gazebo Harmonic (gz-sim 8) on Humble.
    # The Harmonic package set must be installed before launching this file.
    # ros_gz_sim appends ``--force-version`` from its ``gz_version`` launch
    # argument.  Do not repeat it here: gz-sim rejects duplicate version flags.
    gz_args = f"-v 3 {'-s ' if headless else ''}{world}"

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
    feedback_description = moveit_config.robot_description
    feedback_remappings = []
    if with_gripper:
        # RSP normally overwrites measured mimic joint positions with the
        # leader's ideal ratio. Gazebo publishes every linkage joint, so TF
        # must use that feedback, including real constraint error under load.
        # MoveIt and Gazebo retain their original mimic relations.
        xml = evaluate_parameters(context, normalize_parameters([moveit_config.robot_description]))[0]['robot_description']
        feedback_model = ET.fromstring(xml)
        for joint in feedback_model.findall('joint'):
            mimic = joint.find('mimic')
            if joint.get('name', '').startswith('ctm2f110_') and mimic is not None:
                joint.remove(mimic)
        feedback_description = {'robot_description': ET.tostring(feedback_model, encoding='unicode')}
        feedback_remappings = [('robot_description', '/robot_description_feedback')]
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[feedback_description, {"use_sim_time": True, "publish_frequency": 250.0}],
        remappings=feedback_remappings,
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
        arguments=["--frame-id", "world", "--child-frame-id", "base_link", "--z", "0.15"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    controllers = ["joint_state_broadcaster", "rm65b_arm_controller"]
    if with_gripper:
        controllers.append("ctm2f110_hand_controller")
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # Configure while Gazebo is paused; activation happens after unpause.
        # A non-headless Gazebo GUI may take well over a minute to load its
        # rendering plugins before controller_manager becomes available.
        arguments=[*controllers, "--inactive", "--controller-manager-timeout", "180"],
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
    controller_activator = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *controllers, "--activate-as-group", "--controller-manager-timeout", "180",
            "--switch-timeout", "30",
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
    shutdown_stop_requested = {"value": False}

    def stop_gazebo_once(_context):
        if shutdown_stop_requested["value"]:
            return []
        shutdown_stop_requested["value"] = True
        return [stop_gazebo]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            # Contact-mode Gazebo hand motion settles slower than its nominal
            # trajectory duration; allow the controller time to report success.
            {"trajectory_execution.allowed_execution_duration_scaling": 5.0},
            *([{"trajectory_execution.allowed_start_tolerance": 0.02}] if physical_grasp else []),
            *([{"capabilities": "move_group/ExecuteTaskSolutionCapability"}] if enable_mtc_execution else []),
        ],
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
                    # World-local visual assets (for example the ArUco marker
                    # textures) are referenced by filename from SDF files.
                    str(gazebo_share.parent),
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
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=stop_gazebo_once)])
        ),
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
                on_exit=continue_on_success(
                    "Gazebo unpause", [controller_activator]
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_activator,
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
            DeclareLaunchArgument("pick_place_attachment", default_value="false"),
            DeclareLaunchArgument("physical_grasp", default_value="false"),
            DeclareLaunchArgument("wrist_camera", default_value="true"),
            DeclareLaunchArgument("wrist_camera_sensor", default_value="true"),
            DeclareLaunchArgument("pad_compliance", default_value="false"),
            DeclareLaunchArgument("enable_mtc_execution", default_value="false"),
            DeclareLaunchArgument("world_file", default_value=""),
            DeclareLaunchArgument("planner_id", default_value="RRTConnectkConfigDefault"),
            DeclareLaunchArgument("planning_time", default_value="5.0"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.15"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.15"),
            OpaqueFunction(function=launch_setup),
        ]
    )
