"""Stage 1: frozen U workbench, public scene owner, existing fixed execution."""

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pick_place_sim.workbench_world import generate_world


def setup(context):
    geometry = str(Path(LaunchConfiguration("workbench_file").perform(context)).resolve())
    gazebo = Path(get_package_share_directory("robot_gazebo"))
    temporary = tempfile.TemporaryDirectory(prefix="rm65b-workbench-")
    try:
        # Both consumers use this frozen snapshot, including during startup.
        frozen_geometry = Path(temporary.name) / "geometry.yaml"
        frozen_geometry.write_bytes(Path(geometry).read_bytes())
        world = generate_world(
            gazebo / "worlds/rm65b_pick_place_fixed.sdf", frozen_geometry,
            gazebo / "urdf/rm65b_ctm2f110_gz.urdf.xacro", Path(temporary.name) / "world.sdf")
    except Exception:
        temporary.cleanup()
        raise

    def cleanup(_context):
        temporary.cleanup()
        return []

    execute = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory("pick_place_sim")) /
                                          "launch/execute_pick_place.launch.py")),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name in
               ("headless", "use_rviz", "auto_execute", "execution_fault_latch_file")},
            "world_file": str(world), "publish_table": "false",
            "environment_ready_topic": "/perception/scene_ready",
        }.items())
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package="pick_place_scene", executable="scene_sync", parameters=[{
            "use_sim_time": True, "static_scene_file": str(frozen_geometry),
            "observation_topic": "/workbench/collision_objects",
            "allowed_ids": ["pick_table", "left_table", "right_table"],
        }], output="screen"),
        execute,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("auto_execute", default_value="false"),
        DeclareLaunchArgument("execution_fault_latch_file",
                              default_value=os.path.expanduser("~/.ros/pick_place_execution_fault.json")),
        DeclareLaunchArgument("workbench_file", default_value=str(
            Path(get_package_share_directory("pick_place_scene")) / "config/u_workbench.yaml")),
        OpaqueFunction(function=setup),
    ])
