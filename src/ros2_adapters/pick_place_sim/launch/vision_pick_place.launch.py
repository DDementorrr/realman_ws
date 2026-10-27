"""Stage 6: RGB-D ArUco localization feeding the unchanged MTC task."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    sim_share = Path(get_package_share_directory("pick_place_sim"))
    demo_share = Path(get_package_share_directory("pick_place_demo"))
    rgbd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / "launch" / "rgbd_pick_place.launch.py")),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "auto_execute": LaunchConfiguration("auto_execute"),
            "scene_file": str(demo_share / "config" / "vision_scene.yaml"),
            "grip_target": LaunchConfiguration("grip_target"),
            "pick_place_attachment": LaunchConfiguration("pick_place_attachment"),
            "physical_grasp": LaunchConfiguration("physical_grasp"),
            "wrist_camera": LaunchConfiguration("wrist_camera"),
            "pad_compliance": LaunchConfiguration("pad_compliance"),
            "transport_velocity_scaling": LaunchConfiguration("transport_velocity_scaling"),
            "transport_acceleration_scaling": LaunchConfiguration("transport_acceleration_scaling"),
            "transport_keep_orientation": LaunchConfiguration("transport_keep_orientation"),
            "transport_orientation_tolerance": LaunchConfiguration("transport_orientation_tolerance"),
            "physical_contact": LaunchConfiguration("physical_contact"),
            "world_file": LaunchConfiguration("world_file"),
        }.items(),
    )
    perception = Node(
        package="pick_place_perception", executable="aruco_object_pose", output="screen",
        parameters=[{
            "use_sim_time": True,
            "wait_for_observation": ParameterValue(LaunchConfiguration("wrist_camera"), value_type=bool),
            "marker_id": ParameterValue(LaunchConfiguration("marker_id"), value_type=int),
            "marker_center_offset": ParameterValue(LaunchConfiguration("marker_center_offset")),
            "marker_size": 0.045,
            "object_id": "pick_object",
            "rgb_image_topic": "/camera/camera/color/image_raw",
            "depth_image_topic": "/camera/camera/depth/image_rect_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "depth_encoding": "32FC1",
            "depth_scale": 1.0,
            "max_time_skew": 0.1,
            "object_size": [0.05, 0.07, 0.22],
            "camera_frame": "camera_color_optical_frame",
            "output_frame": "base_link",
        }],
    )
    scene_sync = Node(package="pick_place_scene", executable="scene_sync", output="screen",
                      parameters=[{"use_sim_time": True, "allowed_ids": ["pick_object"]}])
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("auto_execute", default_value="true"),
        # The wrist observation pose views the object's vertical -X face.
        DeclareLaunchArgument("marker_id", default_value="1"),
        DeclareLaunchArgument("marker_center_offset", default_value="[0.026, 0.0, -0.055]"),
        DeclareLaunchArgument("grip_target", default_value="0.40"),
        DeclareLaunchArgument("pick_place_attachment", default_value="false"),
        DeclareLaunchArgument("physical_grasp", default_value="true"),
        DeclareLaunchArgument("wrist_camera", default_value="true"),
        DeclareLaunchArgument("pad_compliance", default_value="false"),
        DeclareLaunchArgument("transport_velocity_scaling", default_value="0.15"),
        DeclareLaunchArgument("transport_acceleration_scaling", default_value="0.10"),
        DeclareLaunchArgument("transport_keep_orientation", default_value="false"),
        DeclareLaunchArgument("transport_orientation_tolerance", default_value="0.2"),
        # Keep the side marker in view with a useful working distance and align
        # the camera laterally with the object's -X face.
        # The base was lowered by 25 cm; raise the TCP target by the same
        # amount so the wrist camera keeps its established world observation.
        DeclareLaunchArgument("observation_xyz", default_value="[0.18, -0.075, 0.40]"),
        DeclareLaunchArgument("observation_quaternion", default_value="[0.5, 0.5, 0.5, 0.5]"),
        DeclareLaunchArgument("physical_contact", default_value="true"),
        DeclareLaunchArgument(
            "world_file",
            default_value=str(Path(get_package_share_directory("robot_gazebo")) /
                              "worlds" / "rm65b_pick_place_physical.sdf")),
        rgbd,
        Node(package="pick_place_demo", executable="wrist_observation", output="screen",
             condition=IfCondition(LaunchConfiguration("wrist_camera")),
             parameters=[{"use_sim_time": True,
                          "observation_xyz": ParameterValue(LaunchConfiguration("observation_xyz")),
                          "observation_quaternion": ParameterValue(LaunchConfiguration("observation_quaternion"))}]),
        perception,
        scene_sync,
    ])
