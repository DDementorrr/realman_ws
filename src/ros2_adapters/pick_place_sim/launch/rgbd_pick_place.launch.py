"""Stage 5: contact-only scene with an independent Gazebo Harmonic RGB-D camera."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import UnlessCondition


def generate_launch_description():
    sim_share = Path(get_package_share_directory("pick_place_sim"))
    gazebo_share = Path(get_package_share_directory("robot_gazebo"))
    physical = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / "launch" / "physical_pick_place.launch.py")),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "auto_execute": LaunchConfiguration("auto_execute"),
            "scene_file": LaunchConfiguration("scene_file"),
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
    camera = Node(
        package="ros_gz_sim", executable="create", output="screen",
        condition=UnlessCondition(LaunchConfiguration("wrist_camera")),
        arguments=["-world", "empty", "-name", "pick_place_rgbd", "-file",
                   str(gazebo_share / "worlds" / "pick_place_rgbd_model.sdf"),
                   "-x", "0.55", "-y", "0", "-z", "1.20",
                   "-R", "0", "-P", "1.57079632679", "-Y", "0"],
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=[
            "/pick_place/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/pick_place/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/pick_place/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/pick_place/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/pick_place/rgbd/image", "/camera/camera/color/image_raw"),
            ("/pick_place/rgbd/depth_image", "/camera/camera/depth/image_rect_raw"),
            ("/pick_place/rgbd/camera_info", "/camera/camera/color/camera_info"),
            ("/pick_place/rgbd/points", "/camera/camera/depth/color/points"),
        ],
    )
    depth_info_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=["/pick_place/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        remappings=[("/pick_place/rgbd/camera_info", "/camera/camera/depth/camera_info")],
    )
    optical_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", output="screen",
        condition=UnlessCondition(LaunchConfiguration("wrist_camera")),
        arguments=["--x", "0.55", "--y", "0", "--z", "1.20",
                   "--roll", "3.14159265359", "--pitch", "0",
                   "--yaw", "-1.57079632679", "--frame-id", "world",
                   "--child-frame-id", "camera_color_optical_frame"],
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("auto_execute", default_value="false"),
        DeclareLaunchArgument("grip_target", default_value="0.40"),
        DeclareLaunchArgument("pick_place_attachment", default_value="false"),
        DeclareLaunchArgument("physical_grasp", default_value="true"),
        DeclareLaunchArgument("wrist_camera", default_value="false"),
        DeclareLaunchArgument("pad_compliance", default_value="false"),
        DeclareLaunchArgument("transport_velocity_scaling", default_value="0.15"),
        DeclareLaunchArgument("transport_acceleration_scaling", default_value="0.10"),
        DeclareLaunchArgument("transport_keep_orientation", default_value="false"),
        DeclareLaunchArgument("transport_orientation_tolerance", default_value="0.2"),
        DeclareLaunchArgument("physical_contact", default_value="true"),
        DeclareLaunchArgument(
            "world_file",
            default_value=str(Path(get_package_share_directory("robot_gazebo")) /
                              "worlds" / "rm65b_pick_place_physical.sdf")),
        DeclareLaunchArgument(
            "scene_file",
            default_value=str(Path(get_package_share_directory("pick_place_demo")) /
                              "config" / "physical_scene.yaml")),
        physical,
        TimerAction(period=10.0, actions=[camera]),
        bridge,
        depth_info_bridge,
        optical_tf,
    ])
