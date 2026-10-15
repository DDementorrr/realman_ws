from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("readback", default_value="true"),
        DeclareLaunchArgument("min_position", default_value="0.0"),
        DeclareLaunchArgument("max_position", default_value="1.0"),
        Node(
            package="changingtek_gripper_driver",
            executable="ctm2f110_driver",
            name="ctm2f110_driver",
            namespace="ctm2f110_hand_controller",
            parameters=[{"serial_port": LaunchConfiguration("serial_port"),
                         "baud_rate": LaunchConfiguration("baud_rate"),
                         "dry_run": LaunchConfiguration("dry_run"),
                         "readback": LaunchConfiguration("readback"),
                         "min_position": LaunchConfiguration("min_position"),
                         "max_position": LaunchConfiguration("max_position")}],
            remappings=[("joint_states", "/joint_states")],
            output="screen",
        )
    ])
