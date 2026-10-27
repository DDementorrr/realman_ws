"""Geometry from external RGB-D/TF/model inputs; starts no sensor or robot."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('perception_config', description='Explicit depth_obstacles ROS parameter YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='pick_place_perception', executable='depth_obstacles', output='screen',
             parameters=[LaunchConfiguration('perception_config'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
