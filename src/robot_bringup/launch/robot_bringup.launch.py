from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='robot_bringup').find('robot_bringup')
    default_model_path = os.path.join(pkg_share, 'urdf', 'robot_description.urdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
    )

    ardupilot_tf_publisher_node = Node(
        package='ardupilot_tf_publisher',
        executable='ardupilot_tf_publisher',
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        robot_state_publisher_node,
        ardupilot_tf_publisher_node,
    ])
