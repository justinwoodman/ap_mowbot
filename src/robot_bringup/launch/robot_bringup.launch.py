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

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{'channel_type':'serial',
                    'serial_port': '/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1:1.0-port0', 
                    'serial_baudrate': 1000000, 
                    'frame_id': 'laser',
                    'inverted': False, 
                    'angle_compensate': True, 
                    'scan_mode': 'DenseBoost'}],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        robot_state_publisher_node,
        ardupilot_tf_publisher_node,
        lidar_node,
    ])
