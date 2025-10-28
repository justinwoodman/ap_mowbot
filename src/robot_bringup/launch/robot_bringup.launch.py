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

    ardupilot_odometry_bridge_node = Node(
        package='ardupilot_bridges',
        executable='ap_odometry_bridge',
    )

    ardupilot_navsat_bridge_node = Node(
        package='ardupilot_bridges',
        executable='ap_navsat_bridge',
    )

    ardupilot_cmd_vel_bridge_node = Node(
        package='ardupilot_bridges',
        executable='ap_cmd_vel_bridge',
    )

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type':'serial',
            'serial_port': '/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1:1.0-port0', 
            'serial_baudrate': 1000000, 
            'frame_id': 'laser',
            'inverted': False, 
            'angle_compensate': True, 
            'scan_mode': 'DenseBoost'
        }],
        output='screen'
    )

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        parameters=[{
            "enable_color" : True,
            "enable_infra" : False,
            "enable_infra1" : False,
            "enable_infra2" : False,
            "enable_depth" : False,
            "initial_reset" : True,
            "rgb_camera.color_profile" : "640x480x30",
            "rgb_camera.color_QoS" : "SENSOR_DATA",
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        robot_state_publisher_node,
        ardupilot_odometry_bridge_node,
        ardupilot_navsat_bridge_node,
        ardupilot_cmd_vel_bridge_node,
        lidar_node,
        #realsense_node,
    ])
