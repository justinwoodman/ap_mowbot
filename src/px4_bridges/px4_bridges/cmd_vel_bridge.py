#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from px4_msgs.msg import RoverRateSetpoint, RoverSpeedSetpoint


class Px4CmdVelBridge(Node):
    """
    ROS2 node that converts cmd_vel (Twist) messages to PX4 rover messages.
    
    Subscribes to:
        - /cmd_vel (geometry_msgs/Twist)
    
    Publishes:
        - /fmu/in/rover_rate_setpoint (px4_msgs/RoverRateSetpoint)
        - /fmu/in/rover_speed_setpoint (px4_msgs/RoverSpeedSetpoint)
    """
    
    def __init__(self):
        super().__init__('px4_cmd_vel_bridge')

        # Configure QoS profile for PX4 (Best Effort, Volatile)
        qos_profile_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for companion computer ROS2 topics (Reliable, Volatile)
        qos_profile_ros2 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile_ros2
        )
        
        # Create publishers for PX4 messages
        self.rate_setpoint_pub = self.create_publisher(
            RoverRateSetpoint,
            '/fmu/in/rover_rate_setpoint',
            qos_profile_px4
        )
        
        self.speed_setpoint_pub = self.create_publisher(
            RoverSpeedSetpoint,
            '/fmu/in/rover_speed_setpoint',
            qos_profile_px4
        )
        
        self.get_logger().info('CMD_VEL to PX4 Rover converter node started')
        self.get_logger().info('Subscribed to: /cmd_vel')
        self.get_logger().info('Publishing to: /fmu/in/rover_rate_setpoint')
        self.get_logger().info('Publishing to: /fmu/in/rover_speed_setpoint')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for cmd_vel messages.
        Converts Twist message to PX4 RoverRateSetpoint and RoverSpeedSetpoint.
        
        Twist mapping:
            - linear.x: forward/backward speed (m/s)
            - angular.z: yaw rate (rad/s)
        """
        # Create and publish RoverRateSetpoint
        rate_msg = RoverRateSetpoint()
        rate_msg.timestamp = self.get_clock().now().nanoseconds // 1000  # microseconds
        rate_msg.forward_speed_setpoint = float(msg.linear.x)
        rate_msg.forward_speed_setpoint_normalized = float('nan')  # Not used
        rate_msg.yaw_rate_setpoint = float(msg.angular.z)
        rate_msg.yaw_rate_setpoint_normalized = float('nan')  # Not used
        
        self.rate_setpoint_pub.publish(rate_msg)
        
        # Create and publish RoverSpeedSetpoint
        speed_msg = RoverSpeedSetpoint()
        speed_msg.timestamp = self.get_clock().now().nanoseconds // 1000  # microseconds
        speed_msg.forward_speed_setpoint = float(msg.linear.x)
        speed_msg.forward_speed_setpoint_normalized = float('nan')  # Not used
        speed_msg.yaw_setpoint = float('nan')  # Not used in rate control
        speed_msg.yaw_rate_setpoint = float(msg.angular.z)
        speed_msg.yaw_rate_setpoint_normalized = float('nan')  # Not used
        
        self.speed_setpoint_pub.publish(speed_msg)
        
        # Log the conversion
        self.get_logger().debug(
            f'Converted - Speed: {msg.linear.x:.2f} m/s, Yaw rate: {msg.angular.z:.2f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Px4CmdVelBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()