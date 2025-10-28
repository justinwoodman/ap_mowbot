#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped


class ArduPilotCmdVelBridge(Node):
    """
    Bridges ROS2 navigation stack cmd_vel output to ArduPilot DDS.
    Converts cmd_vel_stamped (RELIABLE) -> ap/cmd_vel (BEST_EFFORT)
    """
    
    def __init__(self):
        super().__init__('ardupilot_cmd_vel_bridge')
    
        # QoS profile for ArduPilot DDS topics (typically best effort)
        qos_profile_ap = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for companion computer ROS2 topics (typically reliable)
        qos_profile_cc = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber for navigation stack cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            'cmd_vel_stamped',
            self.cmd_vel_callback,
            qos_profile_cc
        )

        # Publisher for ArduPilot cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            'ap/cmd_vel',
            qos_profile_ap
        )

    def cmd_vel_callback(self, msg):
        """Forward cmd_vel to ArduPilot with proper frame_id"""
        msg.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = ArduPilotCmdVelBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()