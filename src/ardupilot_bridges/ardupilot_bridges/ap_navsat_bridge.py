#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import NavSatFix


class ArduPilotNavsatBridge(Node):
    """
    Bridges ArduPilot DDS output to ROS2.
    """
    
    def __init__(self):
        super().__init__('ardupilot_navsat_bridge')
    
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

        self.navsat_sub = self.create_subscription(
            NavSatFix,
            'ap/navsat',
            self.navsat_callback,
            qos_profile_ap
        )

        self.navsat_pub = self.create_publisher(
            NavSatFix,
            'gps/fix',
            qos_profile_cc
        )

        self.get_logger().info('ardupilot_navsat_bridge node started')
        self.get_logger().info('Subscribed to: /ap/navsat')
        self.get_logger().info('Publishing to: /gps/fix')

    def navsat_callback(self, msg):
        msg.header.frame_id = 'gps'
        self.navsat_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = ArduPilotNavsatBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()