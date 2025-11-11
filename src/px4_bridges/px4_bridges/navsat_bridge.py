#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import SensorGps
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus


class PX4NavsatBridge(Node):
    """
    ROS2 node that converts PX4 SensorGps messages to standard NavSatFix messages.
    """
    
    def __init__(self):
        super().__init__('px4_navsat_bridge')
        
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

        # Create subscriber for PX4 GPS messages
        self.subscription = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos_profile_px4
        )
        
        # Create publisher for NavSatFix messages
        self.publisher = self.create_publisher(
            NavSatFix,
            '/gps/fix',
            qos_profile_ros2
        )
        
        self.get_logger().info('PX4 GPS Bridge node started')
        self.get_logger().info('Subscribing to: /fmu/out/vehicle_gps_position')
        self.get_logger().info('Publishing to: /gps/fix')
    
    def gps_callback(self, px4_msg):
        """
        Convert PX4 SensorGps message to NavSatFix message.
        
        Args:
            px4_msg (SensorGps): Incoming PX4 GPS message
        """
        # Create NavSatFix message
        navsat_msg = NavSatFix()
        
        # Set header
        navsat_msg.header.stamp = self.get_clock().now().to_msg()
        navsat_msg.header.frame_id = 'gps'
        
        # Set status
        navsat_msg.status.status = self.convert_fix_type(px4_msg.fix_type)
        navsat_msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Set position
        navsat_msg.latitude = px4_msg.latitude_deg
        navsat_msg.longitude = px4_msg.longitude_deg
        navsat_msg.altitude = px4_msg.altitude_msl_m
        
        # Set covariance
        # PX4 provides eph (horizontal accuracy) and epv (vertical accuracy) in meters
        # Convert to covariance matrix (diagonal)
        if px4_msg.eph > 0 and px4_msg.epv > 0:
            # Horizontal accuracy squared for lat/lon
            h_var = (px4_msg.eph / 100.0) ** 2  # PX4 eph is in cm
            v_var = (px4_msg.epv / 100.0) ** 2  # PX4 epv is in cm
            
            navsat_msg.position_covariance = [
                h_var, 0.0, 0.0,
                0.0, h_var, 0.0,
                0.0, 0.0, v_var
            ]
            navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        
        # Publish the converted message
        self.publisher.publish(navsat_msg)
    
    def convert_fix_type(self, px4_fix_type):
        """
        Convert PX4 fix type to NavSatStatus fix type.
        
        PX4 fix types:
        0: No fix
        1: Dead reckoning only
        2: 2D fix
        3: 3D fix
        4: GPS + dead reckoning
        5: Time only fix
        
        Args:
            px4_fix_type (int): PX4 fix type
            
        Returns:
            int: NavSatStatus fix type
        """
        if px4_fix_type == 0 or px4_fix_type == 1:
            return NavSatStatus.STATUS_NO_FIX
        elif px4_fix_type == 2:
            return NavSatStatus.STATUS_FIX  # 2D fix
        elif px4_fix_type >= 3:
            return NavSatStatus.STATUS_FIX  # 3D fix or better
        else:
            return NavSatStatus.STATUS_NO_FIX


def main(args=None):
    rclpy.init(args=args)
    
    node = PX4NavsatBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()