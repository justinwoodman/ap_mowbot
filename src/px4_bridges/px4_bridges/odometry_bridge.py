#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, TransformStamped
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class PX4OdometryBridge(Node):
    """
    Bridge node to convert PX4 VehicleOdometry messages to ROS2 nav_msgs/Odometry
    """
    
    def __init__(self):
        super().__init__('px4_odometry_bridge')
        
        # Declare parameters
        self.declare_parameter('px4_odom_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('nav_odom_topic', '/odometry/filtered')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        
        # Get parameters
        px4_topic = self.get_parameter('px4_odom_topic').value
        nav_topic = self.get_parameter('nav_odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        
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
        
        # Create subscriber for PX4 odometry
        self.px4_sub = self.create_subscription(
            VehicleOdometry,
            px4_topic,
            self.px4_odom_callback,
            qos_profile_px4
        )
        
        # Create publisher for nav_msgs/Odometry
        self.nav_pub = self.create_publisher(
            Odometry,
            nav_topic,
            qos_profile_ros2
        )

        # Transform Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        self.get_logger().info(f'PX4 Odometry Bridge initialized')
        self.get_logger().info(f'Subscribing to: {px4_topic}')
        self.get_logger().info(f'Publishing to: {nav_topic}')
    
    def publish_static_transform(self):
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            
            # This is an identity transform because we are aligning map and odom
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.static_tf_broadcaster.sendTransform(t)

    def px4_odom_callback(self, px4_msg: VehicleOdometry):
        """
        Convert PX4 VehicleOdometry to nav_msgs/Odometry
        
        PX4 VehicleOdometry uses NED (North-East-Down) frame
        ROS typically uses ENU (East-North-Up) or local frame
        This conversion maintains the PX4 frame convention
        """
        
        # Create nav_msgs/Odometry message
        nav_msg = Odometry()
        
        # Header
        nav_msg.header.stamp = self.get_clock().now().to_msg()
        nav_msg.header.frame_id = self.frame_id
        nav_msg.child_frame_id = self.child_frame_id
        
        # Position (NED)
        nav_msg.pose.pose.position = Point(
            x=float(px4_msg.position[0]),
            y=float(px4_msg.position[1]),
            z=float(px4_msg.position[2])
        )
        
        # Orientation (quaternion)
        nav_msg.pose.pose.orientation = Quaternion(
            w=float(px4_msg.q[0]),
            x=float(px4_msg.q[1]),
            y=float(px4_msg.q[2]),
            z=float(px4_msg.q[3])
        )
        
        # Linear velocity (body frame)
        nav_msg.twist.twist.linear = Vector3(
            x=float(px4_msg.velocity[0]),
            y=float(px4_msg.velocity[1]),
            z=float(px4_msg.velocity[2])
        )
        
        # Angular velocity (body frame)
        nav_msg.twist.twist.angular = Vector3(
            x=float(px4_msg.angular_velocity[0]),
            y=float(px4_msg.angular_velocity[1]),
            z=float(px4_msg.angular_velocity[2])
        )
        
        # Position covariance (if available in PX4 message)
        # PX4 provides position and orientation variance
        nav_msg.pose.covariance = [0.0] * 36
        if hasattr(px4_msg, 'position_variance'):
            nav_msg.pose.covariance[0] = float(px4_msg.position_variance[0])  # x
            nav_msg.pose.covariance[7] = float(px4_msg.position_variance[1])  # y
            nav_msg.pose.covariance[14] = float(px4_msg.position_variance[2]) # z
        
        if hasattr(px4_msg, 'orientation_variance'):
            nav_msg.pose.covariance[21] = float(px4_msg.orientation_variance[0]) # roll
            nav_msg.pose.covariance[28] = float(px4_msg.orientation_variance[1]) # pitch
            nav_msg.pose.covariance[35] = float(px4_msg.orientation_variance[2]) # yaw
        
        # Velocity covariance (if available)
        nav_msg.twist.covariance = [0.0] * 36
        if hasattr(px4_msg, 'velocity_variance'):
            nav_msg.twist.covariance[0] = float(px4_msg.velocity_variance[0])  # vx
            nav_msg.twist.covariance[7] = float(px4_msg.velocity_variance[1])  # vy
            nav_msg.twist.covariance[14] = float(px4_msg.velocity_variance[2]) # vz
        
        # Publish converted message
        self.nav_pub.publish(nav_msg)

        # Publish TF transform: odom -> base_footprint
        tf_msg = TransformStamped()
        tf_msg.header.stamp = nav_msg.header.stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        
        tf_msg.transform.translation.x = nav_msg.pose.pose.position.x
        tf_msg.transform.translation.y = nav_msg.pose.pose.position.y
        #tf_msg.transform.translation.z = nav_msg.pose.pose.position.z
        tf_msg.transform.translation.z = 0.0
        
        tf_msg.transform.rotation = nav_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = PX4OdometryBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()