#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, TransformStamped
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import math

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
        Convert PX4 VehicleOdometry (NED) to nav_msgs/Odometry (ENU)
        """
        nav_msg = Odometry()

        # Header
        nav_msg.header.stamp = self.get_clock().now().to_msg()
        nav_msg.header.frame_id = self.frame_id
        nav_msg.child_frame_id = self.child_frame_id

        # Position: NED -> ENU
        px, py, pz = _ned_to_enu_vec(px4_msg.position)
        nav_msg.pose.pose.position = Point(x=px, y=py, z=pz)

        # Orientation: convert quaternion by pre-multiplying with q_frame (NED->ENU)
        # q_frame corresponds to rotation R = [[0,1,0],[1,0,0],[0,0,-1]]
        s = 1.0 / math.sqrt(2.0)
        q_frame = (0.0, s, s, 0.0)  # (w,x,y,z)
        q_px4 = (float(px4_msg.q[0]), float(px4_msg.q[1]), float(px4_msg.q[2]), float(px4_msg.q[3]))
        q_enu = _quat_mul(q_frame, q_px4)
        nav_msg.pose.pose.orientation = Quaternion(w=q_enu[0], x=q_enu[1], y=q_enu[2], z=q_enu[3])

        # Linear velocity: convert same axis mapping
        lvx, lvy, lvz = _ned_to_enu_vec(px4_msg.velocity)
        nav_msg.twist.twist.linear = Vector3(x=lvx, y=lvy, z=lvz)

        # Angular velocity (body frame): apply same axis permutation
        avx, avy, avz = _ned_to_enu_vec(px4_msg.angular_velocity)
        nav_msg.twist.twist.angular = Vector3(x=avx, y=avy, z=avz)

        # Pose covariance: basic diagonal remap if variances available
        nav_msg.pose.covariance = [0.0] * 36
        if hasattr(px4_msg, 'position_variance'):
            # assume position_variance = [var_x_ned, var_y_ned, var_z_ned]
            nav_msg.pose.covariance[0] = float(px4_msg.position_variance[1])  # var_x_enu = var_y_ned
            nav_msg.pose.covariance[7] = float(px4_msg.position_variance[0])  # var_y_enu = var_x_ned
            nav_msg.pose.covariance[14] = float(px4_msg.position_variance[2]) # var_z_enu = var_z_ned

        if hasattr(px4_msg, 'orientation_variance'):
            # orientation variance mapping is application specific; here we copy as-is into roll/pitch/yaw slots.
            nav_msg.pose.covariance[21] = float(px4_msg.orientation_variance[0])
            nav_msg.pose.covariance[28] = float(px4_msg.orientation_variance[1])
            nav_msg.pose.covariance[35] = float(px4_msg.orientation_variance[2])

        # Twist covariance: remap diagonals similarly
        nav_msg.twist.covariance = [0.0] * 36
        if hasattr(px4_msg, 'velocity_variance'):
            nav_msg.twist.covariance[0] = float(px4_msg.velocity_variance[1])
            nav_msg.twist.covariance[7] = float(px4_msg.velocity_variance[0])
            nav_msg.twist.covariance[14] = float(px4_msg.velocity_variance[2])

        # Publish converted message
        self.nav_pub.publish(nav_msg)

        # Publish TF transform: odom -> base_link (use converted pose)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = nav_msg.header.stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'

        tf_msg.transform.translation.x = nav_msg.pose.pose.position.x
        tf_msg.transform.translation.y = nav_msg.pose.pose.position.y
        tf_msg.transform.translation.z = nav_msg.pose.pose.position.z  # keep actual z if you want; previous code forced 0.0
        tf_msg.transform.rotation = nav_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)

def _quat_mul(a, b):
    # a, b are (w,x,y,z)
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

def _ned_to_enu_vec(v):
    # v is indexable [x_n, y_n, z_n]
    return (float(v[1]), float(v[0]), -float(v[2]))

def _reorder_position_cov(cov):
    # cov is 36-length list; mapping only for diagonal position terms
    new = [0.0]*36
    # position indices in row-major 6x6 matrix: (0,0)->0, (1,1)->7, (2,2)->14
    # map x_enu <- y_ned, y_enu <- x_ned, z_enu <- z_ned (sign flip doesn't change variance)
    new[0] = cov[7]   # var_x_enu = var_y_ned
    new[7] = cov[0]   # var_y_enu = var_x_ned
    new[14] = cov[14] # var_z_enu = var_z_ned
    return new

def _reorder_twist_cov(cov):
    # same as position for linear velocities
    new = [0.0]*36
    new[0] = cov[7]
    new[7] = cov[0]
    new[14] = cov[14]
    return new

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