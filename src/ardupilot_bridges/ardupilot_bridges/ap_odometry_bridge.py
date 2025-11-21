#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class ArduPilotOdometryBridge(Node):
    """
    Bridges ArduPilot DDS output to ROS2.
    Converts ap/pose/filtered + ap/twist/filtered -> /odometry/filtered (odom frame)
    Also publishes TF transforms for odom->base_footprint and map->base_footprint.
    """
    
    def __init__(self):
        super().__init__('ardupilot_odometry_bridge')

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

        # PoseStamped subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'ap/pose/filtered',
            self.pose_callback,
            qos_profile_ap
        )

        # TwistStamped subscriber
        self.twist_sub = self.create_subscription(
            TwistStamped,
            'ap/twist/filtered',
            self.twist_callback,
            qos_profile_ap
        )

        # Odometry Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odometry/filtered',
            qos_profile_cc
        )

        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Store latest messages
        self.latest_pose = None
        self.latest_twist = None

        # publish static transform
        self.publish_static_transform()

        self.get_logger().info('ardupilot_odometry_bridge node started')
        self.get_logger().info('Subscribed to: /ap/pose/filtered')
        self.get_logger().info('Subscribed to: /ap/twist/filtered')
        self.get_logger().info('Publishing to: /odometry/filtered')
    
    def pose_callback(self, msg):
        self.latest_pose = msg

        if self.latest_twist is not None:
            self.publish_local_odometry()
    
    def twist_callback(self, msg):
        self.latest_twist = msg

        if self.latest_pose is not None:
            self.publish_local_odometry()

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

    def publish_local_odometry(self):
        """Publish local odometry (odom frame) from pose and twist."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.latest_pose.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set pose from ap/pose/filtered
        odom_msg.pose.pose = self.latest_pose.pose
        
        # Set twist from ap/twist/filtered
        odom_msg.twist.twist = self.latest_twist.twist
        
        # Publish odometry message
        self.odom_pub.publish(odom_msg)
        
        # Publish TF transform: odom -> base_footprint
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom_msg.header.stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        
        tf_msg.transform.translation.x = self.latest_pose.pose.position.x
        tf_msg.transform.translation.y = self.latest_pose.pose.position.y
        #tf_msg.transform.translation.z = self.latest_pose.pose.position.z
        tf_msg.transform.translation.z = 0.0
        
        tf_msg.transform.rotation = self.latest_pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(tf_msg)

        self.latest_pose = None
        self.latest_twist = None

def main(args=None):
    rclpy.init(args=args)
    
    node = ArduPilotOdometryBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()