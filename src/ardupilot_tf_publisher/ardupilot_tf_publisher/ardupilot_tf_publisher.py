import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from geographic_msgs.msg import GeoPointStamped
import tf2_ros

class ArduPilotTfPublisher(Node):
    def __init__(self):
        super().__init__('ardupilot_tf_publisher')

        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Broadcaster for the dynamic odom -> base_link transform
        self.dynamic_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Broadcaster for the static map -> odom transform
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Flag to ensure we only publish the static transform once
        self.static_transform_published = False

        # Subscribe to ArduPilot's local pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'ap/pose/filtered',
            self.pose_callback,
            qos_profile)

        # Subscribe to ArduPilot's global origin
        self.origin_sub = self.create_subscription(
            GeoPointStamped,
            'ap/gps_global_origin/filtered',
            self.origin_callback,
            qos_profile)

    def pose_callback(self, msg):
        # This is for the odom -> base_link transform
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.dynamic_broadcaster.sendTransform(t)

    def origin_callback(self, msg):
        # This is for the static map -> odom transform
        if not self.static_transform_published:
            lat = msg.position.latitude
            lon = msg.position.longitude
            
            self.get_logger().info(
                f"Received GPS global origin. LAT: {lat}, LON: {lon}. "
                "Use these values to configure rviz_satellite."
            )

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

            self.static_broadcaster.sendTransform(t)
            self.static_transform_published = True
            self.get_logger().info("Published static map -> odom transform.")


def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()