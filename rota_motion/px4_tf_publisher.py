#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry

class PX4TFPublisher(Node):
    def __init__(self):
        """Initialize the node, broadcasters, and subscription."""
        super().__init__('px4_tf_publisher')
        
        # Initialize transform broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish the static transform from map to odom
        self.publish_static_transform()
        
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to vehicle_odometry topic
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile,
        )
        self.get_logger().info('PX4 TF Publisher node started')

    def publish_static_transform(self):
        """Publish a static identity transform from map to odom."""
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'map'
        static_transform.child_frame_id = 'odom'
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Published static transform: map -> odom')

    def odom_callback(self, msg):
        """Callback to process vehicle_odometry messages and publish odom to base_link transform."""
        # Check if the pose frame is NED (POSE_FRAME_NED = 1)
        if msg.pose_frame != 1:
            self.get_logger().warn(f'Unsupported pose_frame: {msg.pose_frame}, expected POSE_FRAME_NED (1)')
            return

        # Create the transform message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        # Set translation from position (x, y, z in NED frame)
        transform.transform.translation.x = float(msg.position[0])
        transform.transform.translation.y = float(msg.position[1])
        transform.transform.translation.z = -float(msg.position[2])
        
        # Set rotation from quaternion (msg.q is [w, x, y, z], ROS expects [x, y, z, w])
        transform.transform.rotation.x = float(msg.q[1])
        transform.transform.rotation.y = float(msg.q[2])
        transform.transform.rotation.z = float(msg.q[3])
        transform.transform.rotation.w = float(msg.q[0])
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    node = PX4TFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()