import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, PointStamped, Point
import tf2_geometry_msgs
from px4_msgs.msg import TrajectorySetpoint
import math
import numpy as np
from scipy.spatial.transform import Rotation

class Nav2ToPX4TrajectorySetpoint(Node):
    def __init__(self):
        super().__init__('nav2_to_px4_trajectory_setpoint')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lookahead_point = None
        self.velocity_command = None
        self.subscription_lookahead = self.create_subscription(
            PointStamped,
            '/lookahead_point',
            self.lookahead_cb,
            10
        )
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_cb,
            10
        )
        self.publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )
        self.timer = self.create_timer(0.1, self.process_setpoint)

    def lookahead_cb(self, msg):
        self.lookahead_point = msg

    def velocity_cb(self, msg):
        self.velocity_command = msg

    def process_setpoint(self):
        if (self.lookahead_point is None) or (self.velocity_command is None):
            return
        transformed_point = self.transform_lookahead_point()
        linear_vel_map = self.transform_linear_velocity()
        current_position = self.tf_buffer.transform(PointStamped(), 'odom').point
        dx = transformed_point.x - current_position.x
        dy = transformed_point.y - current_position.y
        desired_yaw = math.atan2(dy, dx)
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.position = [transformed_point.x, transformed_point.y, transformed_point.z]
        setpoint.velocity = [linear_vel_map.x, linear_vel_map.y, linear_vel_map.z]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.yaw = desired_yaw
        setpoint.yawspeed = 0.0
        self.publisher.publish(setpoint)
        self.get_logger().info('Published trajectory setpoint')

    def transform_lookahead_point(self):
        point_transformed = self.tf_buffer.transform(self.lookahead_point, 'odom')
        # add fixed altitude
        point_transformed.point.z = 25.0
        return point_transformed.point

    def transform_linear_velocity(self):
        # convert linear velocity to a point
        point_stamped = PointStamped()
        point_stamped.header.frame_id = 'base_link'
        point_stamped.point.x = self.velocity_command.linear.x
        point_stamped.point.y = self.velocity_command.linear.y
        point_stamped.point.z = self.velocity_command.linear.z
        # transform the point
        vel_transformed = self.tf_buffer.transform(point_stamped, 'odom')
        return vel_transformed



def main(args=None):
    rclpy.init(args=args)
    node = Nav2ToPX4TrajectorySetpoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()