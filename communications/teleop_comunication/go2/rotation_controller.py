#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class RotationController(Node):
    def __init__(self):
        super().__init__('rotation_controller')

        # Publisher for sending velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for odometry (to get current yaw)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscriber for rotation commands (target rotation in radians)
        self.angle_sub = self.create_subscription(Float64, '/eie4_rotate_angle', self.angle_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        # Variables to store orientation and command state
        self.current_yaw = 0.0
        self.initial_yaw = None
        self.target_delta = None
        self.target_yaw = None
        self.rotation_active = False
        self.yaw_threshold = 0.05 

        self.get_logger().info("Rotation Controller node started.")

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def angle_callback(self, msg: Float64):
        # When a new rotation command is received, set up the rotation.
        self.target_delta = msg.data  
        self.initial_yaw = self.current_yaw
        self.target_yaw = self.initial_yaw + self.target_delta
        self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))
        self.rotation_active = True
        self.get_logger().info(f"Received target rotation: {self.target_delta:.2f} radians. "
                                 f"Initial yaw: {self.initial_yaw:.2f}, Target yaw: {self.target_yaw:.2f}")

    def timer_callback(self):
        if self.rotation_active:
            # Calculate error between current yaw and target yaw
            error = self.target_yaw - self.current_yaw
            error = math.atan2(math.sin(error), math.cos(error))
            self.get_logger().info(f"Current yaw: {self.current_yaw:.2f}, Target yaw: {self.target_yaw:.2f}, Error: {error:.2f}")

            # Simple proportional controller
            k_p = 0.5
            angular_speed = k_p * error

            # Limit the angular speed if necessary
            max_speed = 0.5  # rad/s
            angular_speed = max(-max_speed, min(max_speed, angular_speed))

            twist = Twist()
            if abs(error) > self.yaw_threshold:
                twist.angular.z = angular_speed
                self.cmd_pub.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.rotation_active = False
                self.get_logger().info("Rotation complete. Stopping.")

def main(args=None):
    rclpy.init(args=args)
    node = RotationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Rotation Controller node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
