import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import sys

class VirtuixController(Node):
    def __init__(self, non_control=False):
        super().__init__('virtuix_controller')
        # Subscriptions for linear and heading commands.
        self.linear_sub = self.create_subscription(Twist, '/virtuix_linear', self.linear_callback, 10)
        self.heading_sub = self.create_subscription(Float64, '/virtuix_heading', self.heading_callback, 10)
        # Odometry subscription to get current yaw.
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Publisher for sending combined commands.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Timer for control loop.
        self.timer = self.create_timer(0.1, self.control_loop)

        # State variables.
        self.current_yaw = 0.0
        self.target_heading = None  # Absolute heading from virtuix.
        self.latest_linear = Twist()  # Latest linear command.

        # Control parameters.
        self.k_p = 0.5
        self.max_angular_speed = 0.5
        self.yaw_threshold = 0.05

        # Mode flag: True for non-control (preset angular velocity) variant.
        self.non_control = non_control
        if self.non_control:
            self.preset_angular_speed = 0.3  # Preset angular speed (radians per second).
            self.get_logger().info("Non-control mode enabled.")
        else:
            self.get_logger().info("Proportional control mode enabled.")

    def linear_callback(self, msg):
        # Store the latest linear command.
        self.latest_linear = msg

    def heading_callback(self, msg):
        # Store the target absolute heading.
        self.target_heading = msg.data
        self.get_logger().info(f"Received target heading: {self.target_heading:.2f} rad.")

    def odom_callback(self, msg):
        # Extract the current yaw angle from the odometry message.
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        twist = Twist()
        twist.linear = self.latest_linear.linear  # Use the latest linear command.
        if self.target_heading is not None:
            # Compute the error between target and current yaw.
            error = self.target_heading - self.current_yaw
            error = math.atan2(math.sin(error), math.cos(error))  # Normalize to [-pi, pi]
            if self.non_control:
                # Non-control variant: Command a preset angular velocity in the correct direction.
                if abs(error) > self.yaw_threshold:
                    direction = 1.0 if error > 0 else -1.0
                    twist.angular.z = self.preset_angular_speed * direction
                else:
                    twist.angular.z = 0.0
                    self.get_logger().info("Non-control mode: Rotation complete.")
                    self.target_heading = None
            else:
                # Proportional controller variant.
                if abs(error) > self.yaw_threshold:
                    angular_speed = self.k_p * error
                    angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))
                    twist.angular.z = angular_speed
                else:
                    twist.angular.z = 0.0
                    self.get_logger().info("Proportional control: Rotation complete.")
                    self.target_heading = None
        else:
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Published /cmd_vel: linear {twist.linear.x:.2f}, angular {twist.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    non_control = False
    # Check for the flag to run in non-control mode.
    if '--non-control' in sys.argv:
        non_control = True
    node = VirtuixController(non_control=non_control)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down virtuix_controller.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
