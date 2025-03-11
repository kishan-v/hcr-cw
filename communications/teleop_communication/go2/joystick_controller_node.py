import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        # Subscribe to joystick commands from the websocket node.
        self.sub = self.create_subscription(Twist, '/joystick_cmd', self.joystick_callback, 10)
        # Publisher for sending commands to the robot.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Joystick Controller node started.")

    def joystick_callback(self, msg):
        # Forward the received Twist command to /cmd_vel.
        self.cmd_pub.publish(msg)
        self.get_logger().info("Joystick command forwarded to /cmd_vel")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down joystick_controller")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
