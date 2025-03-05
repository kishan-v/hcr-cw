import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarNode(Node):
    def __init__(self, data_channel):
        super().__init__('lidar_node')
        self.data_channel = data_channel
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10
        )
        self.get_logger().info("LiDAR node initialized.")

    def lidar_callback(self, msg):
        # Convert LiDAR data (for example, the ranges) to JSON.
        data = json.dumps({"ranges": list(msg.ranges)})
        if self.data_channel and self.data_channel.readyState == "open":
            self.data_channel.send(data)
            self.get_logger().info("Sent LiDAR data over data channel.")
        else:
            self.get_logger().warn("LiDAR data channel not open yet.")

def run_lidar_node(data_channel):
    rclpy.init()
    node = LidarNode(data_channel)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
