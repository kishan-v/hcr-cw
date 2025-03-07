import json
import rclpy
from lidar_packer import pack_lidar
from rclpy.node import Node
from std_msgs.msg import String  # Using String for your fake data example


class LidarNode(Node):
    def __init__(self, data_channel, loop):
        super().__init__("lidar_node")
        self.data_channel = data_channel
        self.loop = loop
        self.subscription = self.create_subscription(
            String,  # Adjust if using a different message type
            "/occupancy_grid",
            self.lidar_callback,
            10,
        )
        self.counter = 0
        self.get_logger().info("LiDAR node initialized.")

    def lidar_callback(self, msg):
        # do NOT use the message data directly (assuming it's already a JSON string)
        # Convert to bitpacked struct
        data = pack_lidar(msg.data)

        self.counter += 1
        if (
            self.counter >= 10
            and self.data_channel
            and self.data_channel.readyState == "open"
        ):
            # Schedule the send on the main event loop
            from asyncio import run_coroutine_threadsafe

            run_coroutine_threadsafe(self._send_data(data), self.loop)
            self.get_logger().info("Scheduled sending LiDAR data over data channel.")
            self.counter = 0
        else:
            self.get_logger().warn("LiDAR data channel not open yet.")

    async def _send_data(self, data):
        # This coroutine will run on the main loop
        self.data_channel.send(data)


def run_lidar_node(data_channel, loop):
    rclpy.init()
    node = LidarNode(data_channel, loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
