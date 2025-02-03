import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets
import json
import threading
import sys

#TODO: This file needs to be within a ros package 

class DogWebsocketClient(Node):
    def __init__(self):
        super().__init__('dog_websocket_client')
        self.publisher_ = self.create_publisher(String, 'dog/cmd', 10)
        thread = threading.Thread(target=self.run_websocket_client, daemon=True)
        thread.start()

    def process_message(self, message):
        self.get_logger().info(f"Received from relay: {message}")
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)

    def run_websocket_client(self):
        asyncio.run(self.websocket_loop())

    async def websocket_loop(self):
        uri = "ws://132.145.67.221:9090" 
        try:
            async with websockets.connect(uri) as websocket:
                self.get_logger().info("Connected to relay via WebSocket.")
                while True:
                    message = await websocket.recv()
                    self.process_message(message)
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")

def main(args=None):
    print("Python executable:", sys.executable)
    print("sys.path:", sys.path)

    rclpy.init(args=args)
    node = DogWebsocketClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down DogWebsocketClient node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
