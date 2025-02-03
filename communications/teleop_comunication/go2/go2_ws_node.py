import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets
import json
import threading
import time
import sys
from websockets.protocol import State 

class DogWebsocketClient(Node):
    def __init__(self):
        super().__init__('dog_websocket_client')
        self.publisher_ = self.create_publisher(String, 'dog/cmd', 10)
        thread = threading.Thread(target=self.run_websocket_client, daemon=True)
        thread.start()

    def process_message(self, message):
        self.get_logger().info(f"Received from relay: {message}")
        # Calculate latency
        try:
            data = json.loads(message)
            sent_time = data.get("msg", {}).get("timestamp", None)
            if sent_time is not None:
                latency = time.time() - sent_time
                self.get_logger().info(f"Latency: {latency:.3f} seconds")
        except Exception as e:
            self.get_logger().warn(f"Could not compute latency: {e}")
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)

    def run_websocket_client(self):
        while True:
            try:
                asyncio.run(self.websocket_loop())
            except Exception as e:
                self.get_logger().error(f"Exception in run_websocket_client: {e}")
            self.get_logger().info("Reconnecting in 5 seconds...")
            time.sleep(5)

    async def websocket_loop(self):
        # static ip address of the relay server
        uri = "ws://132.145.67.221:9090" 
        try:
            async with websockets.connect(uri, ping_interval=20, ping_timeout=10) as websocket:
                self.get_logger().info("Connected to relay via WebSocket.")
                while True:
                    message = await websocket.recv()
                    self.process_message(message)
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")

def main(args=None):
    # Which python executable is running this script? (For debugging python package not found errors)
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
