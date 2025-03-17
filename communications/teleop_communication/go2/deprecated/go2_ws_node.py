import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  
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
        # Create a publisher that publishes Twist messages to the /cmd_vel topic.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Start the WebSocket client in a separate thread.
        thread = threading.Thread(target=self.run_websocket_client, daemon=True)
        thread.start()

    def process_message(self, message):
        self.get_logger().info(f"Received from relay: {message}")
        try:
            # Parse the incoming JSON message
            data = json.loads(message)
            sent_time = data.get("msg", {}).get("timestamp", None)
            if sent_time is not None:
                latency = time.time() - sent_time
                self.get_logger().info(f"Latency: {latency:.3f} seconds")
            # Create a Twist message from the JSON command.
            twist = Twist()
            twist.linear.x = data.get("msg", {}).get("linear", {}).get("x", 0.0)
            twist.linear.y = data.get("msg", {}).get("linear", {}).get("y", 0.0)
            twist.linear.z = data.get("msg", {}).get("linear", {}).get("z", 0.0)
            twist.angular.x = data.get("msg", {}).get("angular", {}).get("x", 0.0)
            twist.angular.y = data.get("msg", {}).get("angular", {}).get("y", 0.0)
            twist.angular.z = data.get("msg", {}).get("angular", {}).get("z", 0.0)
        except Exception as e:
            self.get_logger().warn(f"Could not process message: {e}")
            return

        # Publish the Twist command on /cmd_vel
        self.cmd_pub.publish(twist)
        self.get_logger().info("Published Twist command on /cmd_vel")

    def run_websocket_client(self):
        # Reconnection loop for robustness.
        while True:
            try:
                asyncio.run(self.websocket_loop())
            except Exception as e:
                self.get_logger().error(f"Exception in websocket client: {e}")
            self.get_logger().info("Reconnecting in 5 seconds...")
            time.sleep(5)

    async def websocket_loop(self):
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
    # Print out the current Python interpreter details for debugging.
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
