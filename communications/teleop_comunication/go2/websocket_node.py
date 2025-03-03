import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
import asyncio
import websockets
import json
import threading
import time

class WebsocketNode(Node):
    def __init__(self):
        super().__init__('websocket_node')
        # Publishers for joystick and virtuix commands
        self.joystick_pub = self.create_publisher(Twist, '/joystick_cmd', 10)
        self.virtuix_linear_pub = self.create_publisher(Twist, '/virtuix_linear', 10)
        self.virtuix_heading_pub = self.create_publisher(Float64, '/virtuix_heading', 10)
        
        self.websocket = None
        self.loop = None

        # Start the websocket client in a separate thread.
        thread = threading.Thread(target=self.run_websocket_client, daemon=True)
        thread.start()

    def run_websocket_client(self):
        while True:
            try:
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)
                self.loop.run_until_complete(self.websocket_loop())
            except Exception as e:
                self.get_logger().error(f"Exception in websocket client: {e}")
            finally:
                self.loop.close()
            self.get_logger().info("Reconnecting in 5 seconds...")
            time.sleep(5)

    async def websocket_loop(self):
        uri = "ws://132.145.67.221:9090"
        try:
            async with websockets.connect(uri, ping_interval=20, ping_timeout=10) as websocket:
                self.websocket = websocket
                self.get_logger().info("Connected to relay via WebSocket.")
                while True:
                    message = await websocket.recv()
                    self.process_message(message)
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")
        finally:
            self.websocket = None

    def process_message(self, message):
        self.get_logger().info(f"Received from relay: {message}")
        try:
            data = json.loads(message)
            # Expect data["type"] to be either "joystick" or "virtuix"
            msg_data = data.get("msg", {})
            linear_x = msg_data.get("linear", {}).get("x", 0.0)
            angular_z = msg_data.get("angular", {}).get("z", 0.0)
            input_type = data.get("type", "joystick")
            
            if input_type == "joystick":
                twist = Twist()
                twist.linear.x = linear_x
                twist.angular.z = angular_z
                self.joystick_pub.publish(twist)
                self.get_logger().info("Published joystick command")
            elif input_type == "omni":
                # For virtuix commands, treat angular.z as the target absolute heading.
                twist = Twist()
                twist.linear.x = linear_x
                twist.angular.z = 0.0
                self.virtuix_linear_pub.publish(twist)
                
                heading_msg = Float64()
                heading_msg.data = angular_z  # absolute heading in radians
                self.virtuix_heading_pub.publish(heading_msg)
                self.get_logger().info(f"Published virtuix command: linear {linear_x}, heading {angular_z}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WebsocketNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down websocket_node")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()