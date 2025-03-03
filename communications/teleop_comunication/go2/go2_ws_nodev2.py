import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  
from std_msgs.msg import Float64
from std_msgs.msg import String 
import asyncio
import websockets
import json
import threading
import time
import sys

class DogWebsocketClient(Node):
    def __init__(self):
        super().__init__('dog_websocket_client')

        # Publisher for linear commands on /cmd_vel (Twist messages)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publisher for rotation commands on /rotate_angle (Float64 messages)
        self.angle_pub = self.create_publisher(Float64, '/rotate_angle', 10)

        # Store reference to the current websocket connection for sending LiDAR data
        self.websocket = None
        self.loop = None 

        # Subscribe to /lidarPost. 
        self.lidar_sub = self.create_subscription(
            String,         
            '/occupancy_grid',
            self.lidar_callback,
            10
        )

        # Start the WebSocket client in a separate thread
        thread = threading.Thread(target=self.run_websocket_client, daemon=True)
        thread.start()

    def lidar_callback(self, msg):
        try:
            # # Parse the incoming JSON from the ROS message
            # incoming_data = json.loads(msg.data)
            
            # # Check that the JSON has the expected keys
            # expected_keys = {"world_dims", "timestamp", "box_vals"}
            # if not expected_keys.issubset(incoming_data.keys()):
            #     self.get_logger().warn("Received LiDAR data does not match expected format.")
            #     return

            
            # # Only send if the websocket is connected
            # if self.websocket is not None:
            #     # Schedule the send operation on the asyncio event loop
            #     asyncio.run_coroutine_threadsafe(
            #         self.websocket.send(json.dumps(incoming_data)),
            #         self.loop
            #     )
                    # Minimal check: verify the expected keys exist as substrings
            if ('"world_dims"' not in msg.data or
                '"timestamp"' not in msg.data or
                '"box_vals"' not in msg.data):
                self.get_logger().warn("Received LiDAR data does not match expected format.")
                return

            # Only send if the websocket is connected.
            if self.websocket is not None:
                # Forward the raw string without additional serialization.
                asyncio.run_coroutine_threadsafe(
                    self.websocket.send(msg.data),
                    self.loop
                )
        except Exception as e:
            self.get_logger().error(f"Error in lidar_callback: {e}")


    def process_message(self, message):
        self.get_logger().info(f"Received from relay: {message}")
        try:
            data = json.loads(message)
            # Expect: data["type"] in { "joystick", "virtuix" }
            
            msg_data = data.get("msg", {})
            linear_x = msg_data.get("linear", {}).get("x", 0.0)
            angular_z = msg_data.get("angular", {}).get("z", 0.0)
            
            input_type = data.get("type", "joystick")  # fallback in case not present

            if input_type == "joystick":
                # JOYSTICK: angular.z is an *angular velocity*, just publish Twist
                twist = Twist()
                twist.linear.x = linear_x
                twist.angular.z = angular_z
                self.cmd_pub.publish(twist)
                self.get_logger().info("Published Twist for joystick control")

            elif input_type == "omni":
                # VIRTUIX: angular.z is *target heading* (absolute angle).
                twist = Twist()
                twist.linear.x = linear_x
                twist.angular.z = 0.0  # No direct turn from here
                self.cmd_pub.publish(twist)

                # Publish the heading to rotation controller
                angle_msg = Float64()
                angle_msg.data = angular_z  
                self.angle_pub.publish(angle_msg)
                self.get_logger().info(f"Published rotate_angle: {angular_z:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error in process_message: {e}")

    def run_websocket_client(self):
        # Reconnection loop for robustness.
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
