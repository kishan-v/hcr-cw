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

        self.stopCount = 0
        self.prevRotation = 0.0 # start at 0 radians

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
            # Parse the incoming JSON message
            data = json.loads(message)
            sent_time = data.get("msg", {}).get("timestamp", None)
            if sent_time is not None:
                latency = time.time() - sent_time
                self.get_logger().info(f"Latency: {latency:.3f} seconds")
            
            # Create a Twist message for linear motion only.
            twist = Twist()
            twist.linear.x = data.get("msg", {}).get("linear", {}).get("x", 0.0)
            twist.linear.y = data.get("msg", {}).get("linear", {}).get("y", 0.0)
            twist.linear.z = data.get("msg", {}).get("linear", {}).get("z", 0.0)
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0



            # Publish the linear Twist command on /cmd_vel
            if twist.linear.x == 0 and twist.linear.y == 0 and twist.linear.z == 0:
                self.stopCount += 1
            else:
                self.stopCount = 0

            if self.stopCount < 3:
                self.cmd_pub.publish(twist)


            # self.get_logger().info("Published Twist command on /cmd_vel for linear motion.")
            
            # Process the angular (rotation) command from the JSON.
            target_angle = data.get("msg", {}).get("angular", {}).get("z", 0.0)
            if self.prevRotation == None or target_angle != self.prevRotation:
                angle_msg = Float64()
                angle_msg.data = target_angle
                self.angle_pub.publish(angle_msg)
                # self.get_logger().info(f"Published rotation command on /rotate_angle: {target_angle:.2f} radians")

            self.prevRotation = target_angle
        except Exception as e:
            self.get_logger().warn(f"Could not process message: {e}")

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
