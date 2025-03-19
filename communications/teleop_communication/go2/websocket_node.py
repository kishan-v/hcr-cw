import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from lidar_packer import pack_lidar as packer
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

        self.last_keepalive_time = self.get_clock().now()
        self.keepalive_timeout = 2.0
        self.locked = False
        self.keepalive_timer =  self.create_timer(1.0, self.check_keepalive)
        
        self.websocket = None
        self.loop = None

        self.lidar_sub = self.create_subscription(
            String,         
            '/occupancy_grid',
            self.lidar_callback,
            10
        )

        # Start the websocket client in a separate thread.
        thread = threading.Thread(target=self.run_websocket_client, daemon=True)
        thread.start()

    def lidar_callback(self, msg):
        try:
            # Minimal check: verify the expected keys exist as substrings


            if ('"world_dims"' not in msg.data or
                '"timestamp"' not in msg.data or
                '"box_vals"' not in msg.data):
                self.get_logger().warn("Received LiDAR data does not match expected format.")
                return

            # Pack the booleans
            packed_data = packer(msg.data)
            # self.get_logger().info(f"Sending lidar data of size: {len(packed_data)}")

            # Only send if the websocket is connected.
            if self.websocket is not None:
                # Forward the raw string without additional serialization.
                asyncio.run_coroutine_threadsafe(
                    self.websocket.send(packed_data),
                    self.loop
                )
        except Exception as e:
            self.get_logger().error(f"Error in lidar_callback: {e}")

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

            msg_type = data.get("type", "keepalive")

            if msg_type == "keepalive":
                self.last_keepalive_time = self.get_clock().now()
                if self.locked:
                    self.get_logger().info("Keepalive received. Unlocking movement commands.")
                    self.locked = False
                return

            if self.locked == True:
                self.get_logger().warn("System locked, waiting for keep alive...")
                return
            
            msg_data = data.get("msg", {})
            linear_x = msg_data.get("linear", {}).get("x", 0.0)
            angular_z = msg_data.get("angular", {}).get("z", 0.0)
            
            if msg_type == "joystick":
                twist = Twist()
                twist.linear.x = linear_x
                twist.angular.z = angular_z
                self.joystick_pub.publish(twist)
                self.get_logger().info("Published joystick command")
            elif msg_type == "omni":
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

    def check_keepalive(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_keepalive_time).nanoseconds / 1e9
        if elapsed > self.keepalive_timeout:
                self.get_logger().warn("Keepalive timeout. Locking movement commands.")
                self.locked = True
                self.send_stop_command()

    def send_stop_command(self):
        twist = Twist()
        twist.linear.x = twist.linear.y = twist.linear.z = 0.0
        twist.angular.x = twist.angular.y = twist.angular.z = 0.0
        self.joystick_pub.publish(twist)
        self.virtuix_linear_pub.publish(twist)
        self.virtuix_linear_pub.publish(twist)
        self.get_logger().info("Published stop command due to keepalive timeout.")




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
