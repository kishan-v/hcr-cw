import socket
import time
import struct
import json
from fake_data import json_message
from lidar_packer import pack_lidar as packer

empty = ["false" for _ in range(4000)]

def send_packed_data():
    # Create a socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('receiver', 8888)  # Using the service name from docker-compose
    print(f"Connecting to {server_address[0]}:{server_address[1]}")
    
    # Add retry logic to handle possible connection timing issues
    max_retries = 5
    for retry in range(max_retries):
        try:
            # Connect to the server
            sock.connect(server_address)
            break
        except socket.error as e:
            if retry < max_retries - 1:
                wait_time = 2 * (retry + 1)
                print(f"Connection failed. Retrying in {wait_time} seconds... ({retry+1}/{max_retries})")
                time.sleep(wait_time)
            else:
                raise e
    
    try:
        # Keep the timestamp and worlddims the same, the box_vals are the main
        # inefficiency
        packed = packer(json_message)

        print(f"Packed box_vals has size {len(packed)}")

        # Send the data
        sock.sendall(packed)
        print(f"Sent {len(packed)} bytes of packed data")
        
        # Wait for confirmation
        response = sock.recv(1024)
        print(f"Received: {response.decode('utf-8')}")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Closing connection")
        sock.close()

if __name__ == "__main__":
    send_packed_data()
