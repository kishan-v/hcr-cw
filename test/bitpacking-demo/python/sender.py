import socket
import time
import struct
import json
from fake_data import json_message

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

        # Define a bytearray for the bitpacked data
        packed = bytearray()
        payload = json.loads(json_message)
        bools = payload['box_vals']
        # Increment by 8 each time (for bitpacking into bytes)
        for i in range(0, len(bools), 8):
            byte = 0
            chunk = bools[i:i+8]
            for j, b, in enumerate(chunk):
                # OR and shift
                byte |= (1 if b else 0) << j
            # Add the byte to the bytearray. Make sure it's clear this is
            # big-endian!
            packed.append(byte)

        print(f"Packed box_vals has size {len(packed)}")

        dims = payload['world_dims']
        header_data = struct.pack('iiifL', dims['width'], dims['depth'], dims['height'], dims['step_size'], payload['timestamp'])
        # Concatenate the struct-packed worlddims and timestamp to the bitpacked box_vals
        full_data = header_data + packed

        # Send the data
        sock.sendall(full_data)
        print(f"Sent {len(full_data)} bytes of packed data")
        
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
