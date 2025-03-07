import socket
import struct
import time

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
        # Example data to send
        int_value = 42
        float_value = 3.14159
        bool_value = True
        char_value = 'A'
        short_array = [1, 2, 3, 4, 5]
        text = "Hello from Python Docker container!"
        
        # Pack the data
        # Format string explanation:
        # i - 4-byte int
        # f - 4-byte float
        # ? - 1-byte boolean
        # c - 1-byte char
        # 5h - 5 short integers (2-bytes each)
        # H - unsigned short for string length
        # {}s - string of specified length
        
        # First pack the fixed-length data
        fixed_data = struct.pack('if?c5h', 
                                int_value, 
                                float_value, 
                                bool_value, 
                                char_value.encode('ascii'),
                                *short_array)
        
        # Then handle the variable-length string
        text_bytes = text.encode('utf-8')
        text_len = len(text_bytes)
        string_data = struct.pack(f'H{text_len}s', text_len, text_bytes)
        
        # Combine both parts
        full_data = fixed_data + string_data
        
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
