import json
import time
import websocket
import threading
import msvcrt # Windows only

RELAYER_URL = "ws://132.145.67.221:9090"
should_quit = False

def on_open(ws):
    def run():
        global should_quit
        print("Keyboard control active:")
        print("  w: forward")
        print("  s: backward")
        print("  a: turn left")
        print("  d: turn right")
        print("  x: stop")
        print("  q: quit")
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                if key == 'q':
                    print("Quitting...")
                    should_quit = True
                    ws.close()
                    break

                linear_x = 0.0
                angular_z = 0.0

                if key == 'w':
                    linear_x = 1.0   # Move forward
                elif key == 's':
                    linear_x = -1.0  # Move backward
                elif key == 'a':
                    angular_z = 1.0  # Turn left
                elif key == 'd':
                    angular_z = -1.0 # Turn right
                elif key == 'x':
                    linear_x = 0.0   # Stop
                    angular_z = 0.0

                command = {
                    "op": "command",
                    "topic": "teleop/cmd_vel",
                    "msg": {
                        "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
                        "timestamp": time.time()
                    }
                }
                message = json.dumps(command)
                try:
                    ws.send(message)
                    print("Sent command:", message)
                except websocket._exceptions.WebSocketConnectionClosedException as e:
                    print("Connection closed while sending. Error:", e)
                    break
                except Exception as e:
                    print("Unexpected error sending message:", e)
                    break
            # time.sleep(0.1) # 
    thread = threading.Thread(target=run, daemon=True)
    thread.start()

def on_message(ws, message):
    print("Received reply:", message)

def on_error(ws, error):
    print("Error:", error)

def on_close(ws, close_status_code, close_msg):
    print("Connection closed. Code:", close_status_code, "Message:", close_msg)

def run_client():
    global should_quit
    while not should_quit:
        try:
            ws = websocket.WebSocketApp(
                RELAYER_URL,
                on_open=on_open,
                on_message=on_message,
                on_error=on_error,
                on_close=on_close
            )
            ws.run_forever(ping_interval=20, ping_timeout=10)
        except Exception as e:
            print("Exception in run_forever:", e)
        if should_quit:
            break
        print("Reconnecting in 5 seconds...")
        time.sleep(5)

if __name__ == "__main__":
    websocket.enableTrace(True)
    run_client()
