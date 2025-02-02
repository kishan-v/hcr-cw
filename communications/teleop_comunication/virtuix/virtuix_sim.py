import json
import time
import websocket
import threading

RELAYER_URL = "ws://132.145.67.221:9090" 

def on_open(ws):
    def run():
        while True:
            command = {
                "op": "command",
                "topic": "teleop/cmd_vel",
                "msg": {
                    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.2},
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
            time.sleep(1)
    thread = threading.Thread(target=run, daemon=True)
    thread.start()

def on_message(ws, message):
    print("Received reply:", message)

def on_error(ws, error):
    print("Error:", error)

def on_close(ws, close_status_code, close_msg):
    print("Connection closed. Code:", close_status_code, "Message:", close_msg)

def run_client():
    while True:
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
        print("Reconnecting in 5 seconds...")
        time.sleep(5)

if __name__ == "__main__":
    websocket.enableTrace(True)
    run_client()
