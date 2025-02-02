import json
import time
import websocket

#TODO: need to add oracle server ip
RELAYER_URL = "ws://<ORACLE_SERVER_IP>:9090"

def on_open(ws):
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
    ws.send(message)
    print("Sent command:", message)

def on_message(ws, message):
    print("Received reply:", message)
    ws.close()

def on_error(ws, error):
    print("Error:", error)

def on_close(ws, close_status_code, close_msg):
    print("Connection closed.")

if __name__ == "__main__":
    websocket.enableTrace(True)
    ws = websocket.WebSocketApp(
        RELAYER_URL,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    ws.run_forever()
