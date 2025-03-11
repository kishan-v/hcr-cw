import asyncio
import websockets
import json
from websockets.protocol import State

connected_clients = set()

async def handler(websocket, path=None):
    connected_clients.add(websocket)
    print(f"New client connected: {websocket.remote_address}")
    try:
        async for message in websocket:
            print(f"Received message from {websocket.remote_address}: {message}")
            # Only forward the message if there is at least one other client.
            if len(connected_clients) > 1:
                for client in connected_clients:
                    if client != websocket and client.state == State.OPEN:
                        try:
                            await client.send(message)
                        except Exception as e:
                            print(f"Error sending to client {client.remote_address}: {e}")
            else:
                print("Only one client connected; not forwarding message.")
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed for {websocket.remote_address}: {e}")
    except Exception as e:
        print(f"Unexpected error with {websocket.remote_address}: {e}")
    finally:
        connected_clients.remove(websocket)
        print(f"Client disconnected: {websocket.remote_address}")

async def main():
    # Set ping_interval and ping_timeout to help maintain connections.
    server = await websockets.serve(handler, "0.0.0.0", 9090, ping_interval=20, ping_timeout=10)
    print("WebSocket relay server started on port 9090")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())
