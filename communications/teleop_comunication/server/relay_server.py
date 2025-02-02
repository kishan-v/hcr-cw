#!/usr/bin/env python3
import asyncio
import websockets
import json


connected_clients = set()

async def handler(websocket, path):

    connected_clients.add(websocket)
    print(f"New client connected: {websocket.remote_address}")
    try:
        async for message in websocket:
            print(f"Received message: {message}")

            for client in connected_clients:
                if client != websocket and client.open:
                    await client.send(message)
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected.")
    finally:
        connected_clients.remove(websocket)

async def main():
    # Listen on all interfaces on port 9090
    server = await websockets.serve(handler, "0.0.0.0", 9090)
    print("WebSocket relay server started on port 9090")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())
