import asyncio
from datetime import datetime
import websockets
import json
import signal

connected = set()
client_message_counts = {}

HOST = "0.0.0.0"
PORT = 8765

def get_timestamp():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def truncate_message(message, length=100):
    try:
        parsed = json.loads(message)
        return json.dumps(parsed)[:length] + "..." if len(message) > length else message
    except:
        return message[:length] + "..." if len(message) > length else message


async def handler(websocket):
    connected.add(websocket)
    client_id = id(websocket)
    client_message_counts[client_id] = 0
    print(f"[{get_timestamp()}] New connection attempt from client {client_id}")
    print(
        f"[{get_timestamp()}] Client {client_id} connected. Total clients: {len(connected)}"
    )

    try:
        async for message in websocket:
            client_message_counts[client_id] += 1
            truncated = truncate_message(message)
            print(
                f"[{get_timestamp()}] Client {client_id} sent message #{client_message_counts[client_id]}: {truncated}"
            )

            recipients = 0
            for peer in connected:
                if peer != websocket:
                    await peer.send(message)
                    recipients += 1
            print(
                f"[{get_timestamp()}] Message from client {client_id} forwarded to {recipients} peers"
            )

    except websockets.exceptions.ConnectionClosed as e:
        print(
            f"[{get_timestamp()}] Client {client_id} connection closed: {e.code} {e.reason}"
        )
    except Exception as e:
        print(f"[{get_timestamp()}] Error with client {client_id}: {e}")
    finally:
        connected.remove(websocket)
        total_messages = client_message_counts.pop(client_id, 0)
        print(
            f"[{get_timestamp()}] Client {client_id} disconnected. Sent {total_messages} messages. Total clients: {len(connected)}"
        )


async def shutdown(server):
    print(f"[{get_timestamp()}] Initiating server shutdown...")
    server.close()
    await server.wait_closed()

    for ws in connected:
        client_id = id(ws)
        print(f"[{get_timestamp()}] Closing connection to client {client_id}")
        await ws.close()
    print(f"[{get_timestamp()}] Server shutdown complete")


async def main():
    try:
        stop = asyncio.Future()

        server = await websockets.serve(
            handler,
            HOST,
            PORT,
            reuse_port=True,
        )

        print(f"Signaling server started on ws://{HOST}:{PORT}")

        def signal_handler():
            stop.set_result(None)

        loop = asyncio.get_event_loop()
        loop.add_signal_handler(signal.SIGTERM, signal_handler)
        loop.add_signal_handler(signal.SIGINT, signal_handler)

        await stop
        await shutdown(server)

    except OSError as e:
        print(f"Failed to start server: {e}")
        print("Make sure port 8765 is not in use")
    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    asyncio.run(main())
