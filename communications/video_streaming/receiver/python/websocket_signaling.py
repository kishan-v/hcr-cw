# websocket_signaling.py
import asyncio
import json
import websockets
from aiortc import RTCSessionDescription


class WebSocketSignaling:
    def __init__(self, uri):
        self.uri = uri
        self.websocket = None

    async def connect(self):
        self.websocket = await websockets.connect(self.uri)

    async def send(self, description):
        message = json.dumps(
            {
                "type": description.type,
                "sdp": description.sdp,
            }
        )
        await self.websocket.send(message)

    async def receive(self):
        message = await self.websocket.recv()
        data = json.loads(message)
        return RTCSessionDescription(sdp=data["sdp"], type=data["type"])

    async def close(self):
        await self.websocket.close()
