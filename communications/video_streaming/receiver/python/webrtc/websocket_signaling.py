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

    async def send(self, message):
        if isinstance(message, RTCSessionDescription):
            # Handle SDP messages
            data = {
                "type": message.type,
                "sdp": message.sdp,
            }
        else:
            # Handle ICE candidate messages
            data = message

        await self.websocket.send(json.dumps(data))

    async def receive(self):
        message = await self.websocket.recv()
        data = json.loads(message)

        msg_type = data.get("type")
        if msg_type == "candidate":
            # Return the raw dict for ICE candidate messages
            return data
        elif msg_type in ["offer", "answer", "pranswer", "rollback"]:
            # Return an RTCSessionDescription for valid SDP types
            return RTCSessionDescription(sdp=data["sdp"], type=msg_type)
        elif msg_type == "restart":
            # Return a restart message
            return {"type": "restart", "clientType": data.get("clientType"), "message": data.get("message")}
        else:
            # Raise an error (or ignore) if we get an unexpected type
            raise ValueError(f"Unexpected message type: {msg_type}")

    async def close(self):
        await self.websocket.close()
