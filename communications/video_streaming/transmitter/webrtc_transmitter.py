#!/usr/bin/env python3
import asyncio
import cv2
import av
import argparse
import logging
import fractions

from aiortc import (
    RTCPeerConnection,
    RTCConfiguration,
    RTCIceServer,
    MediaStreamTrack,
    RTCSessionDescription,
)
from websocket_signaling import WebSocketSignaling


class VideoCameraTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        self.timestamp = 0

        # Configure camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        # Sleep to match frame rate
        await asyncio.sleep(1 / 30)

        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to read frame from webcam")

        # Convert frame and set timestamp
        video_frame = av.VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        return video_frame

    async def next_timestamp(self):
        """Generate timestamps for frames"""
        time_base = fractions.Fraction(1, 30)  # 30 fps
        pts = int(self.timestamp * 30)
        self.timestamp += 1
        return pts, time_base


async def run(pc, signaling):
    await signaling.connect()

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        print("ICE connection state is", pc.iceConnectionState)
        if pc.iceConnectionState == "failed":
            await pc.close()
            await signaling.close()

    @pc.on("signalingstatechange")
    async def on_signalingstatechange():
        print("Signaling state is", pc.signalingState)

    try:
        # Add the local video track
        local_video = VideoCameraTrack()
        pc.addTrack(local_video)

        # Wait for remote offer first
        remote_msg = await signaling.receive()

        if isinstance(remote_msg, RTCSessionDescription):
            await pc.setRemoteDescription(remote_msg)

            # Create and send answer
            if remote_msg.type == "offer":
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                await signaling.send(pc.localDescription)

        # Keep connection alive
        while True:
            try:
                msg = await signaling.receive()
                if not msg:
                    break
                if isinstance(msg, RTCSessionDescription):
                    await pc.setRemoteDescription(msg)

            except Exception as e:
                print(f"Error during connection: {e}")
                break

    except Exception as e:
        print(f"Error: {e}")
    finally:
        await pc.close()
        await signaling.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="WebRTC Transmitter (Stream macOS webcam)"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose logging"
    )
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    # --- TUNABLE PARAMETERS: ICE Servers (STUN/TURN) ---
    # STUN servers help discover your public IP (and may enable direct P2P connection).
    # TURN servers can relay media if a direct connection cannot be established.
    ice_servers = [
        RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
        # TODO: use a TURN server, uncomment and fill in TURN server details:
        RTCIceServer(urls=["turn:130.162.176.219:3478"]),
    ]
    configuration = RTCConfiguration(iceServers=ice_servers)

    pc = RTCPeerConnection(configuration)
    signaling = WebSocketSignaling("ws://localhost:8765")  # TODO:

    try:
        asyncio.get_event_loop().run_until_complete(run(pc, signaling))
    except KeyboardInterrupt:
        pass
