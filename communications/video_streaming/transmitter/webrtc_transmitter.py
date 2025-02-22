#!/usr/bin/env python3
import asyncio
import cv2
import av
import argparse
import logging
import fractions
from typing import Optional

from aiortc import (
    RTCPeerConnection,
    RTCConfiguration,
    RTCIceServer,
    MediaStreamTrack,
    RTCSessionDescription,
    RTCIceCandidate,
)
from websocket_signaling import WebSocketSignaling


WEBSOCKET_SIGNALLING_URI = "ws://130.162.176.219:8765"
VIDEO_SOURCE = "webcam"  # "webcam" or "theta"
# VIDEO_SOURCE = "theta" 


class VideoCameraTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, video_capture: Optional[cv2.VideoCapture] = None):
        super().__init__()
        if video_capture is not None:
            self.cap = video_capture
        else:
            self.cap = cv2.VideoCapture(index=0)
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


async def run(pc: RTCPeerConnection, signaling: WebSocketSignaling):
    await signaling.connect()

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        logging.info(f"ICE connection state changed to: {pc.iceConnectionState}")
        if pc.iceConnectionState == "failed":
            logging.error("ICE connection failed")
            await pc.close()
            await signaling.close()
        elif pc.iceConnectionState == "disconnected":
            logging.warning("ICE connection disconnected")
        elif pc.iceConnectionState == "connected":
            logging.info("ICE connection established successfully")

    @pc.on("signalingstatechange")
    async def on_signalingstatechange():
        print("Signaling state is", pc.signalingState)

    @pc.on("icecandidate")
    async def on_icecandidate(candidate: RTCIceCandidate):
        # Called when this client gathers a new ICE candidate (from STUN/TURN)
        if candidate is not None:
            logging.debug(f"New transmitter ICE candidate: {candidate}")
            await signaling.send(
                {
                    "type": "candidate",
                    "component": candidate.component,
                    "foundation": candidate.foundation,
                    "ip": candidate.ip,
                    "port": candidate.port,
                    "priority": candidate.priority,
                    "protocol": candidate.protocol,
                    "candidateType": candidate.type,
                    "relatedAddress": candidate.relatedAddress,
                    "relatedPort": candidate.relatedPort,
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex,
                    "tcpType": candidate.tcpType,
                }
            )
        else:
            logging.debug("ICE candidate gathering complete")

    try:
        # USE WEBCAM OR RICOH_THETA (using GStreamer backend on Linux)
        if VIDEO_SOURCE == "webcam":
            capture = cv2.VideoCapture(index=0)
            print("Opening webcam")
        elif VIDEO_SOURCE == "theta":
            gst_pipeline = (
                "thetauvcsrc mode=4K ! queue ! h264parse ! decodebin ! queue ! "  # TODO: use nvec
                "videoconvert ! appsink sync=false"
            )
            # Open the GStreamer pipeline in OpenCV
            capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            print(f"Opening GStreamer with pipeline:\n{gst_pipeline}")
            if not capture.isOpened():
                raise IOError('Cannot open RICOH THETA with the given pipeline.')
        else:
            raise ValueError("Invalid video source. Must be 'webcam' or 'theta'")

        # 1) Add local track
        local_video = VideoCameraTrack(video_capture=capture)
        pc.addTrack(local_video)

        # 2) Create and send offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await signaling.send(pc.localDescription)

        print("Sent SDP offer to signaling server")

        # 3) Wait for the remote answer from Unity
        remote_msg = await signaling.receive()
        if (
            isinstance(remote_msg, RTCSessionDescription)
            and remote_msg.type == "answer"
        ):
            print("Received SDP answer from Unity")
            await pc.setRemoteDescription(remote_msg)
        else:
            print("Did not receive valid SDP answer")

        # 4) Keep the connection alive so video continues streaming
        while True:
            try:
                msg = await signaling.receive()
                if msg.get("type") == "candidate":
                    logging.debug(f"Received ICE candidate: {msg}")
                    # Directly add the candidate dictionary
                    candidate = RTCIceCandidate(
                        # TODO: decompose Unity Candidate format into RTCIceCandidate
                        component=msg["component"],
                        foundation=msg["foundation"],
                        ip=msg["ip"],
                        port=msg["port"],
                        priority=msg["priority"],
                        protocol=msg["protocol"],
                        type=msg["candidateType"],
                        relatedAddress=msg.get("relatedAddress"),
                        relatedPort=msg.get("relatedPort"),
                        sdpMid=msg["sdpMid"],
                        sdpMLineIndex=msg["sdpMLineIndex"],
                        tcpType=msg.get("tcpType"),
                    )
                    await pc.addIceCandidate(candidate)
            except Exception as e:
                logging.exception(f"Error during connection: {e}")
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

    # if args.verbose:
    #     logging.basicConfig(level=logging.DEBUG)
    # else:
    #     logging.basicConfig(level=logging.INFO)
    logging.basicConfig(level=logging.DEBUG)

    ice_servers = [
        RTCIceServer(
            urls=["stun:stun.l.google.com:19302"]
        ),  # TODO: uncomment this line
        # TODO: use a TURN server, uncomment and fill in TURN server details:
        RTCIceServer(
            urls=[
                "turn:130.162.176.219:3478?transport=udp",
                "turn:130.162.176.219:3478?transport=tcp",
            ],
            username="username",
            credential="password",
        ),
    ]
    configuration = RTCConfiguration(iceServers=ice_servers)

    pc = RTCPeerConnection(configuration)
    signaling = WebSocketSignaling(uri=WEBSOCKET_SIGNALLING_URI)  # TODO:

    try:
        asyncio.get_event_loop().run_until_complete(run(pc, signaling))
    except KeyboardInterrupt:
        pass
