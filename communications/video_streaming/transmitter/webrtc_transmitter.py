#!/usr/bin/env python3
import asyncio
import cv2
import av
import argparse
import logging
import fractions
import time
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
from concurrent.futures import ThreadPoolExecutor
from computer_vision import process_frame


WEBSOCKET_SIGNALLING_URI = "ws://130.162.176.219:8765"
TURN_SERVER_URI = "turn:130.162.176.219:3478"
VIDEO_SOURCE = "webcam"  # "webcam" or "theta"
# VIDEO_SOURCE = "theta"
CV_INTERVAL_SECS = 0.1  # Minimum seconds between running CV processing on a frame.


class VideoCameraTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, video_capture: Optional[cv2.VideoCapture] = None, cv_interval_secs: float = 1.0):
        """
        :param video_capture: cv2.VideoCapture object (or None to open default)
        :param cv_rate: Minimum seconds between running CV processing on a frame.
        """
        super().__init__()
        if video_capture is not None:
            self.cap = video_capture
        else:
            self.cap = cv2.VideoCapture(index=0)
        self.timestamp = 0
        self.cv_interval_secs = cv_interval_secs
        self.last_cv_time = 0
        self.executor = ThreadPoolExecutor(max_workers=1)  # ThreadPoolExecutor better than ProcessPoolExecutor for GPU acceleration?

        # Configure camera
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # self.cap.set(cv2.CAP_PROP_FPS, 30)

        # self.window_name = "Local Preview"
        # cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        # Sleep to match frame rate
        # await asyncio.sleep(1 / 30)

        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to read frame from webcam")

        current_time = time.time()
        if current_time - self.last_cv_time >= self.cv_interval_secs:
            self.last_cv_time = current_time
            loop = asyncio.get_event_loop()
            try:
                # Offload processing to executor without blocking the main loop.
                frame = await loop.run_in_executor(self.executor, process_frame, frame)
            except Exception as e:
                print(f"CV processing error: {e}")
                # If processing fails, use the original frame.

        # # Display frame using imshow in a non-blocking way
        # cv2.imshow(self.window_name, frame)
        # cv2.waitKey(delay=1)  # Wait 1ms - allows window to update without blocking

        # Optional: resize frame (to reduce bandwidth)
        # frame = cv2.resize(frame, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)  # TODO: hardware acceleration? Move to computer_vision.py?
        video_frame = av.VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        video_frame.opaque = {}
        video_frame.opaque["send_time"] = time.time()

        return video_frame

    async def next_timestamp(self):
        """Generate timestamps for frames"""
        framerate = cv2.CAP_PROP_FPS
        time_base = fractions.Fraction(1, framerate)
        pts = int(self.timestamp * framerate)
        self.timestamp += 1
        return pts, time_base

    def __del__(self):
        cv2.destroyWindow(self.window_name)


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
            # gst_pipeline = ("thetauvcsrc ! decodebin ! autovideoconvert ! video/x-raw,format=BGRx "
                            # "! queue ! videoconvert ! video/x-raw,format=BGR ! queue ! appsink")  # TODO: add hardware acceleration
            # gst_pipeline = ("thetauvcsrc ! queue ! h264parse ! nvdec ! gldownload ! queue "
                            # "! videoconvert n-threads=0 ! video/x-raw,format=BGR ! queue ! appsink")
            # Use the Theta capture pipeline with mode=4K for WebRTC streaming
            gst_pipeline = (
                "thetauvcsrc mode=4K ! decodebin ! autovideoconvert ! "
                "video/x-raw,format=BGRx ! queue ! videoconvert ! "
                "video/x-raw,format=BGR ! queue ! appsink"
            )
            capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            print(f"Opening GStreamer with pipeline:\n{gst_pipeline}")
            if not capture.isOpened():
                raise IOError('Cannot open RICOH THETA with the given pipeline.')
            else:
                raise ValueError("Invalid video source. Must be 'webcam' or 'theta'")

        # Add local track
        local_video = VideoCameraTrack(video_capture=capture, cv_interval_secs=CV_INTERVAL_SECS)
        pc.addTrack(local_video)

        # Create and send offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await signaling.send(pc.localDescription)

        print("Sent SDP offer to signaling server")

        # Wait for the remote answer from Unity
        remote_msg = await signaling.receive()
        if (
            isinstance(remote_msg, RTCSessionDescription)
            and remote_msg.type == "answer"
        ):
            print("Received SDP answer from Unity")
            await pc.setRemoteDescription(remote_msg)
        else:
            print("Did not receive valid SDP answer")

        # Keep the connection alive so video continues streaming
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
        cv2.destroyAllWindows()
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
        ),
        RTCIceServer(
            urls=[
                f"{TURN_SERVER_URI}?transport=udp",
                f"{TURN_SERVER_URI}?transport=tcp",
            ],
            username="username",
            credential="password",
        ),
    ]
    configuration = RTCConfiguration(iceServers=ice_servers)

    pc = RTCPeerConnection(configuration)
    signaling = WebSocketSignaling(uri=WEBSOCKET_SIGNALLING_URI)

    try:
        asyncio.get_event_loop().run_until_complete(run(pc, signaling))
    except KeyboardInterrupt:
        pass
