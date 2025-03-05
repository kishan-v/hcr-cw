#!/usr/bin/env python3
import asyncio
import cv2
import av
import argparse
import logging
import fractions
import time
import math
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


from lidar_node import run_lidar_node
import threading

# Kish's Oracle Server
# WEBSOCKET_SIGNALLING_URI = "ws://130.162.176.219:8765"
# TURN_SERVER_URI = "turn:130.162.176.219:3478"

# Dhruv's Oracle Server
WEBSOCKET_SIGNALLING_URI = "ws://132.145.67.221:8765"
TURN_SERVER_URI = "turn:132.145.67.221:3478"
VIDEO_SOURCE = "mp4"  # "webcam" or "theta" or "mp4"

MP4_SOURCE = "test_video.mp4"

COMP_VIS_MODE = False  # WARNING: Comp. vis. integration is subject to change. It has not been tested properly and may introduce latency.
CV_INTERVAL_SECS = 0.1  # Minimum seconds between running CV processing on a frame.


class VideoCameraTrack(MediaStreamTrack):
    kind = "video"

    def __init__(
        self,
        video_capture: Optional[cv2.VideoCapture] = None,
        cv_interval_secs: float = 1.0,
    ):
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
        self.executor = ThreadPoolExecutor(
            max_workers=1
        )  # ThreadPoolExecutor better than ProcessPoolExecutor for GPU acceleration?

        # Configure camera
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # self.cap.set(cv2.CAP_PROP_FPS, 30)  # TODO: enable 60 FPS?

        # self.window_name = "Local Preview"
        # cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    async def recv(self) -> av.VideoFrame:
        pts, time_base = await self.next_timestamp()

        # Sleep to match frame rate
        # await asyncio.sleep(1 / 30)

        ret, frame = self.cap.read()
        if not ret:
            # If reading from an MP4 file, reset to beginning to loop the video
            if VIDEO_SOURCE == "mp4":
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                if not ret:
                    raise Exception("Failed to read frame from MP4 video source")
            else:
                raise Exception("Failed to read frame from video source")

        if COMP_VIS_MODE:
            current_time = time.time()
            if current_time - self.last_cv_time >= self.cv_interval_secs:
                self.last_cv_time = current_time
                loop = asyncio.get_event_loop()
                try:
                    # Offload processing to executor without blocking the main loop.
                    frame = await loop.run_in_executor(
                        self.executor, process_frame, frame
                    )
                except Exception as e:
                    print(f"CV processing error: {e}")
                    # If processing fails, use the original frame.

        # # Display frame using imshow in a non-blocking way
        # cv2.imshow(self.window_name, frame)
        # cv2.waitKey(delay=1)  # Wait 1ms - allows window to update without blocking

        # Optional: resize frame (to reduce bandwidth)
        # frame = cv2.resize(frame, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)  # TODO: add hardware acceleration? Move to computer_vision.py?
        video_frame = av.VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        # video_frame.opaque = {}
        # video_frame.opaque["send_time"] = time.time()

        return video_frame

    async def next_timestamp(self):
        """Generate timestamps for frames"""
        framerate = self.cap.get(cv2.CAP_PROP_FPS)

        # Handle invalid framerate values
        if framerate <= 0 or not math.isfinite(framerate):
            # Default to 30 fps if invalid
            framerate = 30.0

        # Convert to Fraction (handle float values)
        time_base = fractions.Fraction(1, int(framerate))
        pts = int(self.timestamp * framerate)
        self.timestamp += 1
        return pts, time_base

    def __del__(self):
        cv2.destroyWindow(self.window_name)


async def run(pc: RTCPeerConnection, signaling: WebSocketSignaling, disable_video: bool, disable_lidar: bool) -> None:
    await signaling.connect()

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange() -> None:
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
    async def on_signalingstatechange() -> None:
        print("Signaling state is", pc.signalingState)

    @pc.on("icecandidate")
    async def on_icecandidate(candidate: RTCIceCandidate) -> None:
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
        if not disable_video:
            # USE WEBCAM OR RICOH_THETA (using GStreamer backend on Linux)
            if VIDEO_SOURCE == "webcam":
                input(
                    "Are you sure you want to stream from the webcam and not RICOH Theta? Press Enter to continue..."
                )
                capture = cv2.VideoCapture(index=0)
                print("Opening webcam")
            elif VIDEO_SOURCE == "mp4":
                input( 
                    "Are you sure you want to stream from mp4 -> Ensure correct MP4 path is set. Press Enter to continue..."
                )
                if not MP4_SOURCE:
                    raise ValueError("MP4 path not provided.")
                capture = cv2.VideoCapture(MP4_SOURCE)
                print(f"Opening MP4 file from path: {MP4_SOURCE}")
                if not capture.isOpened():
                    raise IOError("Cannot open the specified MP4 file.")
            elif VIDEO_SOURCE == "theta":
                # gst_pipeline = ("thetauvcsrc ! decodebin ! autovideoconvert ! video/x-raw,format=BGRx "
                # "! queue ! videoconvert ! video/x-raw,format=BGR ! queue ! appsink")  # TODO: add hardware acceleration
                # gst_pipeline = ("thetauvcsrc ! queue ! h264parse ! nvdec ! gldownload ! queue "
                # "! videoconvert n-threads=0 ! video/x-raw,format=BGR ! queue ! appsink")
                # gst_pipeline = ("thetauvcsrc mode=4K ! queue ! h264parse ! nvv4l2decoder ! gldownload ! queue "
                # "! videoconvert n-threads=0 ! video/x-raw,format=BGR ! queue ! appsink sync=false drop=true")
                # The following pipeline has been verified, but initial tests produced ~1s latency in 2K, ~5s in 4K
                # gst_pipeline = (
                #     "thetauvcsrc mode=2K ! decodebin ! autovideoconvert ! "
                #     "video/x-raw,format=BGRx ! queue ! videoconvert ! "
                #     "video/x-raw,format=BGR ! queue ! appsink"
                # )
                gst_pipeline = (
                    "thetauvcsrc mode=2K ! "
                    "h264parse ! "
                    "nvv4l2decoder ! "
                    "nvvidconv ! "
                    "video/x-raw, format=BGRx ! "
                    "videoconvert ! "
                    "appsink sync=false drop=true max-buffers=1"
                )
                capture = cv2.VideoCapture(
                    filename=gst_pipeline, apiPreference=cv2.CAP_GSTREAMER
                )
                print(f"Opening GStreamer with pipeline:\n{gst_pipeline}")
                if not capture.isOpened():
                    raise IOError(
                        "Cannot open RICOH THETA with the given pipeline.\n"
                        "Do you have GStreamer backend installed for opencv-python?\n"
                        "Have you tried re-installing libuvc-theta?"
                    )
            else:
                raise ValueError("Invalid video source. Must be 'webcam' or 'theta'")

            # Add local track
            local_video = VideoCameraTrack(
                video_capture=capture, cv_interval_secs=CV_INTERVAL_SECS
            )
            pc.addTrack(local_video)
        else:
            print("Video streaming Disabled")

        if not disable_lidar:

            # Create a data channel for LiDAR data.
            lidar_channel = pc.createDataChannel("lidar", ordered=False, maxRetransmits=0)

            @lidar_channel.on("open")
            def on_lidar_channel_open():
                print("LiDAR data channel is now open.")

            loop = asyncio.get_event_loop()

            # Start the ROS2 LiDAR node in a separate thread by calling the imported function.
            threading.Thread(target=run_lidar_node, args=(lidar_channel, loop), daemon=True).start()
        else:
            print("LiDAR data disabled")

        # Create and send offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await signaling.send(pc.localDescription)

        logging.info("Sent SDP offer to signaling server")

        # Wait for the remote answer from receiver
        remote_msg = await signaling.receive()
        if (
            isinstance(remote_msg, RTCSessionDescription)
            and remote_msg.type == "answer"
        ):
            logging.info("Received SDP answer from receiver")
            await pc.setRemoteDescription(remote_msg)
        else:
            logging.warning("Expected SDP answer from receiver, but received:", remote_msg)

        # Keep the connection alive so video continues streaming
        while True:
            try:
                msg = await signaling.receive()
                # Handle new ICE candidates from receiver post-handshake
                if isinstance(msg, RTCSessionDescription):
                    logging.info(f"Received SDP {msg.type}: {msg}")
                    await pc.setRemoteDescription(msg)
                elif isinstance(msg, dict):
                    if msg.get("type") == "candidate":
                        logging.info(f"Received ICE candidate: {msg}")
                        # Directly add the candidate dictionary
                        candidate = RTCIceCandidate(
                            # TODO: decompose receiver Candidate format into RTCIceCandidate
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
                else:
                    print(f"Received unexpected message: {msg} of type: {type(msg)}")
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
    parser.add_argument("--disable-video", action="store_true", help="Disable video streaming")
    parser.add_argument("--disable-lidar", action="store_true", help="Disable lidar streaming")
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.info("Enable verbose logging with -v or --verbose")
        logging.basicConfig(level=logging.INFO)

    ice_servers = [
        RTCIceServer(
            urls=["stun:stun1.l.google.com:19302"]  # type: ignore
        ),
        RTCIceServer(
            urls=[
                f"{TURN_SERVER_URI}?transport=udp",
                f"{TURN_SERVER_URI}?transport=tcp",
            ],  # type: ignore
            username="username",
            credential="password",
        ),
    ]
    configuration = RTCConfiguration(iceServers=ice_servers)

    pc = RTCPeerConnection(configuration)
    signaling = WebSocketSignaling(uri=WEBSOCKET_SIGNALLING_URI)

    try:
        asyncio.get_event_loop().run_until_complete(run(pc, signaling, args.disable_video, args.disable_lidar))
    except KeyboardInterrupt:
        pass
