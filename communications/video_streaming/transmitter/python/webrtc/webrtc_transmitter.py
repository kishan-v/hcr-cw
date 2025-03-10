#!/usr/bin/env python3
import argparse
import asyncio
import fractions
import logging
import math
import re
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import av
import cv2
from aiortc import (
    MediaStreamTrack,
    RTCConfiguration,
    RTCIceCandidate,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
)
from computer_vision import annotation
from websocket_signaling import WebSocketSignaling

WEBSOCKET_SIGNALLING_URI = "ws://130.162.176.219:8765"
TURN_SERVER_URI = "turn:130.162.176.219:3478"
DEFAULT_VIDEO_SOURCE = "theta"  # "webcam" or "theta". Can also be specified as a command-line argument: e.g. "python3 webrtc_transmitter.py -d webcam"  # noqa: E501

# Dhruv's Oracle Server
# WEBSOCKET_SIGNALLING_URI = "ws://132.145.67.221:8765"
# TURN_SERVER_URI = "turn:132.145.67.221:3478"

MP4_SOURCE = "test_video.mp4"
DEFAULT_VIDEO_SOURCE = "theta"  # "webcam" or "theta" or "mp4". Can also be specified as a command-line argument: e.g. "python3 webrtc_transmitter.py -d webcam"  # noqa: E501


COMP_VIS_MODE = True  # WARNING: Comp. vis. integration is subject to change. It has not been tested properly and may introduce latency.
CV_INTERVAL_SECS = 0.1  # Minimum seconds between running CV processing on a frame.
COMP_VIS_FPS_CAP = 30


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

        if COMP_VIS_MODE:
            self.compVis = annotation(fps_cap=COMP_VIS_FPS_CAP)

    async def recv(self) -> av.VideoFrame:
        pts, time_base = await self.next_timestamp()

        # Sleep to match frame rate
        # await asyncio.sleep(1 / 30)

        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to read frame from webcam")

        if COMP_VIS_MODE:
            self.compVis.last_frame = frame.copy()
            try:
                frame = self.compVis.annotate(frame)
            except:
                pass

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
        if hasattr(self, "cap"):
            self.cap.release()
        if hasattr(self, "window_name"):
            cv2.destroyWindow(self.window_name)


async def run(
    pc_configuration: RTCConfiguration,
    signaling: WebSocketSignaling,
    input_device: str = DEFAULT_VIDEO_SOURCE,
) -> None:
    pc: RTCPeerConnection = RTCPeerConnection(configuration=pc_configuration)
    await signaling.connect()

    local_video = None
    capture = None

    async def restart_connection():
        nonlocal pc, local_video
        logging.info("Restarting WebRTC connection...")

        # Clean up existing connection
        await pc.close()

        # Create a new peer connection with the same configuration
        pc = RTCPeerConnection(configuration=pc_configuration)

        # Set up event handlers for the new connection
        setup_event_handlers()

        # Re-add the video track to the new connection
        if local_video:
            pc.addTrack(local_video)

        # Create and send a new offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await signaling.send(pc.localDescription)

        logging.info("Connection restarted - sent new SDP offer to signaling server")

    def setup_event_handlers():
        @pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange() -> None:
            logging.info(f"ICE connection state changed to: {pc.iceConnectionState}")
            if pc.iceConnectionState == "failed":
                logging.error("ICE connection failed")
                await pc.close()
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

    # Set up initial event handlers
    setup_event_handlers()

    try:
        # Initialize video capture as in original code
        if input_device == "webcam":
            input(
                "Are you sure you want to stream from the webcam and not RICOH Theta? Press Enter to continue..."
            )
            capture = cv2.VideoCapture(index=0)
            print("Opening webcam")
        elif input_device == "theta":
            if not re.search(r"GStreamer:\s*YES", cv2.getBuildInformation()):
                raise RuntimeError(
                    "GStreamer support is not enabled in this OpenCV build."
                )
            else:
                logging.debug("GStreamer support found in OpenCV build.")
            # Verified pipeline: ~ 300 ms latency (to Python receiver on LAN)
            gst_pipeline = (
                "thetauvcsrc mode=2K ! "
                "h264parse ! "
                "nvv4l2decoder ! "
                "nvvidconv ! "
                "video/x-raw, format=BGRx ! "
                "videoconvert ! "
                "appsink sync=false drop=true max-buffers=1"
            )
            # Work-in-progress, aggressive pipeline (further minimise latency at cost of reliability and quality)
            # gst_pipeline = (
            #     "thetauvcsrc mode=2K ! "
            #     "h264parse ! "
            #     "nvv4l2decoder disable-dpb=true skip-frames=1 ! "  # Disable DPB and allow frame skipping
            #     "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "  # Minimize buffering
            #     "nvvidconv interpolation-method=nearest ! "  # Fastest conversion method
            #     "video/x-raw, format=BGRx ! "
            #     "videoconvert n-threads=2 ! "  # Use multiple threads for conversion
            #     "video/x-raw, format=BGR ! "  # OpenCV expects BGR format. It is possible that OpenCV was handling the BGR conversion downstream in the pipeline above, so better to handle in GStreamer.
            #     "appsink sync=false drop=true max-buffers=1 wait-on-eos=false"
            # )
            capture = cv2.VideoCapture(
                filename=gst_pipeline, apiPreference=cv2.CAP_GSTREAMER
            )
            logging.debug(f"Opening GStreamer with pipeline:\n{gst_pipeline}")
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

        # Initial offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await signaling.send(pc.localDescription)

        logging.info("Sent SDP offer to signaling server")

        # Main message loop
        while True:
            try:
                msg = await signaling.receive()

                # Handle restart message
                if isinstance(msg, dict) and msg.get("type") == "restart":
                    logging.info(f"Received restart request: {msg}")
                    # Only restart if the request comes from a receiver
                    if msg.get("clientType") == "receiver":
                        await restart_connection()
                        continue

                # Handle SDP messages
                if isinstance(msg, RTCSessionDescription):
                    logging.info(f"Received SDP {msg.type}")
                    await pc.setRemoteDescription(msg)

                # Handle ICE candidates
                elif isinstance(msg, dict) and msg.get("type") == "candidate":
                    logging.info(f"Received ICE candidate: {msg}")
                    candidate = RTCIceCandidate(
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
                    logging.warning(
                        f"Received unexpected message: {msg} of type: {type(msg)}"
                    )
            except Exception as e:
                logging.exception(f"Error during connection: {e}")
                break

    except Exception as e:
        logging.exception(f"Error: {e}")
    finally:
        cv2.destroyAllWindows()
        if capture:
            capture.release()
        await pc.close()
        await signaling.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebRTC Transmitter")
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose logging"
    )
    parser.add_argument(
        "--device",
        "-d",
        type=str,
        default=DEFAULT_VIDEO_SOURCE,
        help=f"Video source device. Default: {DEFAULT_VIDEO_SOURCE}. Options: 'webcam', 'theta'.",
    )
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

    # pc = RTCPeerConnection(configuration=configuration)
    signaling = WebSocketSignaling(uri=WEBSOCKET_SIGNALLING_URI)

    try:
        asyncio.get_event_loop().run_until_complete(
            run(
                pc_configuration=configuration,
                signaling=signaling,
                input_device=args.device,
            )
        )
    except KeyboardInterrupt:
        pass
