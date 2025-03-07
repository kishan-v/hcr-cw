#!/usr/bin/env python3
import asyncio
import cv2
import argparse
import logging

from aiortc import (
    RTCPeerConnection,
    RTCConfiguration,
    RTCIceServer,
    RTCSessionDescription,
)
from websocket_signaling import WebSocketSignaling


async def run(pc, signaling):
    # Connect to signaling server
    await signaling.connect()

    # Send restart command to transmitter
    print("Sending restart command to transmitter...")
    restart_message = {
        "type": "restart",
        "clientType": "receiver",
        "message": "Receiver requesting connection restart"
    }
    await signaling.send(restart_message)
    print("Restart command sent")

    # Add video transceiver
    pc.addTransceiver("video", direction="recvonly")

    @pc.on("track")
    def on_track(track):
        print("Received %s track" % track.kind)
        if track.kind == "video":

            async def display_video():
                while True:
                    try:
                        frame = await track.recv()
                        img = frame.to_ndarray(format="bgr24")
                        cv2.imshow("Received Video", img)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break
                    except Exception as e:
                        print("Error receiving frame:", e)
                        break
                cv2.destroyAllWindows()

            asyncio.ensure_future(display_video())

    @pc.on("datachannel")
    def on_datachannel(channel):
        print("Received data channel:", channel.label)
        # Optionally check if this is the LiDAR channel
        if channel.label == "lidar":

            @channel.on("message")
            def on_message(message):
                # Process LiDAR data here. For example, print it or update a UI.
                print("Received LiDAR data:", message)

    try:
        # Wait for and process offer from transmitter
        offer = await signaling.receive()
        if not isinstance(offer, RTCSessionDescription):
            print("Received invalid offer")
            return

        print("\n=== Received SDP Offer ===")
        print(offer.sdp)
        print("========================\n")

        # Set the remote description (offer)
        await pc.setRemoteDescription(offer)

        # Create and send answer
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        print("\n=== Sending SDP Answer ===")
        print(pc.localDescription.sdp)
        print("=========================\n")
        await signaling.send(pc.localDescription)

        # Keep connection alive
        done = asyncio.Future()
        await done

    except Exception as e:
        print(f"Error in WebRTC connection: {e}")
        cv2.destroyAllWindows()
    finally:
        # Cleanup
        await signaling.close()
        await pc.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebRTC Receiver (Display Video)")
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose logging"
    )
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    ice_servers = [
        RTCIceServer(urls=["stun:stun.l.google.com:19302"]),  # type: ignore
        RTCIceServer(
            urls=["turn:130.162.176.219:3478"],  # type: ignore
            username="username",
            credential="password",
        ),
    ]
    configuration = RTCConfiguration(iceServers=ice_servers)

    pc = RTCPeerConnection(configuration)
    # signaling = WebSocketSignaling("ws://130.162.176.219:8765")  # TODO:
    signaling = WebSocketSignaling("ws://132.145.67.221:8765")  # TODO:

    try:
        asyncio.get_event_loop().run_until_complete(run(pc, signaling))
    except KeyboardInterrupt:
        pass
