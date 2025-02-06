#!/usr/bin/env python3
import asyncio
import cv2
import av
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

    try:
        # Create and send offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        print("\n=== SDP Offer ===")
        print(pc.localDescription.sdp)
        print("=================\n")
        await signaling.send(pc.localDescription)

        # Wait for and process answer
        answer = await signaling.receive()
        if isinstance(answer, RTCSessionDescription):
            await pc.setRemoteDescription(answer)
        else:
            print("Received invalid answer")
            return

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

    # --- TUNABLE PARAMETERS: ICE Servers (STUN/TURN) ---
    ice_servers = [
        RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
        # Uncomment and configure TURN server if needed:
        RTCIceServer(urls=["turn:130.162.176.219:3478"]),
    ]
    configuration = RTCConfiguration(iceServers=ice_servers)

    pc = RTCPeerConnection(configuration)
    signaling = WebSocketSignaling("ws://localhost:8765")  # TODO:

    try:
        asyncio.get_event_loop().run_until_complete(run(pc, signaling))
    except KeyboardInterrupt:
        pass
