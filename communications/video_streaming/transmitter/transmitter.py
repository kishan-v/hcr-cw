#!/usr/bin/env python3
import asyncio
import json
import argparse


parser = argparse.ArgumentParser(description='Video streaming transmitter')
parser.add_argument('--ip', default="130.162.176.219", help='Server IP address')
parser.add_argument('--port', type=int, default=5000, help='Server port')
args = parser.parse_args()

SERVER_IP = args.ip
SERVER_PORT = int(args.port)
LATENCY = 50

VIDEO_PARAMS = {
    "width": 1280,  # Default webcam resolution
    "height": 720,
    "framerate": 30,
    "pixel_format": "bgr24",
    "latency": LATENCY,
}


def print_stream_info():
    """Print information needed by receiver to properly decode stream."""
    info = {
        "stream_parameters": VIDEO_PARAMS,
        "connection": {"ip": SERVER_IP, "port": SERVER_PORT},
    }
    print("[TRANSMITTER] Stream Configuration:")
    print(json.dumps(info, indent=2))


# For macOS: -f avfoundation -i "0"
# We'll add a drawtext filter to embed a timestamp
# Using ultrafast + zerolatency to minimize buffering.
# Modify TRANSMIT_COMMAND to use parameters
TRANSMIT_COMMAND = [
    "ffmpeg",
    "-f",
    "avfoundation",
    "-framerate",
    str(VIDEO_PARAMS["framerate"]),
    "-i",
    "0",
    "-vf",
    f"scale={VIDEO_PARAMS['width']}:{VIDEO_PARAMS['height']},"
    "drawtext=fontfile=/Library/Fonts/Arial.ttf: "
    "text='%{pts\\:hms}': "
    "x=10: y=30: fontsize=24: fontcolor=white: box=1: boxcolor=black@0.5",
    "-f",
    "mpegts",
    "-c:v",
    "h264",
    "-preset",
    "ultrafast",
    "-tune",
    "zerolatency",
    "-f",
    "mpegts",
    f"srt://{SERVER_IP}:{SERVER_PORT}?mode=caller&latency={LATENCY}&peerlatency={LATENCY}",
]


async def transmit():
    retry_count = 0
    max_retry_delay = 5
    print_stream_info()

    while True:  # Keep trying to connect
        try:
            retry_delay = min(2**retry_count, max_retry_delay)
            print(f"[TRANSMITTER] Attempt {retry_count + 1}, starting FFmpeg...")

            proc = await asyncio.create_subprocess_exec(
                *TRANSMIT_COMMAND,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )

            while True:
                line = await proc.stderr.readline()
                if not line:
                    print("[TRANSMITTER] FFmpeg process ended, reconnecting...")
                    break
                print(f"[FFmpeg TX] {line.decode('utf-8').rstrip()}")

            # Clean up
            try:
                proc.terminate()
                await proc.wait()
            except:
                pass

            print(f"[TRANSMITTER] Waiting {retry_delay}s before reconnecting...")
            await asyncio.sleep(retry_delay)
            retry_count += 1

        except Exception as e:
            print(f"[TRANSMITTER] Error: {e}")
            await asyncio.sleep(retry_delay)
            retry_count += 1


def main():
    try:
        asyncio.run(transmit())
    except KeyboardInterrupt:
        print("[TRANSMITTER] Stopping.")


if __name__ == "__main__":
    main()
