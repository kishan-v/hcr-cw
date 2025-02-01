#!/usr/bin/env python3
import asyncio
import json
import argparse

# MacBook webcam configuration (using avfoundation on macOS)
MACBOOK_CONFIG = {
    "device": "0",  # default device identifier for the MacBook webcam
    "input_format": "avfoundation",  # input format for the MacBook webcam
    "width": 1280,
    "height": 720,
    "framerate": 30,
    "pixel_format": "bgr24",  # used for any extra processing or logging
    "latency": 50,
    "ffmpeg_codec": "h264",  # using hardware-friendly codec options
    "filter": (
        "scale=1280:720,"
        "drawtext=fontfile=/Library/Fonts/Arial.ttf: "  # macOS font path
        "text='%{pts\\:hms}': "
        "x=10: y=30: fontsize=24: fontcolor=white: box=1: boxcolor=black@0.5"
    ),
    # Additional ffmpeg options if needed can be added here.
}

# Ricoh Theta 360 configuration (using v4l2 on Linux, for example)
THETA_CONFIG = {
    "device": "/dev/video0",  # adjust to your device node for Theta
    "input_format": "mjpeg",
    "width": 1920,  # example resolution; set as needed for Theta
    "height": 960,
    "framerate": 30,
    "latency": 50,
    "ffmpeg_codec": "libx264",
    "filter": None,  # No filter in this example; add if needed.
    "crf": "18",  # Constant Rate Factor option for libx264
}

parser = argparse.ArgumentParser(description="Video streaming transmitter")
parser.add_argument("--ip", default="130.162.176.219", help="Server IP address")
parser.add_argument("--port", type=int, default=5000, help="Server port")
parser.add_argument(
    "--device",
    choices=["macbook", "theta"],
    default="macbook",
    help="Select which device configuration to use.",
)
args = parser.parse_args()

SERVER_IP = args.ip
SERVER_PORT = args.port

# Select the device configuration based on command-line argument.
if args.device == "macbook":
    DEVICE_CONFIG = MACBOOK_CONFIG
elif args.device == "theta":
    DEVICE_CONFIG = THETA_CONFIG
else:
    raise ValueError("Unknown device selected.")


def print_stream_info():
    """Print information needed by the receiver to properly decode the stream."""
    info = {
        "stream_parameters": {
            "width": DEVICE_CONFIG["width"],
            "height": DEVICE_CONFIG["height"],
            "framerate": DEVICE_CONFIG["framerate"],
            "latency": DEVICE_CONFIG["latency"],
        },
        "connection": {"ip": SERVER_IP, "port": SERVER_PORT},
    }
    print("[TRANSMITTER] Stream Configuration:")
    print(json.dumps(info, indent=2))


def build_transmit_command():
    """Builds the ffmpeg command based on the device configuration."""
    # Common SRT URL string for both configurations.
    srt_url = (
        f"srt://{SERVER_IP}:{SERVER_PORT}?mode=caller"
        f"&latency={DEVICE_CONFIG['latency']}&peerlatency={DEVICE_CONFIG['latency']}"
    )

    # Build command differently depending on the input format.
    if DEVICE_CONFIG["input_format"] == "avfoundation":
        # For macOS (MacBook webcam)
        cmd = [
            "ffmpeg",
            "-f",
            DEVICE_CONFIG["input_format"],
            "-framerate",
            str(DEVICE_CONFIG["framerate"]),
            "-i",
            DEVICE_CONFIG["device"],
            "-vf",
            DEVICE_CONFIG["filter"],
            "-f",
            "mpegts",
            "-c:v",
            DEVICE_CONFIG["ffmpeg_codec"],
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            srt_url,
        ]
    elif DEVICE_CONFIG["input_format"] == "mjpeg":
        # For Ricoh Theta 360 (assuming a Linux environment with v4l2)
        cmd = [
            "ffmpeg",
            "-f",
            "v4l2",
            "-input_format",
            DEVICE_CONFIG["input_format"],
            "-video_size",
            f"{DEVICE_CONFIG['width']}x{DEVICE_CONFIG['height']}",
            "-framerate",
            str(DEVICE_CONFIG["framerate"]),
            "-i",
            DEVICE_CONFIG["device"],
            "-c:v",
            DEVICE_CONFIG["ffmpeg_codec"],
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            "-crf",
            DEVICE_CONFIG["crf"],
            "-f",
            "mpegts",
            srt_url,
        ]
    else:
        raise ValueError("Unsupported input format in device configuration.")

    return cmd


# --- Async Transmit Function ---


async def transmit():
    retry_count = 0
    max_retry_delay = 5
    print_stream_info()

    while True:  # Keep trying to connect
        try:
            retry_delay = min(2**retry_count, max_retry_delay)
            print(f"[TRANSMITTER] Attempt {retry_count + 1}, starting FFmpeg...")

            TRANSMIT_COMMAND = build_transmit_command()
            print(f"[TRANSMITTER] Running command: {' '.join(TRANSMIT_COMMAND)}")

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

            # Clean up the process if it hasn't been terminated.
            try:
                proc.terminate()
                await proc.wait()
            except Exception as ex:
                print(f"[TRANSMITTER] Exception during process cleanup: {ex}")

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
