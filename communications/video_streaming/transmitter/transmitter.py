#!/usr/bin/env python3
import argparse
import asyncio
import json

# --- Device Configuration Dictionaries ---
# All ffmpeg options for input and output are stored in lists.
# This makes it easy to modify parameters and to construct the ffmpeg command.
from configs.macbook_config import MACBOOK_CONFIG
from configs.ricoh_theta_config import THETA_CONFIG


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

# Select the configuration based on the command-line argument.
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
            "latency": DEVICE_CONFIG["latency"],
        },
        "connection": {"ip": SERVER_IP, "port": SERVER_PORT},
    }
    print("[TRANSMITTER] Stream Configuration:")
    print(json.dumps(info, indent=2))


def build_transmit_command():
    """
    Build the ffmpeg command by concatenating the input and output option lists
    from the device configuration along with the SRT URL.
    """
    srt_url = (
        f"srt://{SERVER_IP}:{SERVER_PORT}?mode=caller"
        f"&latency={DEVICE_CONFIG['latency']}&peerlatency={DEVICE_CONFIG['latency']}"
    )
    # Start with 'ffmpeg', then add the options from the config,
    # and finally the SRT URL as the destination.
    return (
        ["ffmpeg"]
        + DEVICE_CONFIG["input_opts"]
        + DEVICE_CONFIG["output_opts"]
        + [srt_url]
    )


async def transmit():
    retry_count = 0
    max_retry_delay = 5
    print_stream_info()

    while True:  # keep trying to connect/reconnect
        try:
            retry_delay = min(2**retry_count, max_retry_delay)
            cmd = build_transmit_command()
            print(
                f"[TRANSMITTER] Attempt {retry_count + 1}, running command:\n{' '.join(cmd)}"
            )

            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )

            # Read and print ffmpeg's stderr output line by line.
            while True:
                line = await proc.stderr.readline()
                if not line:
                    print("[TRANSMITTER] FFmpeg process ended, reconnecting...")
                    break
                print(f"[FFmpeg TX] {line.decode('utf-8').rstrip()}")

            # Clean up the process
            try:
                proc.terminate()
                await proc.wait()
            except Exception as ex:
                print(f"[TRANSMITTER] Exception during cleanup: {ex}")

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
