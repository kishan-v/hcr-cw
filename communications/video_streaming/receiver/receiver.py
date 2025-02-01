import asyncio
import subprocess
import time

import cv2
import numpy as np

from config import VIDEO_STREAM_PARAMS, SERVER_CONFIG

SERVER_IP = str(SERVER_CONFIG["ip"])
SERVER_PORT = int(SERVER_CONFIG["port"])


class VideoStreamParams:
    def __init__(
        self, width=256, height=256, framerate=30, pixel_format="bgr24", latency=50
    ):
        self.width = width
        self.height = height
        self.framerate = framerate
        self.pixel_format = pixel_format
        self.latency = latency

    @property
    def frame_size(self):
        return self.width * self.height * 3  # Assuming 3 channels (BGR)


async def receive_video_and_display(params: VideoStreamParams):
    print(
        f"[RECEIVER] Initializing receiver with:"
        f"\n - Resolution: {params.width}x{params.height}"
        f"\n - Expected frame size: {params.frame_size} bytes"
        f"\n - Framerate: {params.framerate} fps"
        f"\n - Pixel format: {params.pixel_format}"
    )

    while True:
        try:
            print(f"[RECEIVER] Video dimensions: {params.width}x{params.height}")
            print(f"[RECEIVER] Expected frame size: {params.frame_size} bytes")

            receiver_command = [
                "ffmpeg",
                "-y",
                "-fflags",
                "nobuffer",
                "-flags",
                "low_delay",
                "-strict",
                "experimental",
                "-thread_queue_size",
                "512",
                "-i",
                f"srt://{SERVER_IP}:{SERVER_PORT}?mode=caller&latency={params.latency}&peerlatency={params.latency}",
                "-vf",
                f"scale={params.width}:{params.height}",
                "-pix_fmt",
                params.pixel_format,
                "-f",
                "rawvideo",
                "-probesize",
                "32",
                "-analyzeduration",
                "0",
                "pipe:1",
            ]
            print(f"[RECEIVER] Starting FFmpeg:\n{' '.join(receiver_command)}")
            proc = await asyncio.create_subprocess_exec(
                *receiver_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )

            byte_count = 0
            start_time = time.time()

            while True:
                try:
                    # Read the full frame size in chunks
                    raw_frame = b""
                    remaining = params.frame_size
                    while remaining > 0:
                        chunk = await proc.stdout.read(min(65536, remaining))
                        if not chunk:
                            raise EOFError("Stream ended")
                        raw_frame += chunk
                        remaining -= len(chunk)

                    actual_size = len(raw_frame)
                    if actual_size != params.frame_size:
                        print(
                            f"[RECEIVER] Frame size mismatch! Expected: {params.frame_size}, Got: {actual_size}"
                        )
                        continue

                    byte_count += actual_size
                    elapsed = time.time() - start_time
                    if elapsed >= 1.0:
                        mbps = (byte_count * 8) / (elapsed * 1_000_000)
                        print(f"[RECEIVER] Current Throughput: {mbps:.2f} Mbps")
                        byte_count = 0
                        start_time = time.time()

                    frame_np = np.frombuffer(raw_frame, dtype=np.uint8).reshape(
                        (params.height, params.width, 3)
                    )
                    cv2.imshow("SRT Receiver", frame_np)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        return

                except ValueError as ve:
                    print(f"[RECEIVER] Frame processing error: {ve}")
                    continue
                except Exception as e:
                    print(f"[RECEIVER] Error while processing frame: {e}")
                    break

            proc.terminate()
            await proc.wait()
            print("[RECEIVER] Waiting 5 seconds before reconnecting...")
            await asyncio.sleep(5)

        except Exception as e:
            print(f"[RECEIVER] Connection failed: {e}")
            print("[RECEIVER] Waiting 5 seconds before reconnecting...")
            await asyncio.sleep(5)


def main():
    params = VideoStreamParams(**VIDEO_STREAM_PARAMS)

    try:
        asyncio.run(receive_video_and_display(params))
    except KeyboardInterrupt:
        print("[RECEIVER] Exiting.")


if __name__ == "__main__":
    main()
