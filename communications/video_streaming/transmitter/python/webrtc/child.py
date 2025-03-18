import sys
import time
import cv2
from ultralytics import YOLO
import numpy as np
from multiprocessing import shared_memory


MODEL_PATH = "yolo11n-pose.pt"
FPS_CAP = 30  # Set the desired FPS cap here

FRAME_PATH = r".\communications\video_streaming\transmitter\python\webrtc\current_frame"


from multiprocessing import shared_memory


import os

if os.name == 'nt':
    import msvcrt
else:
    import fcntl

shape = (480, 640, 3)
size = np.prod(shape)

shm = shared_memory.SharedMemory(name="frame_buffer")
frame_array = np.ndarray(shape, dtype=np.uint8, buffer=shm.buf)

# Open the same lock file
lock_file = open("frame_lock", "rb")



def process_frame(frame_path):
    """Loads YOLO model, processes frame, and outputs keypoints."""
    model = YOLO(MODEL_PATH)

    if os.name == 'nt':
        msvcrt.locking(lock_file.fileno(), msvcrt.LK_LOCK, 1)
    else:
        fcntl.flock(lock_file, fcntl.LOCK_SH)

    frame = np.array(frame_array)  # Copy shared memory content

    # Unlock after reading
    if os.name == 'nt':
        msvcrt.locking(lock_file.fileno(), msvcrt.LK_UNLCK, 1)
    else:
        fcntl.flock(lock_file, fcntl.LOCK_UN)

    results = model(frame, verbose=False)  # Run YOLO
    for result in results:
        for keypoints in result.keypoints.xy:
            print(keypoints.tolist(), flush=True)  # Output keypoints

def yolo_worker():
    """Continuously process frames with FPS cap logic."""
    last_frame_time = time.time()  # Track the time of the last frame
    frame_path = r".\communications\video_streaming\transmitter\python\webrtc\current_frame\frame.jpg"
    while True:
            current_time = time.time()
            elapsed_time = current_time - last_frame_time

            # Enforce FPS cap
            if elapsed_time < 1 / FPS_CAP:
                time.sleep(1 / FPS_CAP - elapsed_time)  # Sleep to maintain FPS cap

            process_frame(frame_path)  # Process the frame
            last_frame_time = time.time()  # Update last frame time

if __name__ == "__main__":
    yolo_worker()  # Start the YOLO worker to continuously process frames
