import sys
import time
import cv2
from ultralytics import YOLO
import numpy as np
from multiprocessing import shared_memory


MODEL_PATH = "yolo11n-pose.pt"
FPS_CAP = 30  # Set the desired FPS cap here



from multiprocessing import shared_memory


import os

if os.name == 'nt':
    import msvcrt
else:
    import fcntl

shape = (960, 1920, 3)
size = np.prod(shape)

shm = shared_memory.SharedMemory(name="frame_buffer")
frame_array = np.ndarray(shape, dtype=np.uint8, buffer=shm.buf)

# Open the same lock file
lock_file = open("frame_lock", "rb")



def process_frame():
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

    results = model(frame, verbose=False, conf=0.2)  # Run YOLO

    out_lst = []
    for result in results:
        tmp_list = []
        for keypoints in result.keypoints.xy:
            tmp_list.append(keypoints.tolist())
        out_lst.append(tmp_list)
    print(out_lst, flush=True)  # Output keypoints

def yolo_worker():
    """Continuously process frames with FPS cap logic."""
    last_frame_time = time.time()  # Track the time of the last frame
    while True:
            current_time = time.time()
            elapsed_time = current_time - last_frame_time

            # Enforce FPS cap
            if elapsed_time < 1 / FPS_CAP:
                time.sleep(1 / FPS_CAP - elapsed_time)  # Sleep to maintain FPS cap

            process_frame()  # Process the frame
            last_frame_time = time.time()  # Update last frame time

if __name__ == "__main__":
    yolo_worker()  # Start the YOLO worker to continuously process frames
