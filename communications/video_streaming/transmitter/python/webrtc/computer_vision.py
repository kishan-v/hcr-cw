import cv2
import time
import logging


def process_frame(frame):
    """
    Perform computer vision/annotation on the frame.
    This function runs in a separate thread.
    """
    start_time = time.time()

    # Example: annotate the frame with a timestamp
    annotated = frame.copy()
    cv2.putText(
        annotated,
        f"{time.time():.2f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    # Calculate processing time in milliseconds
    processing_time = (time.time() - start_time) * 1000
    logging.debug(f"Frame processing time: {processing_time:.2f}ms")

    return annotated
