import threading
import time
from ultralytics import YOLO
import cv2

yolo_model = YOLO("yolo11n-pose.pt")

def process_frame(frame):
    return yolo_model(frame)

class annotation():
    def __init__(self, fps_cap=30):  # Default FPS cap at 30
        self.last_yolo_results = None 
        self.last_frame = None  
        self.fps_cap = fps_cap
        self.yolo_thread = threading.Thread(target=self.yolo_worker, daemon=True)
        self.yolo_thread.start()

    def yolo_worker(self):
        """Continuously process frames in the background with an FPS cap, reducing GPU load."""
        frame_interval = 1.0 / self.fps_cap  # Time per frame
        
        while True:
            start_time = time.time()  # Start time before processing
            
            if self.last_frame is not None:
                self.last_yolo_results = process_frame(self.last_frame)

            elapsed_time = time.time() - start_time  # Time taken for inference
            sleep_time = max(0, frame_interval - elapsed_time)  # Compute remaining sleep time
            
            time.sleep(sleep_time)  # Block the thread to reduce GPU usage

        
    def gesture_hand_raise(self, keypoints, frame):
        try:
            head = keypoints[0] if len(keypoints) > 0 else None  # Nose (Head)
            right_wrist = keypoints[9] if len(keypoints) > 9 else None  # Right wrist
            left_wrist = keypoints[10] if len(keypoints) > 10 else None  # Left wrist

            hand_raised = False 

            head_x, head_y = map(int, head[:2])  # Extract x, y
            r_wrist_x, r_wrist_y = map(int, right_wrist[:2])  # Extract x, y
            l_wrist_x, l_wrist_y = map(int, left_wrist[:2])  # Extract x, y

            if l_wrist_y != 0 and l_wrist_y < head_y:
                hand_raised = True

            if r_wrist_y != 0 and r_wrist_y < head_y:
                hand_raised = True

            if hand_raised:
                cv2.putText(
                    frame,
                    "Requires Attention",
                    (head_x - 40, head_y - 20),  # Position slightly above the head
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),  # Red color
                    2,
                    cv2.LINE_AA,
                )

            return frame 

        except Exception as e:
            print(f"Error occurred during hand raise: {e}")


    def annotate(self, frame):
        if self.last_yolo_results:
            print(len(self.last_yolo_results))
            for result in self.last_yolo_results:
                for keypoints in result.keypoints.xy:
                    for x, y in keypoints:
                        x, y = int(x), int(y)
                        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)  # Draw keypoints
                    
                    frame = self.gesture_hand_raise(keypoints, frame)

                    
        return frame


                            
