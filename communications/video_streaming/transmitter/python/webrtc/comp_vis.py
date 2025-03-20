
import subprocess
import threading
import time
import ast
import cv2

VENV_PYTHON = "./communications/video_streaming/.venv/bin/python"

CHILD_SCRIPT = "./communications/video_streaming/transmitter/python/webrtc/YOLO_pipeline.py"


class annotation:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_yolo_results = None
        self.child_process = None
        self.running = True
        self.start_child_process()  # Start the child process once
        self.output_thread = threading.Thread(target=self.listen_to_child_output, daemon=True)
        self.output_thread.start()  # Start the output listening thread
        self.last_yolo_results = None

    def start_child_process(self):
        """Starts the child process once to keep it open."""
        self.child_process = subprocess.Popen(
            [VENV_PYTHON, CHILD_SCRIPT],
            stdout=subprocess.PIPE,
            stdin=subprocess.PIPE,
            text=True
        )
        print("Child process started...")

    def listen_to_child_output(self):
        """Continuously listen for output from the child process (YOLO results)."""
        while self.running:
            if self.child_process.stdout:
                line = self.child_process.stdout.readline()
                if line:
                    self.process_child_output(line.strip())  # Process each line of output

    def process_child_output(self, line):
        """Process the output from the child process."""
        with self.lock:
            # Assuming line is a list of keypoints as a string
            try:
                keypoints = ast.literal_eval(line) # Convert the output to a list of keypoints
                self.last_yolo_results = keypoints[0]
            except Exception as e:
                print(f"ERROR   Error processing output: {e}")


    def gesture_hand_raise(self, keypoints, frame):
        try:
            head = keypoints[0] if len(keypoints) > 0 else None  # Nose (Head)
            right_wrist = keypoints[9] if len(keypoints) > 9 else None  # Right wrist
            left_wrist = keypoints[10] if len(keypoints) > 10 else None  # Left wrist
            hand_raised = False

            head_x, head_y = map(int, head)  # Extract x, y
            r_wrist_x, r_wrist_y = map(int, right_wrist)  # Extract x, y
            l_wrist_x, l_wrist_y = map(int, left_wrist)  # Extract x, y


            if l_wrist_y != 0 and l_wrist_y < head_y:
                hand_raised = True

            if r_wrist_y != 0 and r_wrist_y < head_y:
                hand_raised = True


            if hand_raised:
                    cv2.putText(
                        frame,
                        "HELP!",
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
        with self.lock:  # Lock to safely access last_yolo_results
            if self.last_yolo_results:
                for person in self.last_yolo_results:
                    for keypoint in person:

                        x, y = int(keypoint[0]), int(keypoint[1])
                        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)  # Draw keypoints
                        
                    if len(person) > 1:
                        frame = self.gesture_hand_raise(person, frame)

            return frame
