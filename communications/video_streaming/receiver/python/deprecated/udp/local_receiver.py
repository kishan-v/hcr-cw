import cv2
import sys

# NOTE: alternative is just run ffplay udp://0.0.0.0:5000

# Open the UDP stream using ffmpeg backend (no GStreamer needed)
# Here we listen on all interfaces (0.0.0.0) at port 5000.
udp_source = "udp://0.0.0.0:5000"

cap = cv2.VideoCapture(udp_source)
if not cap.isOpened():
    print("Error: Unable to open UDP stream.")
    sys.exit(1)

print("Receiving stream. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Warning: No frame received; exiting.")
        break

    cv2.imshow("Receiver", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
