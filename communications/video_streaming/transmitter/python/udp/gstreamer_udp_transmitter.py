import cv2
import sys

# Note: the stream can be received by the following command, assuming receiver is on LAN
# ffplay udp://0.0.0.0:5000
# .. although lots of latency was observed in testing

# Change this to the receiver's IP address on your network
receiver_ip = "10.0.0.157"  # MacBook IP Address

# GStreamer pipeline to capture from the Theta.
capture_pipeline = (
    "thetauvcsrc mode=4K ! decodebin ! autovideoconvert ! "
    "video/x-raw,format=BGRx ! queue ! videoconvert ! "
    "video/x-raw,format=BGR ! queue ! appsink"
)

cap = cv2.VideoCapture(capture_pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Error: Unable to open Theta capture pipeline.")
    sys.exit(1)

# Obtain frame dimensions (or set these manually if known).
frame_width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = 30  # Adjust if needed

# Updated writer pipeline: note the use of h264parse and mpegtsmux.
writer_pipeline = (
    "appsrc ! videoconvert ! x264enc tune=zerolatency ! h264parse ! mpegtsmux ! "
    "udpsink host={} port=5000".format(receiver_ip)
)

out = cv2.VideoWriter(writer_pipeline, cv2.CAP_GSTREAMER, 0, fps, (frame_width, frame_height), True)
if not out.isOpened():
    print("Error: Unable to open video writer pipeline.")
    sys.exit(1)

print("Streaming from Theta to {}:5000. Press 'q' to quit.".format(receiver_ip))
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: No frame received from Theta.")
        break

    out.write(frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
