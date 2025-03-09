#!/usr/bin/env bash

# Replace these with your target IP and port:
TARGET_IP="10.0.0.157"
TARGET_PORT="5000"

# Video source: reads from the Theta camera, decodes, re-encodes, and parses H.264
VIDEO_SOURCE="thetauvcsrc mode=4K ! h264parse ! nvv4l2decoder ! nvv4l2h264enc \
    bitrate=10000000 iframeinterval=15 ! h264parse"

# Send over UDP as RTP H.264
STREAM_SINK="rtph264pay config-interval=1 pt=96 ! udpsink host=$TARGET_IP port=$TARGET_PORT"

# Launch the pipeline
gst-launch-1.0 $VIDEO_SOURCE ! $STREAM_SINK