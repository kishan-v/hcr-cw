#!/bin/bash

if [ -z "$REPO_DIR" ]; then
    REPO_DIR=$(git rev-parse --show-toplevel)
fi

pyenv shell system  # Use system Python (3.8.10)
python3 "$REPO_DIR/communications/video_streaming/transmitter/python/webrtc/webrtc_transmitter.py"