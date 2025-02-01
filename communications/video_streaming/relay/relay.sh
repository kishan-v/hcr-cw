#!/bin/bash
# Set default ports if not provided as arguments
SENDER_PORT=${1:-5000}
RECEIVER_PORT=${2:-5001}

# Check if srt-live-transmit (srt-tools) is installed
if ! command -v srt-live-transmit >/dev/null 2>&1; then
    echo "srt-tools is not installed."
    echo "Please install it using: sudo apt install srt-tools"
    exit 1
fi

# Run the srt-live-transmit command with parameterised ports
srt-live-transmit "srt://:${SENDER_PORT}?mode=listener" "srt://:${RECEIVER_PORT}?mode=listener"