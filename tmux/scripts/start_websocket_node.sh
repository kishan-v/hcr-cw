#!/bin/bash

if [ -z "$REPO_DIR" ]; then
    REPO_DIR=$(git rev-parse --show-toplevel)
fi

pyenv shell system

python3 "$REPO_DIR/communications/teleop_communication/go2/websocket_node.py"