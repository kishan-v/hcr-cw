# Video Streaming (with WebRTC)

## Quick Start

1. Ensure the WebRTC signalling server and Coturn server are both running in Cloud
2. Ensure a WebRTC receiver is running
   - [Unity Receiver](receiver/unity/scripts/webrtc/WebRTCReceiver.cs)
     - e.g. just ensure the Unity game is running.
   - Or, [Python Receiver](receiver/python/webrtc/webrtc_receiver.py)
3. Ensure the Ricoh Theta 360 camera is connected to the Jetson USB-C (data+charging) port **AND** is in **Live Streaming** mode (not Camera or Video mode)
   - Press the mode button on the side of the camera to cycle through modes
4. Start the [WebRTC transmitter](transmitter/python/webrtc/webrtc_transmitter.py) on the Jetson
   - ```bash
     pyenv shell system  # Use system Python (3.8.10)
     python3 "transmitter/python/webrtc/webrtc_transmitter.py"
     ```
   - *Note: the WebRTC handshake will initiate and can take many seconds (~10) to establish the connection before streaming video.*

## Troubleshooting

### Unable to retrieve video from Ricoh Theta
  - Reinstall patched `libuvc` driver:
    - ```bash
      cd ~/libuvc-theta/build && sudo make install
      ```

### Peers are unable to connect and stream video
   - Double-check the IP addresses and ports in the WebRTC transmitter and receiver scripts (including appropriate fields in Unity editor)
   - Ensure the WebRTC signalling server is running in Cloud. Try restarting the server.
   - Ensure the Coturn server is running in Cloud