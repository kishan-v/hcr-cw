# Bernie
### A 2025 HCR Project

## Instructions
- [Video Streaming](/communications/video_streaming/README.md)
- [Teleoperation Communication](/communications/teleop_communication/README.md)

## Responsibilities
The team was split into three core subteams: Dog (formally the Unitree Go2 Edu), Communication and Virtuix. These are outlined below, though collaboration between teams was frequent and some team members worked on more than one component. 

___
### Dog
Benedict Short and JJ Lu Hilson worked together to develop the dog platform and provide a collection of ROS2 topics that the Communications team could continuously integrate with. Data was collected from the dog to provide offline support for continuous integration outside of lab hours. The responsibilites are outlined below:

#### Benedict Short
Developed the containerized Lidar Post-Processing Pipeline on the Dog and in Unity. The containerized post-processing application and setup guide are found under `./unitree_go2/lidar/LidarListener.py`, and the equivalent script in Unity is found under `./virtuix/Assets/Scripts/Lidar/LidarProcessor.cs`.

#### Michael Johnston
- Computer vision `./communications/video_streaming/transmitter/python/webrtc/computer_vision.py`
  - Integration with video streaming pipeline
- CAD files `cad/*`
  - Designed RICOH Theta Z1 mount for Unitree Go2
___

### Virtuix
Advik Chitre, Nik Lewis, Johanna Quinn and Cristina Fray worked primarily on Virtuix side, working to obtain reliable data to control the quadruped robot with. 

#### Advik Chitre
- Virtuix mapping in `./virtuix/Assets/Scripts/Control/TeleopVirtuixCommunication.cs`
- movement processing for smooth walking in `./virtuix/Assets/Scripts/Control/TeleopVirtuixCommunication.cs`
- Joystick mapping with SteamVR & `./virtuix/Assets/Scripts/Control/TeleopJoystickCommunication.cs`
- Alignment between LiDAR, video, and operator Unity in teleop scripts & `./virtuix/Assets/Scripts/Control/JoystickTurn.cs`
- Recording virtuix data and simulating processing in `./virtuix/Assets/Scripts/Control/VirtuixRecord.cs` & `./virtuix/Simulation`

#### Nik Lewis
Worked on the initial data extraction from the Virtuix Omni, seen in `./virtuix/Assets/Scripts/Control/TeleopVirtuixCommunication.cs`. Also added LiDAR bitpacking to `LidarListener.py` and `LidarProcessor.cs`. 

---

### Communications

#### Kishan Vijayarajah
- Video streaming pipeline in
  - Attempt 1 (SRT protocol) `./communications/video_streaming/*`
  - Attempt 2 (WebRTC protocol) `./communications/video_streaming/*`
    - WebRTC transmitter (Python) and receiver (Python and Unity)
    - WebRTC signalling server and signalling protocol
  - Integration with Unitree Go2
    - Compiling and installing linux drivers for Ricoh Theta
    - Building GStreamer pipeline to capture and encode video stream
    - Debugging OpenCV and GStreamer support
  - Integration with Unity `./virtuix/Assets/Scripts/webrtc/*`
    - Creating WebRTC receiver in Unity, displaying 360 video in Sphere (using custom shader)

#### Dhruv Devgan Sharma
- Websocket communication between Unity and GO2 for movement commands and LiDAR data `./communications/teleop_communication/`.
- GO2 movement controllers `./communications/teleop_communication/go2/virtuix_controller_node.py`. and `./communications/teleop_communication/go2/joystick_controller.py`.
- ROS2 Go2 simulation setup as seen in `./communications/teleop_communication/simulation`
- Attempt to switch LiDAR data to webRTC on `merge-communications` branch.
