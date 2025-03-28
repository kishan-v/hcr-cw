# Bernie
### A 2025 HCR Project

## Instructions
- [Video Streaming](/communications/video_streaming/README.md)
- [Teleoperation Communication](/communications/teleop_communication/README.md)
- [Unity](/virtuix/README.md)
- [Lidar](/unitree_go2/lidar/README.md)

## Responsibilities
The team was split into three core subteams: Dog (formally the Unitree Go2 Edu), Communication and Virtuix. These are outlined below, though collaboration between teams was frequent and some team members worked on more than one component. 

___
### Dog
Benedict Short and JJ Lu Hilson worked together to develop the dog platform and provide a collection of ROS2 topics that the Communications team could continuously integrate with. Data was collected from the dog to provide offline support for continuous integration outside of lab hours. The responsibilites are outlined below:

#### JJ Lu-Hilson
Developed the overall system within `./unitree_go2/` including the main Makefile system, the Dockerfiles including the Unitree dependencies to have control over the dog, and development of the dog_interface ROS package to actually control the dog.

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

#### Cristina Fray
- Implementing live video spherical object into Unity to view 360 degree camera feed in VR.
- Map and instructions for repeatable, reproducible and valid testing of simulated search-and-rescue mission, seen in `./evaluation', where both user and Bernie team instructions are clearly outlined. Testing and evaluation of 8 participants.
-  LiDAR calibration and block shader to optimise visual effects on VR.

#### Johanna Quinn
- Implemented first 360 video in Unity for initial testing with VR headset and streamVR
- Recording reproducible and valid testing of simulated search-and-rescue mission, seen in `./evaluation'
- Collected data and processed it for discussion and evaluation, seen in './test_data'
- Coordinated with staff and test participants for availability and made sure everything was covered by a risk assesment
- Testing and evaluation of 8 participants

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
