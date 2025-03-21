# Bernie
### A 2025 HCR Project

## Instructions
- [Video Streaming](/communications/video_streaming/README.md)
- [Teleoperation Communication](/communications/teleop_communication/README.md)

## Responsibilities
The team was split into three core subteams: Dog, Communication and Virtuix. These are outlined below:

### Dog
Benedict Short, JJ Lu Hilson and Michael Johnston worked together to develop the dog platform and provide a collection of ROS2 topics that the Communications team could continuously integrate with. Data was collected from the dog to provide offline support for continuous integration outside of lab hours. The responsibilites are outlined below:

#### Benedict Short
Developed the containerized Lidar Post-Processing Pipeline on the Dog and in Unity. The containerized post-processing application and setup guide is found under `./unitree_go2/lidar/LidarListener.py`, and the equivalent script in Unity is found under `./virtuix/Assets/Scripts/Lidar/LidarProcessor.cs`.