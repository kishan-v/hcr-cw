# Docker File Info

Currently, not working, the container is not able to connect to the host machine's X11 server.


### Build the docker container:
1. On your local computer, give acccess to the X11 display: `xhost +local:root`
2. Build the docker container: `docker build -t unitree_go2 .`
3. Start the container: `docker run -it --rm --net=host  --privileged   --env="DISPLAY" --env="QT_X11_NO_MITSHM=1"   --device /dev/dri --group-add video    unitree_go2 `

Might need to run `source /ros2_ws/install/setup.bash`

### Start the rviz2:
Run `ros2 run rviz2 rviz2`

Expected Error: 
``` qt.qpa.xcb: could not connect to display :0 qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found. ... ```

### Start the Gazebo simulation:
Run `ros2 launch go2_config gazebo.launch.py`

Expected Error Message:
``` [ERROR] [gzclient-7]: process has died [pid 93, exit code -6, cmd 'gzclient']. ```

