# Unitree Go2

## Sim environment
### Build the docker container:
1. On your local computer, give acccess to the X11 display: `xhost +local:root`
2. Build the docker container: `docker build -t unitree_go2 .`
3. Start the container: ```
   docker run -it --rm --net=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1 --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --device /dev/dri --group-add video unitree_go2```

### Start the simulation:
Run `roslaunch unitree_gazebo normal.launch rname:=go2 wname:=earth`

### Example movement script:
This will make the robot turn in a circle `rosrun unitree_controller unitree_move_kinetic`
