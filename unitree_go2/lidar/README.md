## Lidar Setup
Step 0 is to start a tmux session.

1. Enter the directory `hcr-cw/unitree_go`
2. On a fresh install, run the following command: `sudo make build-lidar`. Otherwise, skip to the next step.
3. Run `sudo make run-lidar-debug`
4. Exit the docker file by pressing `ctrl + z` (required due to the way in which the docker container needs to be built to successfully read from the dog's ROS topics)
5. Run `sudo make run-lidar-debug` again.
6. From within the docker container, run `source ~/ros2_ws/install/setup.bash`.
7. Start the program with `python3 LidarListener.py`, (try `python3 LidarListener.py -h` to get all commandline options).
