# Teleop Communication Setup 

Note: Step 3 can be completed using [bernie_start.sh](../../tmux/bernie_start.sh)
1. Ensure the teleop communication server is running in the cloud (Always running)
2. Start the dog interface to listen to CMD_VEL instructions:
    - Go to top level directory 
    - `cd unitree_go2`
    - run `sudo make run-interface`
3. Start the following scripts on the Jetson - recommended to use tmux:
    - `python3 go2/joystick_controller.py`
    - `python3 go2/virtuix_controller_node.py`
        - Add the following flag when starting to disable the control system: `--non-control` (RECOMMENDED)
    - `python3 go2/websocket_node.py`




