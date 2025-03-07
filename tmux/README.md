# Bernie Tmux Configuration

## Overview
A tmux-based process management system for the Unitree Go2 robot is provided for this project. The [`bernie_start.sh`](/tmux/bernie_start.sh) script creates a tmux session that launches all necessary processes in separate panes, providing an organized environment for running and monitoring the robot's software components.

## Usage
### Starting the Tmux Session
- Run [`./bernie_start.sh`](/tmux/bernie_start.sh) to start all scripts in separate tmux panes
  - NOTE: you can set `AUTO_EXIT=1 ./bernie_start.sh` to automatically exit tmux after all scripts have finished running
- If the tmux session is already running, the script will prompt to attach to the existing session, or create a new one (and kill the existing session).
###  Exiting the Tmux Session
- You can kill the tmux session by:
  - Running `./tmux/bernie_start.sh` in any pane and pressing any key when prompted.
  - Or, pressing `Ctrl+b` followed by `:`, then typing `kill-session` and pressing `Enter`
- You can detach from the tmux session without killing it by:
  - Pressing `Ctrl+b` followed by `d`


**Further tmux commands can be found in the [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)**

## Adding Custom Scripts
You can add custom scripts to the tmux session which will execute in their own pane.
- Create your script in the [`tmux/scripts/`](/tmux/scripts) directory.
- Run `chmod +x scripts/your_script.sh` to make the script executable.
- Add the script path to the `SCRIPTS` array in [`bernie_start.sh`](/tmux/bernie_start.sh):
```bash
SCRIPTS=(
    ...
    "$REPO_DIR/tmux/scripts/your_script.sh"
)
