# Bernie Tmux Configuration

## Overview
A tmux-based process management system for the Unitree Go2 robot is provided for this project. The [`bernie_start.sh`](/tmux/bernie_start.sh) script creates a tmux session that launches all necessary processes in separate panes, providing an organized environment for running and monitoring the robot's software components.

## Usage
- Run [`./bernie_start.sh`](/tmux/bernie_start.sh) to start all scripts in separate tmux panes
- NOTE: you can set `AUTO_EXIT=1 ./bernie_start.sh` to automatically exit tmux after all scripts have finished running

## Adding Custom Scripts
- Create script in [`tmux/scripts/`](/tmux/scripts) directory
- Run `chmod +x scripts/your_script.sh`
- Add script path to `SCRIPTS` array in [`bernie_start.sh`](/tmux/bernie_start.sh):
```bash
SCRIPTS=(
    ...
    "$REPO_DIR/tmux/scripts/your_script.sh"
)
```