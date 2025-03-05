
- Create script in `scripts/` directory
- Run `chmod +x scripts/your_script.sh`
- Add script path to `SCRIPTS` array in bernie_start.sh
  - ```bash
    SCRIPTS=(
        ...
        "$REPO_DIR/tmux/scripts/your_script.sh"
    )
    ```
- Run `./bernie_start.sh` to start all scripts in separate tmux panes
- NOTE: you can set `AUTO_EXIT=1 ./bernie_start.sh` to automatically exit tmux after all scripts have finished running