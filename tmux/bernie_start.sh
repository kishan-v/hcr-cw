#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (using sudo)"
  exit 1
fi

# Find repository root directory
REPO_DIR=$(git rev-parse --show-toplevel)
cd "$REPO_DIR" || { echo "Failed to change to repository directory"; exit 1; }
echo "Working from repository: $REPO_DIR"

# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# Add as many scripts as needed, and panes will automatically be created for each
# Remember to run `chmod +x <script_name>.sh` to make the scripts executable
SCRIPTS=(
    "$REPO_DIR/tmux/scripts/start_video_transmitter.sh"
    "$REPO_DIR/tmux/scripts/start_websocket.sh"
    "$REPO_DIR/tmux/scripts/start_virtuix_controller.sh"
    "$REPO_DIR/tmux/scripts/start_joystick_controller.sh"
    "$REPO_DIR/tmux/scripts/script3.sh"
    "$REPO_DIR/tmux/scripts/script4.sh"
)
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------

# Set AUTO_EXIT to 1 to automatically exit panes when scripts complete, 0 to keep bash open
if [ $AUTO_EXIT -eq 1 ]; then
    echo "AUTO_EXIT is set to 1: Panes will automatically exit when scripts complete"
else
    echo "AUTO_EXIT is set to 0: Panes will keep bash open after scripts complete"
    AUTO_EXIT=0
fi

# Get timestamp for logs
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# Create logs directory with timestamp subfolder
LOGS_DIR="$REPO_DIR/tmux/logs"
SESSION_LOG_DIR="$LOGS_DIR/$TIMESTAMP"
mkdir -p "$SESSION_LOG_DIR"
echo "Logs will be stored in: $SESSION_LOG_DIR"

get_command_string() {
    local script=$1
    local log_file=$2
    local scripts_dir="$REPO_DIR/tmux/scripts"
    
    if [ $AUTO_EXIT -eq 1 ]; then
        echo "{ cd \"$scripts_dir\" && $script; } 2>&1 | tee \"$log_file\""
    else
        echo "{ cd \"$scripts_dir\" && $script; } 2>&1 | tee \"$log_file\"; exec bash"
    fi
}

# Check if tmux is installed
command -v tmux >/dev/null 2>&1 || { echo "tmux is not installed. Please install it first."; exit 1; }

# Check if scripts exist and are executable
for script in "${SCRIPTS[@]}"; do
    full_path="$script"
    if [ ! -x "$full_path" ]; then
        echo "Error: Script $full_path doesn't exist or is not executable"
        echo "Run: chmod +x $full_path (if the file exists)"
        exit 1
    fi
done

# Check if we're already inside a tmux session
if [ -n "$TMUX" ]; then
    CURRENT_SESSION=$(tmux display-message -p '#S')
    if [ "$CURRENT_SESSION" = "bernie_control" ]; then
        echo "You're currently inside the bernie_control session."
        read -p "You must kill the current session before you relaunch this script. Kill the current session?: [Y/n]: " kill_choice
        case "$kill_choice" in
            n|N)
                echo "Cancelled."
                exit 1
                ;;
            *)
                echo "Killing current session. Please run this script again to create a new session."
                tmux kill-session -t bernie_control
                exit 0
                ;;
        esac
    fi
fi

# Check if there's already a session called bernie_control
if tmux has-session -t bernie_control 2>/dev/null; then
    echo "WARNING: A tmux session named 'bernie_control' is already running."
    read -p "Do you want to kill it and start a new one (k) or attach to it (a)? [k/a]: " choice
    case "$choice" in
        a|A)
            echo "Attaching to existing session..."
            tmux attach -t bernie_control
            exit 0
            ;;
        k|K)
            echo "Killing existing session..."
            tmux kill-session -t bernie_control
            ;;
        *)
            echo "Invalid choice. Exiting."
            exit 1
            ;;
    esac
else
    echo "No existing bernie_control session found. Creating new session..."
fi

# Extract script name from the first script and create log file path
SCRIPT_NAME=$(basename "${SCRIPTS[0]}" .sh)
LOG_FILE="$SESSION_LOG_DIR/${SCRIPT_NAME}.log"

# Create a new detached tmux session with the first script and log output
echo "Creating tmux session..."
CMD=$(get_command_string "${SCRIPTS[0]}" "$LOG_FILE")
tmux new-session -d -s bernie_control "$CMD"

# Check if session was created successfully
if ! tmux has-session -t bernie_control 2>/dev/null; then
    echo "Failed to create tmux session. Check for errors in ${SCRIPTS[0]}"
    exit 1
fi

# Start with a horizontal split for the second script
if [ ${#SCRIPTS[@]} -gt 1 ]; then
    # Extract script name from the second script and create log file path
    SCRIPT_NAME=$(basename "${SCRIPTS[1]}" .sh)
    LOG_FILE="$SESSION_LOG_DIR/${SCRIPT_NAME}.log"
    
    CMD=$(get_command_string "${SCRIPTS[1]}" "$LOG_FILE")
    tmux split-window -h -t bernie_control:0 "$CMD"
    
    # Track pane direction and index for alternating between horizontal and vertical splits
    current_pane=0
    horizontal=1
    
    # Create remaining panes for scripts[2] onwards
    for ((i=2; i<${#SCRIPTS[@]}; i++)); do
        # Extract script name and create log file path
        SCRIPT_NAME=$(basename "${SCRIPTS[$i]}" .sh)
        LOG_FILE="$SESSION_LOG_DIR/${SCRIPT_NAME}.log"
        
        CMD=$(get_command_string "${SCRIPTS[$i]}" "$LOG_FILE")
        if [ $horizontal -eq 1 ]; then
            tmux split-window -v -t bernie_control:0.$current_pane "$CMD"
            horizontal=0
        else
            tmux split-window -h -t bernie_control:0.$current_pane "$CMD"
            horizontal=1
        fi
        current_pane=$((current_pane + 1))
    done
fi

# Arrange all panes into an even tiled layout
tmux select-layout -t bernie_control:0 tiled

# Attach to the session
echo "Attaching to tmux session..."
tmux attach -t bernie_control