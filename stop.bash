#!/bin/bash

usage() {
    echo "  options:"
    echo "      -p: platorm. Default: gz. Choices:"
    echo "        ms: multirotor simulator"
    echo "        gz: gazebo"
    echo "        cf: crazyflie"
    echo "      -n: namespaces of the drone, separated by comma"
}

# Initialize variables with default values
platform="gz"
drones_namespace_comma=""

# Parse command line arguments
while getopts "p:n:" opt; do
  case ${opt} in
    p )
      platform="${OPTARG}"
      ;;
    n )
      drones_namespace_comma="${OPTARG}"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# Process input for each platform
config_output=$(bash utils/get_platform_config.bash "${platform}")
# Read the output into individual variables and the drones_namespace_comma into an array
read -r config_folder drones_namespace_comma <<< "$config_output"
# Convert the drones_namespace_comma of the output into an array
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# List of tmux sessions to be killed
tmux_session_list=("ground_station" "rosbag")

# Add each drone namespace to the sessions to be killed
for namespace in ${drone_namespaces[@]}
do
  tmux_session_list+=("${namespace}")
done

# TODO(RPS): Remove this
case ${platform} in
  gz )
    pkill -f 'gz sim server'
    ;;
esac

# If inside tmux session, get the current session name
if [[ -n "$TMUX" ]]; then
  current_session=$(tmux display-message -p '#S')
fi

# Send Ctrl+C signal to each window of each session
for session in ${tmux_session_list[@]}; do
  # Check if session exists
  if tmux has-session -t "$session" 2>/dev/null; then
    # Get the list of windows in the session
    windows=($(tmux list-windows -t "$session" -F "#{window_index}"))
    # Iterate through each window and send Ctrl+C
    for window in "${windows[@]}"; do
      # Send Ctrl+C to the window
      tmux send-keys -t "$session:$window" C-c
      sleep 0.1 # Add a small delay to allow the signal to be processed
    done
  fi
done

# Kill all tmux sessions from the list except for the current one
for session in ${tmux_session_list[@]}; do
  if [[ "$session" != "$current_session" ]]; then
    tmux kill-session -t "$session" 2>/dev/null
  fi
done

# Kill the current tmux session, if in a tmux session
if [[ -n "$TMUX" ]]; then
  tmux kill-session -t "$current_session" 2>/dev/null
fi