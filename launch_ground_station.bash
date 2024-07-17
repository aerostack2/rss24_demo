#!/bin/bash

usage() {
    echo "  options:"
    echo "      -p: platorm. Default: gz. Choices:"
    echo "        ms: multirotor simulator"
    echo "        gz: gazebo"
    echo "        cf: crazyflie"
    echo "      -t: launch keyboard teleoperation, choices: [true | false]. Default: false"
    echo "      -v: open rviz, choices: [true | false]. Default: false"
    echo "      -g: launch using gnome-terminal instead of tmux"
}

# Initialize variables with default values
platform="gz"
keyboard_teleop="false"
rviz="false"
use_gnome="false"

# Parse command line arguments
while getopts "p:tvg" opt; do
  case ${opt} in
    p )
      platform="${OPTARG}"
      ;;
    t )
      keyboard_teleop="true"
      ;;
    v )
      rviz="true"
      ;;
    g )
      use_gnome="true"
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

# Check if the drone namespace is empty
if [[ -z ${drone_namespaces} ]]; then
  echo "No drones namespaces" >&2
  exit 1
fi

# Select between tmux and gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drone_namespaces[@]}.txt"
if [[ ${use_gnome} == "true" ]]; then
  tmuxinator_mode="debug"
  tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Launch aerostack2 nodes
echo "drones_namespace_comma: ${drones_namespace_comma}"
eval "tmuxinator ${tmuxinator_mode} -n ground_station -p tmuxinator/ground_station.yaml \
  drone_namespace=${drones_namespace_comma} \
  keyboard_teleop=${keyboard_teleop} \
  rviz=${rviz} \
  ${tmuxinator_end}"

# Attach to tmux session
if [[ ${use_gnome} == "false" ]]; then
  tmux attach-session -t ground_station
# If tmp_file exists, remove it
elif [[ -f ${tmp_file} ]]; then
  rm ${tmp_file}
fi