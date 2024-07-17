#!/bin/bash

usage() {
    echo "  options:"
    echo "      -p: platorm. Default: gz. Choices:"
    echo "        ms: multirotor simulator"
    echo "        gz: gazebo"
    echo "        cf: crazyflie"
    echo "      -g: launch using gnome-terminal instead of tmux"
}

# Initialize variables with default values
platform="gz"
use_gnome="false"

# Parse command line arguments
while getopts "p:g" opt; do
  case ${opt} in
    p )
      platform="${OPTARG}"
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

# Select between tmux and gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drone_namespaces[@]}.txt"
if [[ ${use_gnome} == "true" ]]; then
  tmuxinator_mode="debug"
  tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Launch aerostack2 for each drone namespace
for namespace in ${drone_namespaces[@]}
do
  base_launch="false"
  if [[ ${namespace} == ${drone_namespaces[0]} ]]; then
    base_launch="true"
  fi
  eval "tmuxinator ${tmuxinator_mode} -n ${namespace} -p tmuxinator/aerostack2.yaml \
    drone_namespace=${namespace} \
    platform=${platform} \
    config_folder=${config_folder} \
    base_launch=${base_launch} \
    ${tmuxinator_end}"

  sleep 0.1 # Wait for tmuxinator to finish
done

# Attach to tmux session
if [[ ${use_gnome} == "false" ]]; then
  tmux attach-session -t ${drone_namespaces[0]}
# If tmp_file exists, remove it
elif [[ -f ${tmp_file} ]]; then
  rm ${tmp_file}
fi