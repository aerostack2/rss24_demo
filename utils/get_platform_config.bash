#!/bin/bash

get_platform_config() {
  local platform=$1

  case ${platform} in
    ms )
      config_folder="platforms_config/multirotor_simulator"
      drone_namespaces=$(python3 utils/get_drones.py -p ${config_folder}/world.yaml --sep ',')
      ;;
    gz )
      config_folder="platforms_config/gazebo"
      drone_namespaces=$(python3 utils/get_drones.py -p ${config_folder}/world.yaml --sep ',')
      ;;
    cf )
      config_folder="platforms_config/crazyflie"
      drone_namespaces=$(python3 utils/get_drones.py -p ${config_folder}/config_file.yaml --sep ',')
      ;;
    * )
      echo "Invalid platform: ${platform}" >&2
      exit 1
      ;;
  esac

  echo "${config_folder} ${drone_namespaces}"
}

get_platform_config "$1"