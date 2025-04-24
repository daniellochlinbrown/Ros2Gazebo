#!/usr/bin/env bash
# This script is used to source the UAS Gazebo Simulator environment variables.

## terminal colors
bold=$(tput bold)
normal=$(tput sgr0)

echo -n "Sourcing ${bold}UAS Gazebo Simulator${normal} ... "

# Check Ubuntu version
PYTHON_PREFIX="python"
UBUNTU_RELEASE=$(lsb_release -rs)
if [[ "${UBUNTU_RELEASE}" != "22.04" ]]; then
  echo
  echo "Ubuntu ${UBUNTU_RELEASE} unsupported."
  return 1
else
  ROS_DISTRO="humble"
fi

# Script directory
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# Repository directory
REPO_PATH=$(cd "$(dirname "$DIR")" && pwd)
# Workspace directory
WS_PATH=$(cd "$(dirname "$DIR")/../.." && pwd)

# Source Environments
## ROS2 Distro
source /opt/ros/$ROS_DISTRO/setup.bash

## ROS2 Workspace
source ${WS_PATH}/install/setup.bash

## Gazebo Sim ENV VARs
export GZ_SIM_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:${REPO_PATH}/models:${REPO_PATH}/worlds

## PX4 ENV VARs for ROS2
if [ -z "$PX4_DIR" ]; then
  echo
  echo "The ${bold}PX4_DIR${normal} variable has not been set in your ~/.bashrc."
  read -e -p "Where is your PX4 Firmware directory? " -i "~/Software/PX4-Autopilot" PX4_DIR
  export PX4_DIR
  echo "export PX4_DIR=$PX4_DIR" >>~/.bashrc
  echo "The ${bold}PX4_DIR${normal} variable has been set in your ~/.bashrc cfg file."
  echo
  source ~/.bashrc
fi

# Set ROS2 environment variables
export COLCON_PREFIX_PATH=$PX4_DIR/build/px4_sitl_default/install:${COLCON_PREFIX_PATH}
export AMENT_PREFIX_PATH=$PX4_DIR/build/px4_sitl_default/install:${AMENT_PREFIX_PATH}

echo "[DONE!]"
