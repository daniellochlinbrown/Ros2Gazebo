#!/usr/bin/env bash
set -e

## Bash script to setup PX4 development environment on Ubuntu LTS (22.04). Extracted from https://docs.px4.io/main/en/ros2/user_guide.html#ros2-launch

# script directory
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# Change this to your desired PX4 version
PX4_VERSION="v1.15.3"

# check ubuntu version
UBUNTU_RELEASE=$(lsb_release -rs)
if [[ "${UBUNTU_RELEASE}" != "22.04" ]]; then
  echo "Ubuntu distro unsupported, see docker px4io/px4-dev-base"
  return 1
elif [[ "${UBUNTU_RELEASE}" == "22.04" ]]; then
  ROS_DISTRO="humble"
fi


# PX4-Autopilot Firmware
echo
echo "Cloning PX4-Autopilot Firmware ..."
## Clone firmware source code
bold=$(tput bold)
normal=$(tput sgr0)
if [ -z "$PX4_DIR" ]; then
  echo
  read -e -p "Please type the PX4-Autopilot Firmware ${bold}parent${normal} directory? " -i "~/Software/PX4-Autopilot" PX4_DIR
  export PX4_DIR
fi
mkdir -p $PX4_DIR
cd $PX4_DIR
git clone --recursive -b $PX4_VERSION https://github.com/PX4/PX4-Autopilot.git $PX4_DIR

## Install Ubuntu Dependencies
bash $PX4_DIR/Tools/setup/ubuntu.sh

## Compile PX4 Firmware
cd $PX4_DIR
DONT_RUN=1 make px4_sitl

echo "*****************************************"
echo "[DONE] Completed installing PX4 firmware!"

cd $DIR
