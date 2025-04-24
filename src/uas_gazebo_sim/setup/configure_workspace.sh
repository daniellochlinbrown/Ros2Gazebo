#!/usr/bin/env bash
set -e

echo 
echo "Configuring ROS2 workspace ..."

# check ubuntu version
UBUNTU_RELEASE=$(lsb_release -rs)
if [[ "${UBUNTU_RELEASE}" != "22.04" ]]; then
  echo "Ubuntu distro unsupported, see docker px4io/px4-dev-base"
  return 1
elif [[ "${UBUNTU_RELEASE}" == "22.04" ]]; then
  ROS_DISTRO="humble"
fi

# Worskpace dependencies
sudo apt update
sudo apt install \
  g++ libspatialindex-dev tmux -y

# Python dependencies
python3 -m pip install scipy --user

# QGroundControl dependencies
sudo apt install \
    geoclue-2.0 speech-dispatcher \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 -y
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y

echo
echo "Please log out and log back in to apply the usermod changes."

# TODO: check if these dependencies are needed in ROS2
# sudo apt-get install \
#   libpcl1 \
#   ros-$ROS_DISTRO-octomap-* \
#   ros-$ROS_DISTRO-vision-opencv \
#   ros-$ROS_DISTRO-camera-info-manager \
#   ros-$ROS_DISTRO-image-proc \
#   ros-$ROS_DISTRO-image-view \
#   ros-$ROS_DISTRO-image-transport-plugins \
#   -y

# script directory
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Download QGroundControl Appimage
mkdir -p ~/Software
cd ~/Software
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
cd ${DIR}

# Export directories
if [ -z "$WS_PATH" ]; then
  WS_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
  WS_PATH=$(dirname "$WS_DIR")
  export WS_PATH
fi

bold=$(tput bold)
normal=$(tput sgr0)
if [ -z "$ROS2_WS_DIR" ]; then
  echo
  read -e -p "Please type your ${bold}default${normal} ROS2 'ros2_ws' workspace directory? " -i "~/Software/ros2_ws" ROS2_WS_DIR
  export ROS2_WS_DIR
fi

if [ -z "$PX4_DIR" ]; then
  echo
  read -e -p "Please type the PX4-Autopilot Firmware ${bold}parent${normal} directory? " -i "~/Software" PX4_DIR
  export PX4_DIR
fi

# Clone PX4 msgs repository (required in ROS2 and PX4)
git clone https://github.com/PX4/px4_msgs.git ${ROS2_WS_DIR}/src/px4_msgs

echo "export ROS2_WS_PATH=${ROS2_WS_DIR}" >>~/.bashrc
echo "export WS_PATH=${WS_PATH}" >>~/.bashrc
echo "export PX4_DIR=${PX4_DIR}" >>~/.bashrc
# echo "export PX4_DIR=${PX4_DIR}/PX4-Autopilot" >>~/.bashrc

ACTIVATE_FUN='
# Alias to kill any running ROS and Gazebo processes
alias killgazebo="killall gz gzclient gzserver mavros_server roscore"

## Function that handles different environments that conflict with ROS, python, etc.
function activate() {
  if [[ $@ == "qutas_sim" ]]; then
    source ${WS_PATH}/src/uas_gazebo_sim/scripts/init_simulator.sh
  else
    command activate "$@"
  fi
}

export -f activate
'

# Check if activate function exists in ~/.bashrc
if [[ "$(type -t activate)" == "function" ]]; then
  echo -e -p "Activate function already declared in ~/.bashrc."
else
  echo "${ACTIVATE_FUN}" >>~/.bashrc
  echo "Activate function appended to ~/.bashrc."
fi

cd ${WS_PATH}
# Revise this code instruction
colcon build --packages-select px4_msgs uas_gazebo_sim -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source ~/.bashrc

cd ${DIR}

echo "[DONE] Completed configuring workspace!"
