#!/usr/bin/env bash
set -e

echo
echo "Installing ROS2 Humble ..."

# Check that apt is up to date
sudo apt update && sudo apt install locales
# Check locale settings
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
# Install ROS2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
## Install desktop and dev tools
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Source ROS2 Humble and add it to the user .bashrc
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Some Python dependencies must also be installed
pip install --user -U empy==3.3.4 pyros-genmsg setuptools==70.2.0

echo
echo "[DONE] ROS2 Humble installed!"
