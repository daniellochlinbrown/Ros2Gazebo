#!/usr/bin/env bash
set -e

# script directory
export DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

UBUNTU_RELEASE=$(lsb_release -rs)
if [[ "${UBUNTU_RELEASE}" != "22.04" ]]; then
  echo "Ubuntu distro unsupported, see docker px4io/px4-dev-base"
  return 1
elif [[ "${UBUNTU_RELEASE}" == "22.04" ]]; then
  ROS_DISTRO="humble"
fi

source ${DIR}/install_px4.sh

source ${DIR}/${ROS_DISTRO}/install_ros-${ROS_DISTRO}.sh

source ${DIR}/configure_workspace.sh

echo "**************************************"
echo "*              [DONE!]               *"
echo "*  UAS Gazebo simulator installed!   *"
echo "**************************************"
