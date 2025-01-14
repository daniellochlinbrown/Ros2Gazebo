#!/bin/bash
# Update package list
sudo apt update

# Install ROS metapackage
sudo apt install -y ros-humble-desktop

# Install additional dependencies
sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-gazebo-ros \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-image-pipeline

# Resolve dependencies for your workspace
rosdep update
rosdep install --from-paths src --ignore-src -r -y
