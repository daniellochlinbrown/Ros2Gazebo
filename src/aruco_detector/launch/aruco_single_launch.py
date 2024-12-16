#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gazebo',
            output='screen',
            arguments=['-s', 'libgazebo_ros_factory.so'],
        ),
        Node(
            package='aruco_detector',
            executable='aruco_detector.py',
            name='aruco_detector_node',
            output='screen'
        ),
    ])
