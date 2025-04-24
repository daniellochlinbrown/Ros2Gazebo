# UAS Gazebo Simulator <!-- omit in toc -->

Basic simulation environment of an X500 UAV using ROS2, Gazebo Ignition, and PX4 Autopilot firmware.  
The repository contains source code that supports PX4 Software in the Loop (SIL) and Hardware in the Loop (HIL).

## Table of Contents <!-- omit in toc -->

- [System Requirements](#system-requirements)
  - [Operating systems](#operating-systems)
- [Installation **(Last Revision: 2024-07-03)**](#installation-last-revision-2024-07-03)
  - [Important Notes](#important-notes)
  - [Steps](#steps)
- [Quick Start (Command-line Interface)](#quick-start-command-line-interface)
  - [Loading Simulator](#loading-simulator)
  - [UAV arming, take-off and landing](#uav-arming-take-off-and-landing)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [Changelog](#changelog)
- [Copyright](#copyright)

## System Requirements

### Operating systems

- Linux (Ubuntu 22.04.X LTS)

## Installation **(Last Revision: 2024-07-03)**

Installation steps are automated using shell scripts and direct commands to streamline the setup process. The installation should work on clean Ubuntu installations.

### Important Notes

The installation and core scripts assume the following folder structure for this project:

- A folder containing the *default* ROS2 workspace (e.g., `~/ros2_ws`).
- A *parent* folder containing PX4 Autopilot code (e.g., **Autopilot parent folder:** `~` and **PX4 Firmware:** `~/PX4-Autopilot`).
- A folder containing your customized UAS ROS2 workspace (e.g., `~/qut_uas_ws`).  
  The folder location could be the same as the one used as your default workspace (_i.e._ `ros2_ws`).

### Steps

Open a terminal and run the following commands:

1. Set a *ROS2 workspace* directory and download this repository.  
   The workspace directory can be either your default `ros2_ws` path or a new directory.  
   As an example, the target directory will be located at `~/Software/qut_uas_ws`:

    ```sh
    export WS_DIR=~/Software/qut_uas_ws
    mkdir -p $WS_DIR/src
    cd $WS_DIR/src
    ```

    Clone the repository:

    ```sh
    git clone git@github.com:saef-uas/uas_gazebo_sim.git
    ```

2. **Add Gazebo and ROS Sources**

   To install Gazebo and ROS 2 Humble, add the required repositories:

   ```sh
   sudo sh -c 'echo "deb [trusted=yes] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update
   sudo apt install curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
   sudo apt update

3. **Install Core ROS 2 Packages and PX4 Dependencies**

   Run the following command to install ROS 2 Humble Desktop, PX4 message definitions, and essential simulation tools:

   ```sh
   sudo apt install -y \
       ros-humble-desktop \
       python3-colcon-common-extensions \
       ros-humble-common-interfaces \
       ros-humble-gazebo-ros-pkgs \
       ros-humble-px4-msgs \
       ros-humble-mavros \
       ros-humble-mavros-extras \
       ros-humble-ament-cmake-auto \
       ros-humble-ament-lint-auto \
       ros-humble-ament-cmake-flake8 \
       ros-humble-ament-cmake-pep257 \
       ros-humble-ament-cmake-cpplint \
       ros-humble-ament-cmake-pytest \
       ros-humble-ament-cmake \
       ros-humble-ros-gz-bridge \
       ros-humble-ros-gz-sim \
       ros-humble-image-pipeline

4. Execute the [autoinstall_requirements.sh](setup/autoinstall_requirements.sh) script to install ROS, Gazebo, PX4 Firmware, common dependencies, and build the ROS package.

    ```sh
    cd uas_gazebo_sim/setup
    bash autoinstall_requirements.sh
    ```

---

## Quick Start (Command-line Interface)

### Loading Simulator

To load the simulation environment, open a new terminal and type:

```sh
activate qutas_sim
```
Or

```sh
source ${WS_PATH}/src/uas_gazebo_sim/scripts/init_simulator.sh
```

to source the ROS2 workspace and the simulator environment files.
Then, run:

```sh
killgazebo # Kills any running gazebo processes
cd <package-path>
bash launch/load_demo.sh -w aspa135_m3
```

to load Gazebo with PX4 and the chosen world environment (in this case, SERF).

### UAV arming, take-off and landing

It is possible to send commands from the PX4 interactive console using the terminal where the simulation environment was loaded.
As soon as you start typing, you will observe the `pxh>` prefix in the terminal session.

```sh
pxh> commander arm
pxh> commander takeoff
pxh> commander land
```

<!-- ### Waypoints

To run the ROS node with pre-defined waypoints you can either use `rosrun`:

```sh
rosrun uas_gazebo_sim offboard_demo_node
```

or `roslaunch`:

```sh
roslaunch uas_gazebo_sim uas_demo_run.launch
```

You might need to re-source the `init_simulator.sh` script if you run these commands in a new terminal session. -->

## Documentation

Please check our [Wiki](https://gitlab.com/qut-asl-upo/uas_gazebo_sim/-/wikis/home) for extended documentation and tutorials.

## Contributing

Please check our contributing [guidelines](CONTRIBUTING.md) for further details.

## Changelog

See the [changelog](CHANGELOG.md) for further details.

## Copyright

Copyright &copy; 2018-2024 Juan Sandino (j.sandino@qut.edu.au), Felipe Gonzalez (felipe.gonzalez@qut.edu.au), Queensland University of Technology.
All rights reserved.

This software is protected under the Copyright Act 1968 and the
QUT MOPP F/5.1 (Copyright).
For authorised uses only.

**This software is confidential and cannot not be used, modified or shared unless for the approved specific purposes by the Copyright owners.**