#!/usr/bin/env bash

# This script is used to launch a demo environment for UAV simulation.
# It initializes the following components:
# 1. PX4 and Gazebo for UAV simulation.
# 2. Computer Vision (CV) modules.
# 3. RTPS (Real-Time Publish-Subscribe) bridge for communication.
# 4. Motion control systems for UAV operation.
#
# Usage:
# Run this script to set up the complete simulation environment in a tmux session.
#
# Prerequisites:
# - Ensure that PX4, Gazebo, and required dependencies are installed.
# - Verify that the RTPS bridge and motion control modules are properly configured.
#

# The script includes default values for configuration and can be customized as needed.
WORLD_NAME="aspa135_m3"
MODEL_NAME="x500_vision"
VERBOSE_FLAG=""
MODEL_0_POSE="0,0,0,0,0,0"
MODEL_1_POSE="0,0,0,0,0,0"
ARUCO_LAUNCH_FILE="aruco_single_launch.py"


usage() {
  echo "Usage: $0 [-w world_name] [-m model_name] [-v] [-a aruco_launch]"
  exit 1
}

while getopts ":w:m:va:" opt; do
  case $opt in
    w) WORLD_NAME="$OPTARG" ;;
    v) VERBOSE_FLAG="-v" ;;
    a) ARUCO_LAUNCH_FILE="$OPTARG" ;;
    *) usage ;;
  esac
done
shift $((OPTIND-1))

# Adjust MODEL_POSE based on WORLD_NAME
case "$WORLD_NAME" in
  serf)    
    MODEL_0_POSE="0,0,-3.65,0,0,0"
    MODEL_1_POSE="0,-2,-3.65,0,0,0"
  ;;
  aspa135_m3) 
    MODEL_0_POSE="5.05,3.24,32.31,0,0,0"
    MODEL_1_POSE="5.05,5.24,32.31,0,0,0"
  ;;
  bunger)
    MODEL_0_POSE="-2,2,10,0,0,0"
    MODEL_1_POSE="-2,0,10,0,0,0"
  ;;
  *)
    echo "[WARN] Unknown world '$WORLD_NAME', using defaults"
    ;;
esac

# Check PX4 directory
if [ ! -d "$PX4_DIR" ]; then
  echo "[ERROR] PX4 directory not found at $PX4_DIR"
  exit 1
fi

# Repo paths
REPOSITORY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_PATH="$REPOSITORY_DIR/worlds/${WORLD_NAME}/${WORLD_NAME}.sdf"

# PX4 and Gazebo UAS model commands
GZ_RUN_CMD="gz sim $VERBOSE_FLAG -r '$WORLD_PATH' --gui-config $REPOSITORY_DIR/launch/gazebo_gui.config"
PX4_RUN_0_CMD="PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_${MODEL_NAME} PX4_GZ_WORLD=${WORLD_NAME} PX4_GZ_MODEL_POSE=${MODEL_0_POSE} $PX4_DIR/build/px4_sitl_default/bin/px4 -i 0"
PX4_RUN_1_CMD="PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_${MODEL_NAME} PX4_GZ_WORLD=${WORLD_NAME} PX4_GZ_MODEL_POSE=${MODEL_1_POSE} $PX4_DIR/build/px4_sitl_default/bin/px4 -i 1"

# Source ROS 2 and workspace setup files
source /opt/ros/humble/setup.bash
source $REPOSITORY_DIR/../../install/setup.bash  # This path should match ROS2 standards, but check your workspace location

# -- Tmux session setup --
# Names
SESSION="ugs_multi"
WINDOW="run"

# Start a detached session
tmux new-session -d -s $SESSION -n $WINDOW

# Split into 2 columns
tmux split-window -h -t $SESSION:$WINDOW

# Split each column into two rows
tmux select-pane -t $SESSION:$WINDOW.1
tmux split-window -v -t $SESSION:$WINDOW
tmux select-pane -t $SESSION:$WINDOW.0
tmux split-window -v -t $SESSION:$WINDOW

# Pane layout now:
#  Pane 0 | Pane 2
#  -------+-------
#  Pane 1 | Pane 3

# Pane 0: PX4
tmux select-pane -t $SESSION:$WINDOW.0
tmux send-keys \
  "sleep 15" C-m \
  "$PX4_RUN_0_CMD" C-m \
  "$PX4_RUN_1_CMD" C-m # It seems you need a separate pane per PX4 command. Work on this.

# Pane 1: Computer vision (YOLOv5)
tmux select-pane -t $SESSION:$WINDOW.1
tmux send-keys \
  "sleep 17" C-m \
  "ros2 run yolov5_ros2 yolo_node" C-m

# Pane 2: RTPS bridge
tmux select-pane -t $SESSION:$WINDOW.2
tmux send-keys \
  "ros2 run ros_gz_bridge parameter_bridge camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image" C-m

# Pane 3: Motion control (WIP)
tmux select-pane -t $SESSION:$WINDOW.3
tmux send-keys \
  "echo '🤖 [WIP] Launch your motion-control ROS2 package here'" C-m \
  "# e.g. ros2 run my_drone_control motion_node" C-m

# Pane 5: Gazebo Sim
tmux select-pane -t $SESSION:$WINDOW.0
tmux split-window -v -t $SESSION:$WINDOW
tmux send-keys \
  "$GZ_RUN_CMD" C-m

# Finally, attach
tmux select-pane -t $SESSION:$WINDOW.0
tmux attach-session -t $SESSION
