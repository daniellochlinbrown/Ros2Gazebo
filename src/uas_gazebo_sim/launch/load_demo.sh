#!/bin/bash

# Default values
WORLD_NAME="aspa135_m3"
MODEL_NAME="x500"
VERBOSE_FLAG="" # Verbose is off by default
MODEL_POSE="0,0,0,0,0,0"
# PX4_GZ_MODEL_POSE_1="0,0,10,0,0,0"
# PX4_GZ_MODEL_POSE_2="0,0,10,0,0,0"
ARUCO_LAUNCH_FILE="aruco_single_launch.py" # Default Aruco detection launch file

# Usage message
usage() {
  echo "Usage: $0 [-w world_name] [-v]"
  echo "  -w  Specify the world name (default: aspa135_m3)"
  echo "  -v  Enable verbose output"
  echo "  -a  Specify the Aruco detection launch file (default: aruco_single.launch.py)"
  exit 1
}

# Parse command line options
while getopts ":w:v:m:a" opt; do
  case ${opt} in
  w)
    WORLD_NAME=${OPTARG}
    ;;
  v)
    VERBOSE_FLAG="-v"
    ;;
  a)
    ARUCO_LAUNCH_FILE=${OPTARG}
    ;;
  m)
    MODEL_NAME=${OPTARG}
    ;;
  \?)
    echo "Invalid Option: -$OPTARG" 1>&2
    usage
    ;;
  :)
    echo "Invalid Option: -$OPTARG requires an argument" 1>&2
    usage
    ;;
  esac
done
shift $((OPTIND - 1))

if [ "$WORLD_NAME" == "serf" ]; then
  MODEL_POSE="0,0,-3.65"
fi

if [ "$WORLD_NAME" == "aspa135_m3" ]; then
  MODEL_POSE="0,0,1.81,0,0,0"
  echo "World name: $WORLD_NAME"
fi

if [ "$WORLD_NAME" == "bunger" ]; then
  MODEL_POSE="-2,2,10,0,0,0"
  echo "World name: $WORLD_NAME"
fi

# Define repository Path and world path
REPOSITORY_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd) # Go up one level
WORLD_PATH="$REPOSITORY_DIR/worlds/${WORLD_NAME}/${WORLD_NAME}.sdf"

# Ensure the PX4 directory exists
if [ ! -d "$PX4_DIR" ]; then
  echo "[ERROR]: PX4 Autopilot directory does not exist."
  exit 1
fi

# Start Gazebo simulation
echo "Starting Gazebo with world: $WORLD_PATH"
gz sim $VERBOSE_FLAG -r "$WORLD_PATH" --gui-config "$REPOSITORY_DIR/launch/gazebo_gui.config" &

# Wait a bit to ensure Gazebo starts properly
sleep 5

# Start PX4 SITL
echo "Starting PX4 SITL..."
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_${MODEL_NAME} PX4_GZ_WORLD=${WORLD_NAME} PX4_GZ_MODEL_POSE=${MODEL_POSE} $PX4_DIR/build/px4_sitl_default/bin/px4 &

#What is the function of "&" at the end of line 81

# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD="$WORLD_NAME" $PX4_DIR/build/px4_sitl_default/bin/px4 -i 1 
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD="$WORLD_NAME" PX4_GZ_MODEL_POSE="$PX4_GZ_MODEL_POSE_1" $PX4_DIR/build/px4_sitl_default/bin/px4 -i 2
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD="$WORLD_NAME" PX4_GZ_MODEL_POSE="$PX4_GZ_MODEL_POSE_2" $PX4_DIR/build/px4_sitl_default/bin/px4 -i 3

# Wait a bit to ensure PX4 SITL starts properly
sleep 10

# Source ROS 2 setup file
# source /opt/ros/humble/setup.bash
source $REPOSITORY_DIR/../../install/setup.bash  # This path should match ROS2 standards, but check your workspace location

# Start the ROS 2-Gazebo parameter bridge
echo "Starting ROS 2-Gazebo parameter bridge..."
ros2 run ros_gz_bridge parameter_bridge camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image &

# Wait a bit to ensure the bridge starts properly
sleep 5

# Start Aruco marker detection using aruco_ros
# sleep 5
# echo "Starting Aruco marker detection..."
# ros2 launch aruco_detector "$ARUCO_LAUNCH_FILE" &

# Wait a bit to ensure Aruco detection starts
sleep 5

# Detector model
ros2 run yolov5_ros2 yolo_node


# Start rqt_image_view to visualize the image
# echo "Starting rqt_image_view..."
# ros2 run rqt_image_view rqt_image_view

# Wait indefinitely or until you manually stop the script
wait
