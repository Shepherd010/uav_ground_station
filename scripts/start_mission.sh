#!/bin/bash
# Mission startup script
# Loads a waypoint XML file and sends the START command to the navigator
# Run on: ground station

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"

WAYPOINT_FILE="${1:-/home/groundstation/waypoints.xml}"

# Use the same ROS master as the ground station core
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.31.116:11311}"
export ROS_IP="${ROS_IP:-192.168.31.116}"
# ROS_HOSTNAME 可选：仅在用户已设置时才导出，避免空字符串导致 ROS 主机名解析失败
if [ -n "${ROS_HOSTNAME:-}" ]; then
    export ROS_HOSTNAME
fi

echo "=========================================="
echo "  Start Mission"
echo "=========================================="
echo "  Waypoint file: $WAYPOINT_FILE"
echo "  ROS_MASTER_URI: $ROS_MASTER_URI"
echo ""

# Load ROS environment
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi

if [ -f "$WORKSPACE/devel/setup.bash" ]; then
    source "$WORKSPACE/devel/setup.bash"
fi

# Check roscore
if ! rosnode list &>/dev/null; then
    echo "[✗] roscore is not running. Please run start_ground_station.sh first."
    exit 1
fi

# Check navigator node
if ! rosnode list | grep -q "uav_navigator"; then
    echo "[!] Warning: navigator node is not running, navigation may fail"
fi

echo "Loading waypoints..."
rosservice call /uav/waypoint_manager/load_waypoints "{file_path: '$WAYPOINT_FILE'}"

echo ""
echo "Starting navigation..."
rosservice call /uav/navigator/command "{command: 'START'}"

echo ""
echo "Navigation started!"
echo ""
echo "Available commands:"
echo "  Pause:      rosservice call /uav/navigator/command \"{command: 'PAUSE'}\""
echo "  Cancel:     rosservice call /uav/navigator/command \"{command: 'CANCEL'}\""
echo "  Land:       rosservice call /uav/navigator/command \"{command: 'LAND'}\""
echo "  ReturnHome: rosservice call /uav/navigator/command \"{command: 'RETURN_TO_HOME'}\""
echo "  Emergency:  rosservice call /uav/navigator/command \"{command: 'EMERGENCY_STOP'}\""
echo "  Status:     rostopic echo /uav/navigator/status"
