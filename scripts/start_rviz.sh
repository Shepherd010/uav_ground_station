#!/bin/bash
# RViz ground station startup script
# Starts RViz with the waypoint panel plugin for waypoint annotation and monitoring
# Run on: ground station

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"

# Use the same ROS master as the ground station core
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
# ROS_HOSTNAME 可选：仅在用户已设置时才导出，避免空字符串导致 ROS 主机名解析失败
if [ -n "${ROS_HOSTNAME:-}" ]; then
    export ROS_HOSTNAME
fi

echo "=========================================="
echo "  RViz Ground Station Startup"
echo "=========================================="
echo "  ROS_MASTER_URI: $ROS_MASTER_URI"
echo "  ROS_IP:         $ROS_IP"
echo ""

# Load ROS environment
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
    echo "[✓] ROS Noetic environment loaded"
else
    echo "[✗] ROS Noetic setup not found"
    exit 1
fi

# Load workspace
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
    source "$WORKSPACE/devel/setup.bash"
    echo "[✓] Workspace loaded"
else
    echo "[!] Workspace not built"
    exit 1
fi

# Check roscore
if ! rosnode list &>/dev/null; then
    echo "[✗] roscore is not running. Please run start_ground_station.sh first."
    exit 1
fi

echo "[✓] roscore is connected"
echo ""

# Launch RViz with the integrated panel configuration
# (rviz_ground_station.launch auto-loads config.yaml + starts RViz)
echo "Starting RViz..."
roslaunch rviz_waypoint_panel rviz_ground_station.launch
