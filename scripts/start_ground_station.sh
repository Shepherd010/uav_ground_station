#!/bin/bash
# Ground station core startup script
# Starts roscore (if not running) and launches navigator + safety_monitor + waypoint_manager
# Run on: ground station (192.168.31.30 in the default onboard setup)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"

# -----------------------------------------------------------------------------
# ROS network defaults for the current ground station (this machine):
#   - ROS Master runs on 192.168.31.116
#   - Onboard computer (192.168.31.180) connects to this master
# Override by setting ROS_MASTER_URI / ROS_IP / ROS_HOSTNAME before running.
# -----------------------------------------------------------------------------
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
# ROS_HOSTNAME 可选：仅在用户已设置时才导出，避免空字符串导致 ROS 主机名解析失败
if [ -n "${ROS_HOSTNAME:-}" ]; then
    export ROS_HOSTNAME
fi

echo "=========================================="
echo "  Ground Station Core Startup"
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
    echo "[✓] Workspace loaded: $WORKSPACE"
else
    echo "[!] Workspace not built. Run: catkin build"
    exit 1
fi

# Start roscore if it is not already running
if ! rosnode list &>/dev/null; then
    echo "[!] roscore is not running, starting it..."
    roscore > /tmp/roscore.log 2>&1 &
    sleep 3
fi

echo "[✓] roscore is running"

# Launch ground station core nodes
echo ""
echo "Starting ground station core nodes..."
roslaunch uav_navigator ground_station.launch

echo ""
echo "Ground station core finished."
