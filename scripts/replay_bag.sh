#!/bin/bash
# ============================================================================
# rosbag 回放脚本
# ============================================================================
# 用法：
#   ./scripts/replay_bag.sh                         # 回放最新的 bag
#   ./scripts/replay_bag.sh /path/to/file.bag       # 回放指定 bag
#   ./scripts/replay_bag.sh /path/to/bag_directory  # 回放目录下的 bag 文件
#   ./scripts/replay_bag.sh --loop                  # 循环回放最新的 bag
#   ./scripts/replay_bag.sh --rate 0.5 file.bag     # 以半速回放
#
# 脚本会启动（如果尚未运行）：
#   1. roscore
#   2. 使用 uav_navigation.rviz 配置的 RViz
#   3. rosbag play（不限制话题，回放 bag 中的全部话题）
# ============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"
DEFAULT_BAG_ROOT="/home/groundstation/experiments/rosbag"
RVIZ_CONFIG="$WORKSPACE/src/rviz_waypoint_panel/config/uav_navigation.rviz"

ROSCORE_LOG="/tmp/uav_ground_station_replay_roscore_$$.log"
RVIZ_LOG="/tmp/uav_ground_station_replay_rviz_$$.log"

ROSCORE_PID=""
RVIZ_PID=""
BAG_PID=""
STARTED_ROSCORE=0
USE_SIM_TIME_CONFIGURED=0
USE_SIM_TIME_EXISTED=0
USE_SIM_TIME_VALUE=""
BAG_FILES=()
PLAYBACK_ARGS=(--clock)

info()  { printf '\033[1;32m[✓]\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m[!]\033[0m %s\n' "$*"; }
error() { printf '\033[1;31m[✗]\033[0m %s\n' "$*" >&2; }
die()   { error "$*"; exit 1; }

usage() {
    cat <<EOF
用法: $0 [选项] [bag文件或目录]

选项：
  --loop              循环回放
  --rate FACTOR       设置回放速度倍率（默认 1.0）
  -h, --help          显示帮助

不指定 bag 时，默认回放：
  $DEFAULT_BAG_ROOT 中最近修改的 .bag 文件

示例：
  $0
  $0 /home/groundstation/experiments/rosbag/flight_2026-09-08_15-22-39
  $0 --rate 0.5 full_flight.bag
EOF
}

# 停止一个由本脚本启动的后台进程，并尽量给它留下优雅退出的机会。
stop_process() {
    local pid="$1"
    local name="$2"
    local waited=0

    if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
        return 0
    fi

    info "正在停止 $name (PID: $pid)..."
    kill -INT "$pid" 2>/dev/null || true
    while kill -0 "$pid" 2>/dev/null && (( waited < 50 )); do
        sleep 0.1
        waited=$((waited + 1))
    done

    if kill -0 "$pid" 2>/dev/null; then
        warn "$name 未及时退出，发送 TERM..."
        kill -TERM "$pid" 2>/dev/null || true
        waited=0
        while kill -0 "$pid" 2>/dev/null && (( waited < 20 )); do
            sleep 0.1
            waited=$((waited + 1))
        done
    fi

    if kill -0 "$pid" 2>/dev/null; then
        warn "$name 仍未退出，发送 KILL..."
        kill -KILL "$pid" 2>/dev/null || true
    fi

    wait "$pid" 2>/dev/null || true
}

cleanup() {
    local exit_status=$?

    # 防止清理过程中再次触发自身。
    trap - EXIT INT TERM

    if [[ -n "$BAG_PID" ]]; then
        stop_process "$BAG_PID" "rosbag play"
        BAG_PID=""
    fi

    if [[ -n "$RVIZ_PID" ]]; then
        stop_process "$RVIZ_PID" "RViz"
        RVIZ_PID=""
    fi

    # 只关闭本脚本自己启动的 roscore，不影响用户已有的 ROS master。
    if (( STARTED_ROSCORE )) && [[ -n "$ROSCORE_PID" ]]; then
        stop_process "$ROSCORE_PID" "roscore"
        ROSCORE_PID=""
    fi

    # 如果脚本连接的是已有 roscore，恢复进入回放前的 sim time 参数。
    if (( USE_SIM_TIME_CONFIGURED )) && rosnode list >/dev/null 2>&1; then
        if (( USE_SIM_TIME_EXISTED )); then
            rosparam set /use_sim_time "$USE_SIM_TIME_VALUE" >/dev/null 2>&1 || true
        else
            rosparam delete /use_sim_time >/dev/null 2>&1 || true
        fi
    fi

    exit "$exit_status"
}

wait_for_ros_master() {
    local attempts=0

    while (( attempts < 30 )); do
        if rosnode list >/dev/null 2>&1; then
            return 0
        fi

        if [[ -n "$ROSCORE_PID" ]] && ! kill -0 "$ROSCORE_PID" 2>/dev/null; then
            return 1
        fi

        sleep 1
        attempts=$((attempts + 1))
    done

    return 1
}

start_roscore() {
    if rosnode list >/dev/null 2>&1; then
        info "检测到已有 roscore，继续使用当前 ROS master"
        return 0
    fi

    info "roscore 未运行，正在启动..."
    roscore >"$ROSCORE_LOG" 2>&1 &
    ROSCORE_PID=$!
    STARTED_ROSCORE=1

    if ! wait_for_ros_master; then
        error "roscore 启动失败，日志：$ROSCORE_LOG"
        tail -n 40 "$ROSCORE_LOG" >&2 || true
        exit 1
    fi

    info "roscore 已启动 (PID: $ROSCORE_PID)"
}

resolve_bag_files() {
    local input="${1:-}"
    local latest_bag=""

    if [[ -z "$input" ]]; then
        [[ -d "$DEFAULT_BAG_ROOT" ]] || die "rosbag目录不存在：$DEFAULT_BAG_ROOT"

        latest_bag=$(find "$DEFAULT_BAG_ROOT" -type f -name '*.bag' -printf '%T@ %p\n' 2>/dev/null \
            | sort -nr \
            | sed -n '1s/^[^ ]* //p')
        [[ -n "$latest_bag" ]] || die "在 $DEFAULT_BAG_ROOT 中没有找到 .bag 文件"
        BAG_FILES=("$latest_bag")
    elif [[ -f "$input" ]]; then
        BAG_FILES=("$(realpath "$input")")
    elif [[ -d "$input" ]]; then
        mapfile -t BAG_FILES < <(find "$input" -maxdepth 1 -type f -name '*.bag' -print | sort)
        ((${#BAG_FILES[@]} > 0)) || die "目录中没有找到 .bag 文件：$input"
    else
        die "bag文件或目录不存在：$input"
    fi

    local bag
    for bag in "${BAG_FILES[@]}"; do
        [[ -f "$bag" ]] || die "bag文件不存在：$bag"
        if ! rosbag info "$bag" >/dev/null; then
            die "无法读取 rosbag：$bag"
        fi
    done
}

load_replay_parameters() {
    [[ -f "$WORKSPACE/config.yaml" ]] || die "配置文件不存在：$WORKSPACE/config.yaml"

    # RViz 面板依赖 config.yaml 中的 topics、panel 等参数。
    rosparam load "$WORKSPACE/config.yaml"

    # 让 RViz/TF 使用 rosbag 的时间戳，避免回放时出现时间外推错误。
    USE_SIM_TIME_CONFIGURED=1
    if USE_SIM_TIME_VALUE="$(rosparam get /use_sim_time 2>/dev/null)"; then
        USE_SIM_TIME_EXISTED=1
    fi
    rosparam set /use_sim_time true
}

start_rviz() {
    [[ -f "$RVIZ_CONFIG" ]] || die "RViz配置文件不存在：$RVIZ_CONFIG"

    info "正在启动 RViz..."
    rviz -d "$RVIZ_CONFIG" >"$RVIZ_LOG" 2>&1 &
    RVIZ_PID=$!

    # 给 RViz 加载插件和显示项的时间，避免 rosbag 一启动就错过早期消息。
    sleep 2
    if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
        error "RViz 启动失败，日志：$RVIZ_LOG"
        tail -n 40 "$RVIZ_LOG" >&2 || true
        exit 1
    fi

    info "RViz 已启动 (PID: $RVIZ_PID)"
}

# ----------------------------------------------------------------------------
# 参数解析
# ----------------------------------------------------------------------------
BAG_INPUT=""
while (($# > 0)); do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --loop)
            PLAYBACK_ARGS+=(--loop)
            ;;
        --rate)
            (($# >= 2)) || die "--rate 需要一个速度倍率"
            if ! [[ "$2" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] \
                || [[ "$2" =~ ^0*([.]0*)?$ ]]; then
                die "无效的回放速度倍率：$2"
            fi
            PLAYBACK_ARGS+=(--rate "$2")
            shift
            ;;
        --)
            shift
            (($# == 1)) || die "只能指定一个 bag 文件或目录"
            BAG_INPUT="$1"
            ;;
        -* )
            die "未知选项：$1（使用 --help 查看用法）"
            ;;
        *)
            [[ -z "$BAG_INPUT" ]] || die "只能指定一个 bag 文件或目录"
            BAG_INPUT="$1"
            ;;
    esac
    shift
done

# ----------------------------------------------------------------------------
# ROS 环境
# ----------------------------------------------------------------------------
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
if [[ -n "${ROS_HOSTNAME:-}" ]]; then
    export ROS_HOSTNAME
fi

if [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
else
    die "未找到 ROS Noetic 环境：/opt/ros/noetic/setup.bash"
fi

if [[ -f "$WORKSPACE/devel/setup.bash" ]]; then
    # shellcheck disable=SC1090,SC1091
    source "$WORKSPACE/devel/setup.bash"
else
    die "工作区尚未构建：$WORKSPACE/devel/setup.bash"
fi

for required_command in roscore rosnode rosparam rosbag rviz; do
    command -v "$required_command" >/dev/null 2>&1 \
        || die "命令不可用：$required_command，请检查 ROS/工作区环境"
done

resolve_bag_files "$BAG_INPUT"

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_roscore
load_replay_parameters
start_rviz

echo ""
info "开始回放全部话题（使用 rosbag --clock）"
for bag in "${BAG_FILES[@]}"; do
    echo "  bag: $bag"
done
echo ""

rosbag play "${PLAYBACK_ARGS[@]}" "${BAG_FILES[@]}" &
BAG_PID=$!

set +e
wait "$BAG_PID"
BAG_STATUS=$?
set -e
BAG_PID=""

if (( BAG_STATUS != 0 )); then
    error "rosbag 回放异常结束（退出码：$BAG_STATUS）"
    exit "$BAG_STATUS"
fi

info "rosbag 回放完成"
info "RViz 将保持打开，关闭 RViz或按 Ctrl-C 结束本次回放会话"

# 回放结束后保留 RViz，便于查看最终轨迹；关闭 RViz 后触发统一清理。
set +e
wait "$RVIZ_PID"
RVIZ_STATUS=$?
set -e
RVIZ_PID=""

if (( RVIZ_STATUS != 0 )); then
    warn "RViz 已退出（退出码：$RVIZ_STATUS）"
fi
