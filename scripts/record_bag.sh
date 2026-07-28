#!/bin/bash
# ============================================================================
# 综合 rosbag 录制脚本 — 录制所有相关话题，支持随时起停
# ============================================================================
# 用法：
#   ./scripts/record_bag.sh start              # 开始录制
#   ./scripts/record_bag.sh stop               # 停止录制
#   ./scripts/record_bag.sh status             # 查看录制状态
#   ./scripts/record_bag.sh start -o ~/mydir   # 指定输出目录
#
# 输出：
#   ~/experiments/rosbag/YYYY-MM-DD_HH-MM-SS/full_flight.bag
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"

# -----------------------------------------------------------------------------
# 默认配置
# -----------------------------------------------------------------------------
DEFAULT_OUTPUT_BASE="${HOME}/experiments/rosbag"
PID_FILE="/tmp/rosbag_record.pid"
OUTPUT_FILE="/tmp/rosbag_record_output.txt"

# 排除的话题（正则，管道分隔）：
#   /rosout*           — 日志聚合，极度冗余
#   /mavros/hil/       — HITL 仿真，真实飞行中无意义
#   /mavros/debug_value/ — 调试值，非必要
#   /mavros/log_transfer/ — 原始日志传输数据块
#   /mavros/tunnel/    — 隧道数据
#   /mavros/statustext/ — 状态文本（rosout 中有副本）
#   /mavros/timesync_status — 时间同步调试
#   /mavros/param/     — 参数快照（可从 rosparam dump 获取）
EXCLUDE_REGEX="/(rosout|mavros/hil/|mavros/debug_value/|mavros/log_transfer/|mavros/tunnel/|mavros/statustext/|mavros/timesync_status|mavros/param/)"

# -----------------------------------------------------------------------------
# 工具函数
# -----------------------------------------------------------------------------
info()  { echo -e "\033[1;32m[✓]\033[0m $*"; }
warn()  { echo -e "\033[1;33m[!]\033[0m $*"; }
error() { echo -e "\033[1;31m[✗]\033[0m $*"; }

# 检查 ROS 环境
ensure_ros() {
    if ! command -v rosbag &>/dev/null; then
        if [ -f /opt/ros/noetic/setup.bash ]; then
            source /opt/ros/noetic/setup.bash
        fi
    fi
    if ! command -v rosbag &>/dev/null; then
        error "rosbag 不可用，请先 source ROS 环境"
        exit 1
    fi
    if ! rosnode list &>/dev/null; then
        error "roscore 未运行，无法录制"
        exit 1
    fi
}

# 获取录制进程 PID
get_pid() {
    if [ -f "$PID_FILE" ]; then
        local pid
        pid=$(cat "$PID_FILE" 2>/dev/null || true)
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "$pid"
            return 0
        fi
    fi
    return 1
}

# -----------------------------------------------------------------------------
# 状态查询
# -----------------------------------------------------------------------------
cmd_status() {
    ensure_ros

    local pid
    if pid=$(get_pid); then
        info "录制中"
        echo "  PID:       $pid"

        # 读出启动时的元信息
        if [ -f "$OUTPUT_FILE" ]; then
            local dir start_time
            dir=$(head -1 "$OUTPUT_FILE" 2>/dev/null || echo "未知")
            start_time=$(sed -n '2p' "$OUTPUT_FILE" 2>/dev/null || echo "未知")
            echo "  输出目录:  $dir"
            echo "  开始时间:  $start_time"

            # 文件大小（持续增长中）
            local bag_file="${dir}/full_flight.bag"
            if [ -f "$bag_file" ]; then
                local size
                size=$(du -h "$bag_file" 2>/dev/null | cut -f1)
                echo "  bag 大小:  $size"
            fi

            # 运行时长
            local now start_epoch elapsed
            now=$(date +%s)
            start_epoch=$(date -d "$start_time" +%s 2>/dev/null || echo 0)
            if [ "$start_epoch" -gt 0 ]; then
                elapsed=$((now - start_epoch))
                printf "  已运行:    %02d:%02d:%02d\n" $((elapsed/3600)) $(( (elapsed%3600)/60 )) $((elapsed%60))
            fi
        fi

        echo ""
        echo "  当前话题数: $(rostopic list 2>/dev/null | wc -l)"
    else
        warn "当前未在录制"
        if [ -f "$PID_FILE" ]; then
            echo "  发现残留 PID 文件，已自动清理"
            rm -f "$PID_FILE"
        fi
    fi
}

# -----------------------------------------------------------------------------
# 开始录制
# -----------------------------------------------------------------------------
cmd_start() {
    ensure_ros

    # 检查是否已在录制
    if get_pid &>/dev/null; then
        error "已在录制中，请先执行 stop"
        cmd_status
        exit 1
    fi

    # 解析参数
    local output_base="$DEFAULT_OUTPUT_BASE"
    while [ $# -gt 0 ]; do
        case "$1" in
            -o|--output) output_base="$2"; shift 2 ;;
            *) warn "忽略未知参数: $1"; shift ;;
        esac
    done

    # 创建带时间戳的输出目录
    local timestamp dir_name output_dir
    timestamp=$(date +"%Y-%m-%d_%H-%M-%S")
    dir_name="flight_${timestamp}"
    output_dir="${output_base}/${dir_name}"
    mkdir -p "$output_dir"

    # 保存元信息
    echo "$output_dir"  >  "$OUTPUT_FILE"
    echo "$timestamp"   >> "$OUTPUT_FILE"

    # 导出话题清单（方便事后查看录制时的话题全貌）
    rostopic list > "${output_dir}/topic_list.txt" 2>/dev/null || true

    info "开始录制..."
    echo "  输出目录:  $output_dir"
    echo "  话题总数:  $(rostopic list 2>/dev/null | wc -l)"
    echo "  排除:      $EXCLUDE_REGEX"
    echo ""

    # 启动 rosbag record 在后台
    # 使用 -a 录制全部话题，-x 排除指定的冗余话题
    # --bz2 压缩（可选，如需节省空间可取消注释）
    rosbag record \
        -a \
        -x "$EXCLUDE_REGEX" \
        -o "${output_dir}/full_flight" \
        --quiet \
        &>/dev/null &
    local bag_pid=$!

    # 短暂等待确认启动成功
    sleep 0.5
    if ! kill -0 "$bag_pid" 2>/dev/null; then
        error "rosbag 启动失败，请检查 ROS 环境"
        rm -f "$OUTPUT_FILE"
        exit 1
    fi

    echo "$bag_pid" > "$PID_FILE"
    info "录制已启动 (PID: $bag_pid)"
    echo ""
    echo "  ⏺  运行 './scripts/record_bag.sh stop'   停止录制"
    echo "  ⏺  运行 './scripts/record_bag.sh status'  查看状态"
    echo ""
    echo "  bag 文件写入: ${output_dir}/full_flight*.bag"
}

# -----------------------------------------------------------------------------
# 停止录制
# -----------------------------------------------------------------------------
cmd_stop() {
    ensure_ros

    local pid
    if ! pid=$(get_pid); then
        warn "当前未在录制"
        rm -f "$PID_FILE" "$OUTPUT_FILE"
        exit 0
    fi

    local dir
    dir=$(head -1 "$OUTPUT_FILE" 2>/dev/null || echo "未知")

    info "正在停止录制 (PID: $pid)..."
    # 发送 SIGINT 让 rosbag 优雅关闭 bag 文件
    kill -INT "$pid" 2>/dev/null || true

    # 等待进程退出（最多等 15 秒）
    local waited=0
    while kill -0 "$pid" 2>/dev/null && [ $waited -lt 15 ]; do
        sleep 0.5
        waited=$((waited + 1))
    done

    # 如果还没退出，强制杀死
    if kill -0 "$pid" 2>/dev/null; then
        warn "rosbag 未及时退出，强制终止..."
        kill -9 "$pid" 2>/dev/null || true
        sleep 0.5
    fi

    rm -f "$PID_FILE" "$OUTPUT_FILE"

    # 查找实际的 bag 文件
    if [ "$dir" != "未知" ] && [ -d "$dir" ]; then
        local bag_file
        bag_file=$(ls -t "${dir}"/full_flight*.bag 2>/dev/null | head -1 || true)
        if [ -n "$bag_file" ] && [ -f "$bag_file" ]; then
            local size
            size=$(du -h "$bag_file" 2>/dev/null | cut -f1)
            info "录制已停止"
            echo "  bag 文件: $bag_file"
            echo "  大小:     $size"
        else
            info "录制已停止"
            echo "  输出目录: $dir"
        fi
    else
        info "录制已停止"
    fi
}

# -----------------------------------------------------------------------------
# 主入口
# -----------------------------------------------------------------------------
case "${1:-}" in
    start)
        shift
        cmd_start "$@"
        ;;
    stop)
        cmd_stop
        ;;
    status)
        cmd_status
        ;;
    *)
        echo "用法: $0 {start|stop|status}"
        echo ""
        echo "  start        开始录制全部相关话题"
        echo "  start -o DIR 指定输出目录（默认: $DEFAULT_OUTPUT_BASE）"
        echo "  stop         停止录制"
        echo "  status       查看录制状态"
        echo ""
        echo "示例:"
        echo "  $0 start"
        echo "  $0 start -o ~/flight_data"
        echo "  $0 status"
        echo "  $0 stop"
        exit 1
        ;;
esac
