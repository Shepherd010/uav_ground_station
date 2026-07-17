# uav_navigator — 导航核心包

无人机地面站导航控制核心，基于 10 状态状态机驱动 PX4 飞控，通过 MAVROS 实现 OFFBOARD 模式下的全自动航点飞行。

## 架构

```
┌─────────────────────────────────────────────────────────┐
│                    uav_navigator                         │
│                                                          │
│  ┌──────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │ navigator │  │safety_monitor│  │     logger        │  │
│  │ (状态机)  │  │  (独立安全)   │  │ (终端格式化输出)  │  │
│  └─────┬─────┘  └──────┬───────┘  └──────────────────┘  │
│        │               │                                 │
│  ┌─────┴───────────────┴──────────────────────────────┐ │
│  │            experiment_recorder (实验记录)            │ │
│  └────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

### 四个节点

| 节点 | 可执行文件 | 职责 |
|------|-----------|------|
| **navigator** | `navigator` | 10 状态状态机：IDLE→PRE_FLIGHT→ARMING→TAKEOFF→NAVIGATING→HOVERING→LANDING→LANDED，含 EMERGENCY / RETURNING |
| **safety_monitor** | `safety_monitor` | 独立安全节点，监控高度超限、通信超时、setpoint 流失效、模式异常、MAVROS 断连、位置跳变 |
| **logger** | `logger` | 终端格式化输出，实时显示飞行状态、位置、航点进度、偏差 |
| **experiment_recorder** | `experiment_recorder` | 自动实验记录：rosbag + CSV + 摘要，支持按日期/序号组织目录 |

## 状态机

```
                    ┌─────────┐
                    │  IDLE   │ ← RESET
                    └────┬────┘
                         │ START (需航点)
                    ┌────▼────┐
                    │PRE_FLIGHT│ (预飞行检查)
                    └────┬────┘
                         │ 检查通过
                    ┌────▼────┐
                    │ ARMING  │ (请求解锁)
                    └────┬────┘
                         │ 已解锁
                    ┌────▼────┐
                    │ TAKEOFF │ (预发布→OFFBOARD→起飞)
                    └────┬────┘
                         │ 到达起飞高度
                    ┌────▼────┐
            ┌───────│NAVIGATING│ (逐个航点)
            │       └────┬────┘
            │            │ 到达航点, 还有下一个
            │       ┌────▼────┐
            │       │ HOVERING│ (悬停 N 秒)
            │       └────┬────┘
            │            │ 悬停完成→下一个
            │            │ 最后一个航点到达
            │       ┌────▼────┐
            │       │ LANDING │
            │       └────┬────┘
            │            │ 高度 < 阈值
            │       ┌────▼────┐
            │       │ LANDED  │ ← RESET → IDLE
            │       └─────────┘
            │
            │  任意飞行状态 ──→ EMERGENCY (紧急停止)
            │  飞行中超时 ──→ RETURNING (返航→降落)
            └──────────────────┘
```

## 数据流

### 输入（Subscribe）

| 话题 | 消息类型 | 来源 | 用途 |
|------|---------|------|------|
| `mavros/state` | `mavros_msgs/State` | MAVROS | FCU 连接状态、解锁、模式 |
| `mavros/local_position/odom` | `nav_msgs/Odometry` | MAVROS | 当前位置、速度 |
| `uav/waypoints/current` | `geometry_msgs/PoseArray` | waypoint_manager | 当前航点列表（latched） |
| `uav/safety/alert` | `std_msgs/String` | safety_monitor | 安全告警（触发紧急/返航） |
| `uav/config/reload` | `std_msgs/String` | RViz panel | 配置热重载通知 |

### 输出（Publish）

| 话题 | 消息类型 | 频率 | 用途 |
|------|---------|------|------|
| `mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 20Hz | OFFBOARD 位置指令 |
| `uav/navigator/status` | `NavigatorStatus` | 10Hz | 状态、航点进度、位置、错误信息 |
| `uav/trajectory/real` | `nav_msgs/Path` | 10Hz | 实际飞行轨迹（FIFO ≤500 点） |
| `uav/experiment/metrics` | `ExperimentMetrics` | 10Hz | 实验指标（偏差、时间等） |

### 服务（Server）

| 服务 | 类型 | 说明 |
|------|------|------|
| `uav/navigator/command` | `NavigatorCommand` | 接收面板命令（START/PAUSE/CANCEL/LAND/RETURN_TO_HOME/EMERGENCY_STOP/RESET） |

## 消息定义

### NavigatorStatus

```
uint8 state                  # 状态机当前状态 (0-9)
uint8 current_waypoint_index # 当前目标航点索引 (0-based)
uint8 total_waypoints        # 航点总数
float32 current_x/y/z        # 当前位置 (m)
float32 target_x/y/z         # 目标位置 (m, 无目标时为 NaN)
bool is_armed                # 解锁状态
string current_mode          # PX4 飞行模式
string error_message         # 紧急原因（非紧急时为空）
```

### NavigatorCommand

```
# Request
string command   # START | PAUSE | CANCEL | LAND | RETURN_TO_HOME | EMERGENCY_STOP | RESET
---
# Response
bool success
string message
```

## 关键安全机制

- **OFFBOARD 预发布**：TAKEOFF 阶段先发布 N 次 setpoint（默认 100 次 @ 20Hz ≈ 5 秒），再请求 OFFBOARD 模式。PX4 要求持续 >2Hz 的 setpoint 流才能接受 OFFBOARD
- **setpoint 不间断**：由独立 `ros::Timer`（20Hz）统一发布 setpoint，与状态机逻辑解耦，避免任何代码路径阻塞 setpoint 流
- **高度保护**：odom 回调中实时检查高度，超限直接进 EMERGENCY
- **位置跳变检测**：相邻 odom 消息位移超过阈值（默认 2m）触发紧急
- **模式丢失检测**：飞行中模式被切出 OFFBOARD，容忍窗口（默认 2s）后触发紧急
- **超时保护**：每个状态都有独立超时（起飞 20s、降落 60s、紧急 120s）
- **RC 接管**：PX4 硬件级 RC 优先级高于 OFFBOARD，遥控器拨杆随时可接管

## 命令行操作

```bash
# 启动导航
rosservice call /uav/navigator/command "{command: 'START'}"

# 暂停（当前位置悬停）
rosservice call /uav/navigator/command "{command: 'PAUSE'}"

# 取消（降落）
rosservice call /uav/navigator/command "{command: 'CANCEL'}"

# 立即降落
rosservice call /uav/navigator/command "{command: 'LAND'}"

# 返航降落
rosservice call /uav/navigator/command "{command: 'RETURN_TO_HOME'}"

# 紧急停止（不可逆，需 RESET）
rosservice call /uav/navigator/command "{command: 'EMERGENCY_STOP'}"

# 从 EMERGENCY/LANDED 重置
rosservice call /uav/navigator/command "{command: 'RESET'}"

# 检查状态
rostopic echo /uav/navigator/status
```

## 配置

所有参数从 `config.yaml` 全局命名空间加载（与 panel/waypoint_manager 共享），支持运行时热重载：

```bash
# 修改 config.yaml 后在 RViz 面板点击"从文件加载"
# 或命令行：
rosparam load /home/groundstation/catkin_ws/config.yaml
rostopic pub /uav/config/reload std_msgs/String "data: 'manual reload'"
```

关键参数（详见 `config.yaml`）：

| 参数路径 | 默认值 | 说明 |
|---------|--------|------|
| `flight_defaults/takeoff_height` | 1.0 | 起飞高度 (m) |
| `flight_defaults/hover_duration` | 5.0 | 航点悬停时间 (s) |
| `flight_defaults/setpoint_rate` | 20.0 | setpoint 发布频率 (Hz) |
| `waypoint/reach_threshold_xy` | 0.15 | 水平到达阈值 (m) |
| `waypoint/reach_threshold_z` | 0.2 | 垂直到达阈值 (m) |
| `safety/max_height_limit` | 2.0 | 最大高度限制 (m) |
| `safety/communication_timeout` | 5.0 | 通信超时 (s) |
| `mode/offboard_timeout` | 8.0 | OFFBOARD 模式切换超时 (s) |
| `experiment/max_real_path_points` | 500 | real_path FIFO 上限 |

## 依赖

- MAVROS（`mavros_msgs`）
- PX4 飞控（OFFBOARD 模式支持）
- `uav_waypoint_manager`（航点数据源）
