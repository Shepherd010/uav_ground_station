# 无人机地面站航点导航系统

## 系统概述

本系统是一套面向 PX4 飞控无人机地面站的完整航点导航解决方案。采用模块化设计，实现从航点标注到飞行执行、轨迹对比、实验记录的全流程自动化控制。

**核心特性：**
- 完全配置化，所有参数从 YAML 配置文件读取，禁止硬编码
- 状态机驱动的导航核心，所有状态转换可追溯
- 独立安全监控节点，具有最高优先级中断权
- 航点持久化存储（XML），支持离线编辑和复用
- 增强版 RViz 面板，集成航点规划、系统控制、一键操作、实时日志
- Logger 节点，终端格式化输出飞机状态、导航信息和轨迹偏差
- 实验记录节点，自动记录 rosbag 和 CSV 指标
- 规划轨迹与实际轨迹对比可视化

**适用场景：**
- 地面站（本机）运行 roscore，无人机通过 WiFi 接入同一 ROS 网络
- 无人机通过 DLIO 进行环境感知和定位
- 通过 MAVROS 与 PX4 飞控通信，实现 OFFBOARD 航点导航

---

## 系统架构

### 模块划分

```
┌─────────────────────────────────────────────────────────────┐
│                        地面站 (Ground Station)                 │
│  ┌─────────────────┐  ┌─────────────────┐                    │
│  │  rviz_waypoint  │  │uav_waypoint_    │                    │
│  │  _panel         │  │  manager        │                    │
│  │  (RViz插件)      │  │  (航点管理)      │                    │
│  └────────┬────────┘  └────────┬────────┘                    │
│           │                    │                             │
│           │  /uav/waypoints    │  /uav/waypoints             │
│           │  /input            │  /current                   │
│           │                    │                             │
│  ┌────────┴────────────────────┴─────────────────────────┐  │
│  │                    uav_navigator                         │  │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────────┐ │  │
│  │  │ navigator│  │safety_   │  │  logger  │  │experiment│  │
│  │  │(导航核心) │  │monitor   │  │ (日志)   │  │recorder  │  │
│  │  └──────────┘  └──────────┘  └──────────┘  └────────┘ │  │
│  └────────────────────────┬─────────────────────────────────┘  │
│                           │                                    │
│  /mavros/setpoint_        │  /mavros/state                     │
│  position/local           │  /mavros/local_position/odom       │
│                           │                                    │
└───────────────────────────┼────────────────────────────────────┘
                            │
                    ┌───────┴───────┐
                    │  无人机上位机   │
                    │  DLIO + MAVROS│
                    │  + PX4飞控    │
                    └───────────────┘
```

### 模块说明

| 模块 | 包名 | 节点 | 功能 |
|------|------|------|------|
| RViz 航点面板 | `rviz_waypoint_panel` | (插件) | 航点规划、可视化、编辑、保存、加载、系统控制、一键操作 |
| 航点管理器 | `uav_waypoint_manager` | `waypoint_manager` | 航点保存/加载 XML、验证、发布 |
| 导航核心 | `uav_navigator` | `navigator` | 状态机驱动的 MAVROS 导航控制、轨迹发布、偏差计算 |
| 安全监控 | `uav_navigator` | `safety_monitor` | 通信超时、高度超限、setpoint 流、模式一致性、位置跳变监控 |
| 日志节点 | `uav_navigator` | `logger` | 格式化终端输出飞机状态、导航信息、轨迹偏差 |
| 实验记录 | `uav_navigator` | `experiment_recorder` | 自动记录 rosbag 和 CSV 实验指标 |

### 数据流

**航点规划与导航流程：**
1. 用户在 RViz 中点击"2D Nav Goal"标注规划点
2. 面板显示规划点（橙色球体），用户可删除、编辑
3. 点击"连接轨迹"生成规划轨迹（橙色线）
4. 点击"保存航点"将航点发布到 navigator，面板通过 `NavigatorStatus.total_waypoints` 确认收到
5. 点击"发布任务"开始导航
6. 导航核心自动执行：解锁 → OFFBOARD → 起飞 → 依次执行航点 → 降落
7. 同时发布 `/uav/trajectory/planned`（绿色）和 `/uav/trajectory/real`（红色）用于对比
8. 实验记录节点自动记录数据到 `/home/groundstation/experiments/`

### 状态机

```
IDLE → PRE_FLIGHT → ARMING → TAKEOFF → NAVIGATING → HOVERING → LANDING → LANDED
```

异常转换：
- 任何状态 ──EMERGENCY_STOP──→ EMERGENCY
- 任何状态 ──通信超时/高度超限/位置跳变/模式丢失──→ EMERGENCY 或 RETURNING → LANDING

---

## 目录结构

```
catkin_ws/
├── src/
│   ├── uav_navigator/              # 导航核心 + 安全监控 + 日志 + 实验记录
│   │   ├── src/
│   │   │   ├── navigator.cpp
│   │   │   ├── safety_monitor.cpp
│   │   │   ├── logger.cpp
│   │   │   └── experiment_recorder.cpp
│   │   ├── msg/NavigatorStatus.msg
│   │   ├── msg/ExperimentMetrics.msg
│   │   ├── srv/NavigatorCommand.srv
│   │   ├── config/navigator_config.yaml
│   │   ├── launch/ground_station.launch
│   │   └── test/                    # 自动化测试
│   ├── uav_waypoint_manager/       # 航点管理
│   │   ├── src/waypoint_manager.cpp
│   │   ├── srv/LoadWaypoints.srv
│   │   ├── srv/SaveWaypoints.srv
│   │   ├── srv/ClearWaypoints.srv
│   │   └── config/manager_config.yaml
│   ├── rviz_waypoint_panel/       # 增强版 RViz 插件
│   │   ├── src/waypoint_panel.cpp
│   │   ├── src/waypoint_panel.h
│   │   ├── config/
│   │   │   ├── panel_config.yaml
│   │   │   └── uav_navigation.rviz
│   │   ├── launch/rviz_ground_station.launch
│   │   └── plugin_description.xml
│   └── rviz_navi_multi_goals_pub_plugin/  # 原插件（备份）
├── uav_hover/src/hover/            # 原 hover 包（备份）
├── scripts/                         # 启动脚本
│   ├── start_ground_station.sh
│   ├── start_rviz.sh
│   ├── start_mission.sh
│   ├── test_sitl.sh
│   ├── test_sitl.py
│   ├── test_integration_mock.sh
│   ├── test_integration_mock.py
│   ├── test_scripts.sh
│   └── mock_mavros_node.py
├── uav_navigation.rviz              # 完整 RViz 配置（集成导航）
├── README.md
├── CHANGELOG.md
└── CLAUDE.md
```

---

## 安装与编译

### 前提条件

- ROS Noetic
- MAVROS（`sudo apt install ros-noetic-mavros ros-noetic-mavros-extras`）
- Qt5（RViz 插件需要）
- DLIO（无人机上位机）

### 编译

```bash
cd /home/groundstation/catkin_ws

catkin build
# 或
catkin_make

source devel/setup.bash
```

---

## 使用流程

### 默认 ROS 网络配置

| 角色 | ROS_MASTER_URI | ROS_IP | ROS_HOSTNAME |
|---|---|---|---|
| 地面站 | `http://192.168.31.116:11311` | `192.168.31.116` | （未设置，可选） |
| 无人机上位机 | `http://192.168.31.116:11311` | `192.168.31.180` | `192.168.31.180` |

可以通过环境变量覆盖默认值。启动脚本中 `ROS_HOSTNAME` 仅在用户已设置时才会导出，避免空字符串导致 ROS 节点解析失败。

### 1. 启动无人机机载链路

**执行位置：** 无人机上位机（SSH 远程）

```bash
ssh uav@192.168.31.180
~/uav_scripts/start_full.sh
```

机载链路会启动 Livox 驱动、DLIO SLAM、MAVROS、DLIO-MAVROS 桥接节点。

**检查 MAVROS 连接：**
```bash
rostopic echo /mavros/state
# 期望看到 connected: True
```

### 2. 启动地面站核心

**执行位置：** 地面站（本机）

```bash
./scripts/start_ground_station.sh
```

或手动：
```bash
roscore
roslaunch uav_navigator ground_station.launch
```

**启动内容：** navigator、safety_monitor、waypoint_manager、logger、experiment_recorder。

### 3. 启动可视化与航点面板

**执行位置：** 地面站（本机）

```bash
./scripts/start_rviz.sh
```

或手动：
```bash
roslaunch rviz_waypoint_panel rviz_ground_station.launch
```

**RViz 配置：**
1. 加载集成配置文件：`uav_navigation.rviz`
2. 航点面板插件已自动添加：`Panels -> UAV Waypoint Navigation`
3. 配置 2D Nav Goal 工具的话题为 `move_base_simple/goal`（默认）
4. 默认只保留两个面板：`Displays` 和 `UAV Waypoint Navigation`，其他面板（Selection、Tool Properties、Views、Time）已移除，以节省侧栏空间

### 4. 航点规划与保存

**面板 UI 功能：**

**飞行状态栏（系统状态 + 导航状态合并）：**
- 状态指示灯与状态文字（颜色编码：绿=运行中，橙=等待，红=紧急）
- MAVROS 连接状态（绿/红指示灯）
- 解锁状态（已解锁/未解锁）
- 飞行模式（OFFBOARD、LOITER 等）
- 航点进度条与航点计数
- 当前位置/目标位置显示

**配置参数区（默认折叠）：**
- 点击标题旁的复选框展开/折叠
- 显示当前加载的配置文件内容
- 支持从文件重新加载 `config.yaml`

**任务控制区（按生命周期分组）：**

- **地面站节点**
  - 🚀 **启动地面站** — 启动 navigator / safety_monitor / waypoint_manager / logger / experiment_recorder
  - ⏹ **停止地面站** — 停止所有地面站核心节点

- **任务执行**
  - 📂 **加载默认任务** — 从 `/home/groundstation/waypoints.xml` 加载航点并执行
  - ▶ **执行当前航点** — 发布面板当前航点并开始导航
  - ⏸ **暂停任务** — 当前位置悬停
  - ⏹ **取消任务** — 取消当前任务并降落

- **飞行阶段**
  - 🛫 **一键起飞** — 发布当前航点并启动导航序列（解锁→起飞→航点）
  - 🛬 **一键降落** — 立即发送降落命令
  - 🏠 **返航降落** — 返回起飞点并降落

- **安全与重置**
  - 🔄 **重置状态机** — 从 EMERGENCY 或 LANDED 状态恢复到 IDLE
  - 🛑 **紧急停止** — 触发紧急停止（带确认对话框）

> **遥控器优先：** 所有飞行相关按钮的 tooltip 均提示"遥控器拨杆可随时接管，优先级最高"。

**航点规划区：**
- 🔗 **连接** — 将规划点连接成轨迹
- 🗑 **删除** — 删除选中的规划点
- 🧹 **清除** — 清除所有规划点和航点
- 💾 **保存** — 发布航点到 navigator 并确认收到
- ▶ **发布** — 开始导航
- 状态显示：PLANNING / CONNECTED / SAVED / NAVIGATING

**航点表格：**
- x, y, z, yaw 列（可编辑）
- 删除 / 上移 / 下移 / 清除
- 保存到 XML / 从 XML 加载 / 发布

**操作日志：**
- 带时间戳的 HTML 格式日志（绿=INFO，橙=WARN，红=ERROR）

**操作步骤：**
1. 在 RViz 中点击"2D Nav Goal"工具，在地图上点击标注规划点
2. 规划点自动填充到面板表格，地图上显示橙色球体标记
3. 点击"连接轨迹"，地图上显示橙色规划轨迹
4. 在表格中编辑航点 z（高度）和 yaw 值
5. 点击"保存航点"，等待状态变为 SAVED（navigator 已确认收到）
6. 点击"保存航点"（文件按钮）保存到 XML，或"加载航点"从 XML 读取

**XML 格式示例：**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<waypoints>
  <metadata>
    <created>2026-06-24 10:30:00</created>
    <frame_id>map</frame_id>
    <count>3</count>
  </metadata>
  <waypoint id="1">
    <x>1.0</x>
    <y>0.0</y>
    <z>2.0</z>
    <yaw>0.0</yaw>
  </waypoint>
  <waypoint id="2">
    <x>2.0</x>
    <y>1.0</y>
    <z>2.0</z>
    <yaw>1.57</yaw>
  </waypoint>
  <waypoint id="3">
    <x>0.0</x>
    <y>0.0</y>
    <z>2.0</z>
    <yaw>0.0</yaw>
  </waypoint>
</waypoints>
```

### 5. 执行导航与轨迹对比

**方法 1：航点规划面板（推荐）**

在 RViz 面板中：
1. 使用 2D Nav Goal 打点
2. 点击"连接轨迹"
3. 点击"保存航点"等待确认
4. 点击"发布任务"开始导航

**方法 2：命令行**
```bash
# 加载航点
rosservice call /uav/waypoint_manager/load_waypoints "{file_path: '/home/groundstation/waypoints.xml'}"

# 开始导航
rosservice call /uav/navigator/command "{command: 'START'}"
```

**方法 3：任务脚本**
```bash
./scripts/start_mission.sh /home/groundstation/waypoints.xml
```

**导航过程自动执行：**
1. 预飞行检查（MAVROS 连接、位置数据、航点数据）
2. 请求解锁（arming）
3. 切换到 OFFBOARD 模式
4. 起飞到指定高度
5. 依次执行航点（到达→悬停→下一个）
6. 所有航点完成→降落

**轨迹对比：**
- `/uav/plan_maker/trajectory`：规划轨迹（橙色）
- `/uav/trajectory/planned`：执行规划轨迹（绿色）
- `/uav/trajectory/real`：实际飞行轨迹（红色）
- `/uav/experiment/metrics`：实时偏差指标

**实时监控：**
```bash
# 查看导航状态
rostopic echo /uav/navigator/status

# 查看当前位置
rostopic echo /mavros/local_position/odom

# 查看偏差指标
rostopic echo /uav/experiment/metrics
```

**日志节点输出示例：**
```
╔══════════════════════════════════════════════════════════════════════════════╗
║           UAV Navigator Logger - 无人机导航状态监控器                        ║
╠══════════════════════════════════════════════════════════════════════════════╣
║ 时间      | 状态  | 当前位置(x,y,z) | 目标位置(x,y,z) | 偏差  | 模式 | 解锁 ║
╚══════════════════════════════════════════════════════════════════════════════╝
[METRICS]  18:30:45 | deviation: 0.05 m | dxyz: (0.02, -0.01, 0.00)
[NAVIGATOR] 18:30:45 | state: NAVIGT | wp: 2/5 | pos: (1.50, 0.80, 2.00) | tgt: (2.00, 1.00, 2.00) | armed: ✓ | mode: OFFBOARD
```

### 6. 实验记录

实验记录节点 `experiment_recorder` 在 navigator 进入 PRE_FLIGHT 时自动开始记录，在 LANDED 或 EMERGENCY 时停止。

**记录内容：**
- `/uav/navigator/status`
- `/mavros/local_position/odom`
- `/mavros/setpoint_position/local`
- `/mavros/state`
- `/uav/safety/alert`
- `/uav/trajectory/planned`
- `/uav/trajectory/real`
- `/uav/experiment/metrics`

**输出位置：**
```
/home/groundstation/experiments/YYYY-MM-DD/experiment_NNN/
├── metadata.yaml      # 实验元数据
├── data.bag           # 原始 ROS bag
├── metrics.csv        # 处理后的指标
├── trajectory_real.csv # 真实轨迹
└── summary.txt        # 文本摘要
```

### 紧急控制

```bash
# 紧急停止（立即停止所有操作，切换到紧急状态）
rosservice call /uav/navigator/command "{command: 'EMERGENCY_STOP'}"

# 取消导航并降落
rosservice call /uav/navigator/command "{command: 'CANCEL'}"

# 返航（返回起飞点并降落）
rosservice call /uav/navigator/command "{command: 'RETURN_TO_HOME'}"

# 重置（从 EMERGENCY 或 LANDED 状态恢复）
rosservice call /uav/navigator/command "{command: 'RESET'}"
```

或在 RViz 面板中点击对应按钮。

---

## 配置说明

### 全局配置：`uav_navigator/config/navigator_config.yaml`

```yaml
# 参数作用域说明：
#   safety/max_height_limit: 安全保护上限，与航点 z 值无关
#   flight/takeoff_height: 默认起飞高度，没有航点 z 值时使用
#   waypoint/reach_threshold_*: 导航精度阈值

namespace: ""  # 不使用命名空间前缀，保持与机载 MAVROS 话题一致

topics:
  mavros_state: "mavros/state"
  local_position_odom: "mavros/local_position/odom"
  setpoint_position: "mavros/setpoint_position/local"
  arming_service: "mavros/cmd/arming"
  set_mode_service: "mavros/set_mode"
  waypoint_current: "uav/waypoints/current"
  navigator_status: "uav/navigator/status"
  navigator_command: "uav/navigator/command"
  safety_alert: "uav/safety/alert"
  planned_path: "uav/trajectory/planned"
  real_path: "uav/trajectory/real"
  metrics: "uav/experiment/metrics"

flight:
  takeoff_height: 1.0          # 起飞高度（米）
  takeoff_timeout: 20.0        # 起飞阶段超时时间（秒）
  hover_duration: 2.0          # 航点悬停时间（秒）
  setpoint_rate: 20.0          # setpoint 发布频率（Hz）
  offboard_pre_pub_count: 100  # 切入 OFFBOARD 前预发布数量
  landing_height_threshold: 0.15  # 降落完成高度阈值

waypoint:
  reach_threshold_xy: 0.15      # 水平到达阈值（米）
  reach_threshold_z: 0.2        # 垂直到达阈值（米）
  min_waypoint_spacing: 0.3     # 最小航点间距（米）

mode:
  offboard_timeout: 8.0         # OFFBOARD 模式切换超时（秒）
  mode_retry_interval: 5.0      # 模式请求重试间隔
  arm_retry_interval: 5.0       # 解锁请求重试间隔

safety:
  max_height_limit: 10.0         # 最大允许飞行高度（米）
  communication_timeout: 5.0     # 通信超时（秒）

offboard_safety:
  min_setpoint_rate_hz: 10.0        # 健康 setpoint 最小发布率
  setpoint_timeout_warn: 0.5        # setpoint 超时警告阈值
  setpoint_timeout_emergency: 1.0   # setpoint 超时触发紧急停止
  mode_mismatch_tolerance: 2.0      # 飞行中模式丢失容忍时间

position_safety:
  max_jump_distance: 2.0            # 最大允许位置跳变（米）
  jump_window: 0.1                  # 跳变检测窗口（秒）

experiment:
  output_dir: "/home/groundstation/experiments"  # 实验输出目录
  auto_record: true                              # 自动记录
  real_path_sample_interval: 0.1                 # 真实轨迹采样间隔
```

### 面板配置：`rviz_waypoint_panel/config/panel_config.yaml`

```yaml
panel:
  goal_topic: "move_base_simple/goal"
  marker_topic: "visualization_marker"
  waypoint_input_topic: "uav/waypoints/input"
  navigator_status_topic: "uav/navigator/status"
  mavros_state_topic: "mavros/state"
  odom_topic: "mavros/local_position/odom"
  nav_command_service: "uav/navigator/command"
  save_service: "uav/waypoint_manager/save_waypoints"
  load_service: "uav/waypoint_manager/load_waypoints"

  plan_maker:
    points_topic: "uav/plan_maker/points"
    trajectory_topic: "uav/plan_maker/trajectory"
    sphere_scale: 0.15
    color_r: 1.0
    color_g: 0.65
    color_b: 0.0
    color_a: 0.9
    trajectory_width: 0.05

  marker:
    arrow_scale_x: 0.6
    arrow_scale_y: 0.15
    arrow_scale_z: 0.15
    number_scale: 0.8
    number_offset_z: 0.6
    color_r: 1.0
    color_g: 0.84
    color_b: 0.0
    color_a: 1.0

  table:
    default_max_goals: 10

  spin_timer_ms: 100
```

---

## 服务接口

### 导航命令服务：`/uav/navigator/command`

**类型：** `uav_navigator/NavigatorCommand`

| 命令 | 说明 | 可用状态 |
|------|------|----------|
| `START` | 开始导航 | IDLE, LANDED |
| `PAUSE` | 暂停导航 | NAVIGATING |
| `CANCEL` | 取消导航并降落 | NAVIGATING, HOVERING, TAKEOFF |
| `LAND` | 立即降落 | 除 LANDED/LANDING 外的所有状态 |
| `RETURN_TO_HOME` | 返航 | NAVIGATING, HOVERING |
| `EMERGENCY_STOP` | 紧急停止 | 所有状态 |
| `RESET` | 重置状态机 | EMERGENCY, LANDED |

### 航点管理服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/uav/waypoint_manager/load_waypoints` | `LoadWaypoints` | 从 XML 加载航点 |
| `/uav/waypoint_manager/save_waypoints` | `SaveWaypoints` | 保存航点到 XML |
| `/uav/waypoint_manager/clear_waypoints` | `ClearWaypoints` | 清空航点 |

### 安全监控服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/uav/safety/emergency_stop` | `std_srvs/Trigger` | 触发紧急停止 |

---

## 安全机制

### 安全监控（safety_monitor）

安全监控是独立节点，即使导航核心崩溃也能触发保护：

1. **通信超时检测**：监控 navigator 状态更新时间
2. **高度超限检测**：监控 `mavros/local_position/odom`
3. **setpoint 流检测**：飞行中 setpoint 流停止触发告警
4. **模式一致性检测**：navigator 在 NAVIGATING 但 MAVROS 模式不是 OFFBOARD，持续超过阈值触发告警
5. **位置跳变检测**：0.1 秒内位置变化超过 2 米触发告警
6. **MAVROS 连接断开**：检测到 MAVROS 连接断开触发告警

### 导航核心安全保护

1. **OFFBOARD 超时**：超过 `offboard_timeout` 秒未切入 OFFBOARD，自动进入 LANDING
2. **解锁超时**：超过 `offboard_timeout` 秒未解锁，自动尝试上锁并返回 IDLE
3. **起飞超时**：起飞阶段超过 `takeoff_timeout` 秒未到达高度，自动进入 LANDING
4. **模式丢失保护**：飞行中模式被切出 OFFBOARD 超过 `mode_mismatch_tolerance` 秒，触发 EMERGENCY
5. **位置跳变保护**：检测到位置跳变触发 EMERGENCY
6. **降落超时**：降落阶段超过 60 秒未降落到阈值高度，强制进入 LANDED
7. **紧急状态超时**：紧急状态持续 2 分钟后自动转为 LANDED

### 遥控器接管

**RC（遥控器）拥有最高控制权，优先于所有地面站软件命令。**

PX4 原生支持遥控器接管，无需代码处理：

1. 在 OFFBOARD 模式下，拨动遥控器模式开关即可随时切出 OFFBOARD，夺回控制权
2. 即使地面站发送了紧急停止、返航或降落命令，只要遥控器在手动/自稳/定高等非 OFFBOARD 模式，飞控会立即响应遥控器
3. 因此在任何自动飞行阶段，操作员都应保持遥控器在手，并熟悉模式切换拨杆位置

**地面站中的体现：**
- RViz 面板所有飞行相关按钮的 tooltip 均提示"遥控器拨杆可随时接管，优先级最高"
- 启动地面站和启动任务时，操作日志会打印 RC 接管提示
- 安全监控检测到模式被切出 OFFBOARD 时，会先等待 `mode_mismatch_tolerance` 秒再触发告警，给 RC 接管留出合理窗口

---

## 自动化测试

### 1. 脚本稳健性测试

```bash
./scripts/test_scripts.sh
```

### 2. PX4 SITL 端到端测试

```bash
./scripts/test_sitl.sh
# 或
python3 scripts/test_sitl.py
```

在本地启动 PX4 SITL + Gazebo + MAVROS，验证完整导航链路。

**前提条件：**
- PX4-Autopilot 位于 `~/PX4-Autopilot`
- 已安装 GeographicLib 数据集：
  ```bash
  echo "1234" | sudo -S /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
  ```

### 3. Mock 集成测试

```bash
python3 scripts/test_integration_mock.py
```

无需 PX4 SITL，使用 mock MAVROS 节点验证地面站链路。

---

## 故障排查

### 启动日志出现 "Couldn't find an AF_INET address for []"

**现象：**
```
invalid ROS_HOSTNAME (an empty string)
Couldn't find an AF_INET address for []: Name or service not known
Error in XmlRpcClient::doConnect: Could not connect to server (Invalid argument).
```

**原因：** `ROS_HOSTNAME` 被设置为空字符串，ROS 节点无法解析自身地址。

**解决：** 启动脚本已修复，不再导出空的 `ROS_HOSTNAME`。如需自定义主机名，请在运行脚本前设置：
```bash
export ROS_HOSTNAME=groundstation
./scripts/start_ground_station.sh
```

### MAVROS 连接不上

```bash
# 在地面站检查话题
rostopic echo /mavros/state

# 在机载端检查串口连接
ls -l /dev/ttyUSB*

# 检查 DLIO 是否正常运行
rostopic echo /robot/dlio/odom_node/odom
```

### 插件不显示航点标记

1. 确保 RViz 中添加了 Marker / MarkerArray 显示
2. 确保 `move_base_simple/goal` 话题有数据：`rostopic echo /move_base_simple/goal`
3. 检查面板配置：`rosparam get /panel`

### 航点不发布

1. 检查 waypoint_manager 是否运行：`rosnode list | grep waypoint_manager`
2. 检查话题：`rostopic list | grep waypoint`
3. 检查话题是否有数据：`rostopic echo /uav/waypoints/current`

### 导航不启动

1. 检查 navigator 状态：`rostopic echo /uav/navigator/status`
2. 检查错误信息：`rostopic echo /uav/navigator/status | grep error_message`
3. 检查命令是否发送成功：`rosservice call /uav/navigator/command "{command: 'START'}"`

### 实验记录不生成

1. 检查 experiment_recorder 节点是否运行：`rosnode list | grep experiment`
2. 检查输出目录权限：`ls -ld /home/groundstation/experiments`
3. 手动触发记录：`rostopic pub /uav/experiment/record std_msgs/Bool "{data: true}"`

### 编译错误

```bash
# 清理编译缓存
catkin clean
# 或
catkin_make clean

# 重新编译
catkin build
source devel/setup.bash
```

---

## 修改日志

详见 [CHANGELOG.md](CHANGELOG.md)

---

## 许可证

MIT License
