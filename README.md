# 无人机地面站航点导航系统

面向 PX4 飞控的全自动航点导航地面站，基于 ROS Noetic + MAVROS + RViz。模块化设计，覆盖航点标注→规划→验证→执行→监控→记录的全流程。

**版本：** v2.8.1 | **许可证：** MIT

---

## 快速开始

```bash
# 1. 机载端（SSH 到无人机）
ssh uav@192.168.31.180
~/uav_scripts/start_full.sh

# 2. 地面站核心（本机）
cd /home/groundstation/catkin_ws_copy
./scripts/start_ground_station.sh

# 3. 可视化面板（本机，新终端）
./scripts/start_rviz.sh

# 4. 执行任务
./scripts/start_mission.sh /home/groundstation/waypoints.xml
```

## 系统架构

```
┌──────────────────────────────────────────────────────────────────────┐
│                       地面站 (Ground Station)                         │
│                                                                       │
│  ┌─────────────────────┐  ┌──────────────────────┐                   │
│  │  rviz_waypoint_panel │  │ uav_waypoint_manager  │                   │
│  │  (RViz Qt5 面板)     │  │ (XML 持久化 + 验证)   │                   │
│  │  · 航点标注/编辑     │  │ · validate/load/save │                   │
│  │  · 可视化 (MarkerArr)│  │ · per-waypoint params│                   │
│  │  · 导航控制          │  └──────────┬───────────┘                   │
│  │  · 系统管理          │             │                               │
│  └──────────┬──────────┘             │                               │
│             │ uav/waypoints/input    │ uav/waypoints/current         │
│             │ uav/waypoints/params   │ uav/waypoints/params_loaded   │
│             │                        │                               │
│  ┌──────────┴────────────────────────┴────────────────────────────┐ │
│  │                       uav_navigator                              │ │
│  │  ┌───────────┐ ┌──────────────┐ ┌────────┐ ┌─────────────────┐ │ │
│  │  │ navigator │ │safety_monitor│ │ logger │ │experiment_      │ │ │
│  │  │ 10状态机  │ │ 独立安全节点 │ │终端输出│ │  recorder       │ │ │
│  │  │ OFFBOARD  │ │ 高度/通信/   │ │格式化  │ │  rosbag+CSV     │ │ │
│  │  │ 20Hz setpt│ │ 跳变/模式    │ │状态表  │ │  自动记录       │ │ │
│  │  └───────────┘ └──────────────┘ └────────┘ └─────────────────┘ │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│                                                                       │
│  统一配置: config.yaml → 全局命名空间 → 所有节点共享 + 热重载         │
└───────────────────────────────────┬──────────────────────────────────┘
                                    │ MAVROS (mavros/state, setpoint, odom)
                            ┌───────┴───────┐
                            │  无人机上位机   │
                            │  DLIO + MAVROS │
                            │  + PX4 飞控    │
                            └───────────────┘
```

### 四包职责

| 包 | 节点 | 说明 |
|---|------|------|
| **uav_navigator** | `navigator` | 10 状态状态机，OFFBOARD 模式控制，20Hz setpoint 流 |
| | `safety_monitor` | 独立安全节点：高度/通信/位置跳变/模式异常/MAVROS 断连 |
| | `logger` | 终端格式化输出：状态、位置、航点进度、轨迹偏差 |
| | `experiment_recorder` | 自动记录 rosbag + CSV + 摘要（按日期/序号组织） |
| **uav_waypoint_manager** | `waypoint_manager` | XML 持久化，航点验证（坐标/重复/间距/高度），路径遍历保护 |
| **rviz_waypoint_panel** | (RViz 插件) | Qt5 面板：航点标注、编辑、可视化、导航控制、系统管理、配置热重载 |

## 状态机

```
                    ┌─────────┐
                    │  IDLE   │ ← RESET
                    └────┬────┘
                         │ START（需航点数据）
                    ┌────▼────┐
                    │PRE_FLIGHT│
                    └────┬────┘
                         │ 检查通过（MAVROS/位置/航点）
                    ┌────▼────┐
                    │ ARMING  │
                    └────┬────┘
                         │ 解锁成功
                    ┌────▼────┐
                    │ TAKEOFF │（预发布 100 次→OFFBOARD→起飞）
                    └────┬────┘
                         │ 到达起飞高度
                    ┌────▼────┐
            ┌───────│NAVIGATING│（逐航点飞行）
            │       └────┬────┘
            │            │ 到达 → 还有下一个
            │       ┌────▼────┐
            │       │ HOVERING│（悬停 N 秒）
            │       └────┬────┘
            │            │ 最后一个航点到达
            │       ┌────▼────┐
            │       │ LANDING │
            │       └────┬────┘
            │            │ 高度 < 阈值
            │       ┌────▼────┐
            │       │ LANDED  │ ← RESET → IDLE
            │       └─────────┘
            │
            │  任意状态 ──EMERGENCY_STOP/高度超限/位置跳变──→ EMERGENCY
            │  飞行中 ──通信超时──→ RETURNING → LANDING
            └──────────────────────────────────┘
```

## 完整数据流

```
Panel (RViz)                Waypoint Manager              Navigator
───────────                 ────────────────              ─────────

2D Nav Goal 打点
  │
  ▼
receiveGoal()
  │
  ├─→ addPlanMakerPoint() → publishPlanMakerMarkers()
  │     └→ /uav/plan_maker/points (MarkerArray) → RViz 渲染
  │         · SPHERE (位置球)
  │         · ARROW (yaw 方向)
  │         · TEXT_VIEW_FACING (编号)
  │
  ├─→ addWaypointToTable() → 表格 6 列
  │
  └─→ [🔗连接] → publishPlanTrajectory()
        └→ /uav/plan_maker/trajectory (Path) → RViz 黄色轨迹线

[💾保存] → savePlanWaypoints()
  ├─→ waypoint_pub_.publish() ──→ uav/waypoints/input ──→ waypointsCallback()
  │     (PoseArray, latched)                                │
  │                                                        ├→ validateWaypoints()
  │                                                        │   · NaN/Inf 硬拒绝
  │                                                        │   · 高度/间距/重复 软警告
  │                                                        │
  ├─→ waypoint_params_pub_ ────→ uav/waypoints/params ──→ waypointParamsCallback()
  │     (Float64MultiArray)                                 │
  │                                                        ├→ waypoints_pub_.publish()
  │                                                        │   └→ uav/waypoints/current ──→ navigator.waypointsCallback()
  │                                                        │       (latched PoseArray)
  │                                                        │
  │                                                        └→ waypoint_params_current_pub_.publish()
  │                                                            └→ uav/waypoints/params_loaded ──→ panel.receiveWaypointParams()
  │                                                                (更新表格 hover/speed 列)

[▶发布] → publishPlanTask() → nav_command START
  └→ /uav/navigator/command ──→ navigator.commandCallback()
                                   │
                                   ├→ PRE_FLIGHT → ARMING → TAKEOFF
                                   ├→ NAVIGATING (20Hz setpoint + 10Hz status)
                                   │
                                   └→ publishStatus()
                                       └→ /uav/navigator/status ──→ panel.receiveNavStatus()
                                           (状态显示 + 进度条 + marker 颜色编码)

实时回显:
  /mavros/state ────────────────→ panel MAVROS 连接/解锁/模式显示
  /mavros/local_position/odom ──→ panel 位置显示
  /uav/trajectory/real ─────────→ RViz 红色实际轨迹 (FIFO ≤500 点)

加载已有航点:
  [📂加载] → loadWaypoints()
    └→ load_waypoints srv CALL → waypoint_manager.loadWaypointsCallback()
        ←── response.waypoints (PoseArray 直接返回，不用 waitForMessage)
    └→ 填充 plan_maker_points_ + 表格 + publishPlanMakerMarkers()
    └→ 自动连接轨迹 (≥2 点)
```

## 话题速查

| 话题 | 类型 | 方向 | 频率 | 用途 |
|------|------|------|------|------|
| `uav/waypoints/input` | PoseArray | panel→manager | 按需(latched) | 面板发布航点 |
| `uav/waypoints/params` | Float64MultiArray | panel→manager | 按需(latched) | per-waypoint 悬停/速度 |
| `uav/waypoints/current` | PoseArray | manager→navigator | 按需(latched) | 验证后的航点 |
| `uav/waypoints/params_loaded` | Float64MultiArray | manager→panel | 按需(latched) | 反馈 per-waypoint 参数 |
| `uav/navigator/status` | NavigatorStatus | navigator→panel | 10Hz | 状态/位置/航点进度/错误 |
| `uav/navigator/command` | NavigatorCommand | panel→navigator | 按需(service) | START/PAUSE/CANCEL/... |
| `uav/safety/alert` | String | safety_monitor→navigator | 按需 | 安全告警 |
| `uav/safety/heartbeat` | String | safety_monitor→* | 1Hz | 安全节点心跳 |
| `uav/plan_maker/points` | MarkerArray | panel→RViz | ~3Hz 刷新 | 航点可视化（球+箭头+编号） |
| `uav/plan_maker/trajectory` | Path | panel→RViz | 按需 | 规划轨迹（黄色） |
| `uav/trajectory/real` | Path | navigator→RViz | 10Hz | 实际飞行轨迹（红色） |
| `uav/config/reload` | String | panel→navigator+safety | 按需 | 配置热重载通知 |
| `uav/config/loaded` | String | panel→logger | 按需 | 配置加载成功事件 |
| `uav/experiment/metrics` | ExperimentMetrics | navigator→recorder | 10Hz | 偏差等实验指标 |
| `mavros/state` | State | MAVROS→* | ~10Hz | FCU 连接/解锁/模式 |
| `mavros/local_position/odom` | Odometry | MAVROS→* | ~30Hz | 位置/速度 |
| `mavros/setpoint_position/local` | PoseStamped | navigator→MAVROS | 20Hz | OFFBOARD 位置指令 |

## 服务速查

| 服务 | 请求 | 响应 | 可用状态 |
|------|------|------|----------|
| `/uav/navigator/command` | `command: START` | `success, message` | IDLE, LANDED |
| | `command: PAUSE` | | NAVIGATING |
| | `command: CANCEL` | | NAVIGATING, HOVERING, TAKEOFF |
| | `command: LAND` | | 除 LANDED/LANDING 外 |
| | `command: RETURN_TO_HOME` | | NAVIGATING, HOVERING |
| | `command: EMERGENCY_STOP` | | 所有状态 |
| | `command: RESET` | | EMERGENCY, LANDED |
| `/uav/waypoint_manager/load_waypoints` | `file_path` | `success, message, waypoint_count, waypoints(PoseArray)` | — |
| `/uav/waypoint_manager/save_waypoints` | `file_path` | `success, message` | — |
| `/uav/waypoint_manager/clear_waypoints` | — | `success, message` | — |
| `/uav/safety/emergency_stop` | — | `success, message` | — |

## 航点可视化

### Marker 系统（`/uav/plan_maker/points`）

每个航点发布 3 种 marker：

| Marker | namespace | 视觉 | 显示内容 |
|--------|-----------|------|---------|
| **SPHERE** | `uav_plan_maker_points` | 彩色球体 | 精确位置标记 |
| **ARROW** | `uav_plan_maker_arrows` | 方向箭头 | yaw 朝向（机头方向） |
| **TEXT_VIEW_FACING** | `uav_plan_maker_numbers` | 白色数字 | 航点编号（始终面朝相机） |

### 导航进度颜色编码

| 状态 | 颜色 | 触发条件 |
|------|------|---------|
| 🟢 已到达 | 绿色 `#4CAF50` α=0.7 | `index < current_waypoint_idx` |
| 🟠 当前目标 | 亮橙色 `#FF9800` α=1.0 **放大 1.5x** | `index == current_waypoint_idx` |
| 🔵 待飞行 | 蓝色 `#2196F3` α=0.8 | `index > current_waypoint_idx` |
| 🟠 规划中 | 橙色 α=0.9 | 未开始导航 |

- **生命周期**：marker `lifetime=1.0s`，面板每 ~300ms 定时刷新。面板崩溃后 marker 1 秒内自动消失
- **选中高亮**：选中航点绿色通道提升至 1.0，尺寸放大 1.3x

### 轨迹线

| 话题 | 颜色 | 线宽 | 含义 |
|------|------|------|------|
| `uav/plan_maker/trajectory` | 🟡 黄色 `#FFC107` | 0.05 | 规划轨迹（用户编辑的航点连线） |
| `uav/trajectory/real` | 🔴 红色 `#F44336` | 0.08 | 实际飞行轨迹（FIFO，默认最多 500 点） |

## XML 航点格式

```xml
<?xml version="1.0" encoding="UTF-8"?>
<waypoints>
  <metadata>
    <created>2026-07-17 10:30:00</created>
    <frame_id>map</frame_id>
    <count>3</count>
  </metadata>
  <waypoint id="1">
    <x>1.0</x>
    <y>0.0</y>
    <z>2.0</z>
    <yaw>0.0</yaw>
    <hover_time>5.0</hover_time>     <!-- 悬停时间（秒），可选，默认 5.0 -->
    <speed>2.0</speed>               <!-- 飞行速度（m/s），可选，默认 2.0 -->
    <orientation>                    <!-- 四元数（从 yaw 自动计算） -->
      <x>0.0</x><y>0.0</y><z>0.0</z><w>1.0</w>
    </orientation>
  </waypoint>
  <waypoint id="2">
    <x>3.0</x><y>0.0</y><z>2.0</z><yaw>0.0</yaw>
    <hover_time>3.0</hover_time>
    <speed>2.0</speed>
    <orientation>
      <x>0.0</x><y>0.0</y><z>0.0</z><w>1.0</w>
    </orientation>
  </waypoint>
</waypoints>
```

## 配置管理

### 统一配置文件：`config.yaml`

所有参数集中在 workspace 根目录的 `config.yaml`，加载到 ROS 全局命名空间 `/`。四个节点 + 面板共享同一份配置，修改后通过面板"📂 从文件加载"即可热重载（无需重启节点）。

```
config.yaml 参数分类：
  第一类 infrastructure  — 话题/服务名（极少改动）
  第二类 safety/mode     — 安全阈值（面板加载后立即生效）
  第三类 flight_defaults — 默认飞行参数（可被每航点覆盖）
  第四类 waypoint        — 导航精度参数
  第五类 validation      — 航点验证参数
  第六类 panel           — RViz 面板可视化参数
  第七类 logger          — 日志配置
  第八类 experiment      — 实验记录配置
```

### 配置热重载流程

```
1. 编辑 config.yaml
2. RViz 面板 → 配置参数（展开折叠区）→ 📂 从文件加载 → 选择 config.yaml
3. 后台执行:
   a. rosparam load config.yaml (QProcess 异步)
   b. panel.loadConfig() — 更新面板本地参数
   c. publish /uav/config/reload → navigator.loadConfig() + safety_monitor.loadConfig()
4. 参数立即生效，无需重启
```

### 关键参数速查

| 参数路径 | 默认 | 说明 |
|---------|------|------|
| `flight_defaults/takeoff_height` | 1.0 m | 起飞高度 |
| `flight_defaults/hover_duration` | 5.0 s | 每航点悬停时间（可被 XML 覆盖） |
| `flight_defaults/travel_speed` | 2.0 m/s | 飞行速度（可被 XML 覆盖） |
| `flight_defaults/setpoint_rate` | 20.0 Hz | setpoint 发布频率 |
| `waypoint/reach_threshold_xy` | 0.15 m | 水平到达判定阈值 |
| `waypoint/reach_threshold_z` | 0.2 m | 垂直到达判定阈值 |
| `safety/max_height_limit` | 2.0 m | 硬性高度保护上限 |
| `safety/communication_timeout` | 5.0 s | 导航器通信超时 |
| `validation/duplicate_threshold` | 0.01 m | 重复航点判定距离 |
| `experiment/max_real_path_points` | 500 | 实际轨迹 FIFO 上限 |
| `panel/spin_timer_ms` | 100 ms | 面板 UI 刷新间隔 |

## 安全机制

### 多层防护

```
第 1 层: PX4 硬件 RC 接管（遥控器拨杆随时切出 OFFBOARD）
第 2 层: safety_monitor 独立节点（navigator 崩溃也能触发保护）
第 3 层: navigator 内置安全检查（高度/跳变/模式丢失/超时）
```

### safety_monitor 检测项

| 检测项 | 条件 | 动作 |
|--------|------|------|
| 高度超限 | z > max_height_limit | publishAlert HEIGHT_EXCEEDED |
| 导航器通信超时 | 无 status > communication_timeout | publishAlert NAVIGATOR_TIMEOUT |
| setpoint 流失效 | 飞行中无 setpoint > setpoint_timeout | publishAlert SETPOINT_TIMEOUT |
| 模式丢失 | 飞行中 mode≠OFFBOARD > tolerance | publishAlert MODE_MISMATCH |
| MAVROS 断连 | mavros connected=false | publishAlert MAVROS_DISCONNECTED |
| MAVROS 超时 | 无 mavros state > communication_timeout | publishAlert MAVROS_TIMEOUT |
| 位置跳变 | 位移 > jump_distance / window | publishAlert POSITION_JUMP |
| 心跳 | 每秒发布 uav/safety/heartbeat | 证明节点存活 |

### 导航器内置保护

| 保护项 | 超时 | 动作 |
|--------|------|------|
| 起飞超时 | 20 s | → LANDING |
| 降落超时 | 60 s | → LANDED |
| 紧急超时 | 120 s | → LANDED |
| 返航超时 | takeoff_timeout × 3 | → LANDING |
| 模式丢失 | 2 s 容忍 | → EMERGENCY |

## RViz 性能配置

为解决点云无限累积导致的卡顿，`uav_navigation.rviz` 已优化：

| 参数 | 旧值 | 新值 | 效果 |
|------|------|------|------|
| Dense Map Decay Time | 9999s（永不过期） | **20s** | 点云自动淘汰，GPU 负载大幅降低 |
| Dense Map Queue Size | 10 | **5** | 内存缓冲减半 |
| DLIO Trajectory Style | Billboards | **Lines** | 路径渲染提速 3-5x |
| Waypoint Markers (旧) | active | **removed** | 消除遗留僵尸渲染管线 |
| Planned Trajectory | active | **removed** | 消除废弃话题订阅 |

> **说明：** 点云 Decay Time 可在 RViz 界面 Displays → Point Clouds → Dense Map → Decay Time 中实时调整。增大可看更大范围地图，减小可进一步提升帧率。

## 面板按钮与命令行对应

| 面板操作 | 等价命令 | 说明 |
|---------|---------|------|
| ▶ 开始任务 | `rosservice call /uav/navigator/command "{command: 'START'}"` | 发布航点 + 启动导航 |
| ⏸ 悬停 | `rosservice call /uav/navigator/command "{command: 'PAUSE'}"` | 保持当前位置 |
| 🛬 降落 | `rosservice call /uav/navigator/command "{command: 'LAND'}"` | 立即着陆 |
| 🏠 返航 | `rosservice call /uav/navigator/command "{command: 'RETURN_TO_HOME'}"` | 返回起飞点着陆 |
| 🛑 紧急停止 | `rosservice call /uav/navigator/command "{command: 'EMERGENCY_STOP'}"` | 紧急 AUTO.LAND |
| 🔄 重置 | `rosservice call /uav/navigator/command "{command: 'RESET'}"` | EMERGENCY/LANDED→IDLE |
| 💾 保存 | `rosservice call /uav/waypoint_manager/save_waypoints "{file_path: '...'}"` | 保存到 XML |
| 📂 加载 | `rosservice call /uav/waypoint_manager/load_waypoints "{file_path: '...'}"` | 从 XML 加载 |

## 实验记录

`experiment_recorder` 在 navigator 进入 PRE_FLIGHT 时自动开始记录，LANDED/EMERGENCY 时自动停止。

```
~/experiments/YYYY-MM-DD/experiment_NNN/
├── metadata.yaml          # 实验元数据 + 偏差统计
├── data.bag               # 原始 rosbag（全部话题）
├── metrics.csv            # 逐帧指标 CSV
├── trajectory_real.csv    # 实际飞行轨迹 CSV
└── summary.txt            # 文本摘要（状态时长分布）
```

## 调试

```bash
# 导航器状态（最重要）
rostopic echo /uav/navigator/status
# 关注字段: state, current_waypoint_index, total_waypoints, error_message

# 安全告警
rostopic echo /uav/safety/alert

# MAVROS 连接
rostopic echo /mavros/state

# 当前位置
rostopic echo /mavros/local_position/odom

# 当前航点
rostopic echo /uav/waypoints/current

# 航点参数（per-waypoint hover/speed）
rostopic echo /uav/waypoints/params_loaded

# 实验指标
rostopic echo /uav/experiment/metrics

# 节点列表
rosnode list

# 话题列表
rostopic list
```

## 目录结构

```
catkin_ws_copy/
├── config.yaml                        # ★ 统一配置文件（唯一可变参数来源）
├── CHANGELOG.md                       # 修改日志
├── CLAUDE.md                          # AI 助手指令
├── README.md                          # 本文件
├── scripts/                           # 启动脚本
│   ├── start_ground_station.sh        # 启动地面站核心
│   ├── start_rviz.sh                  # 启动 RViz + 面板
│   └── start_mission.sh               # 加载航点并执行
├── src/
│   ├── uav_navigator/                 # ★ 导航核心包
│   │   ├── README.md                  # 包详细文档
│   │   ├── src/navigator.cpp          # 10 状态状态机
│   │   ├── src/safety_monitor.cpp     # 独立安全节点
│   │   ├── src/logger.cpp             # 终端日志
│   │   ├── src/experiment_recorder.cpp# 实验记录
│   │   ├── msg/NavigatorStatus.msg    # 导航状态消息
│   │   ├── msg/ExperimentMetrics.msg  # 实验指标消息
│   │   ├── srv/NavigatorCommand.srv   # 导航命令服务
│   │   ├── launch/ground_station.launch
│   │   └── CMakeLists.txt
│   ├── uav_waypoint_manager/          # ★ 航点持久化包
│   │   ├── README.md                  # 包详细文档
│   │   ├── src/waypoint_manager.cpp   # XML 读写 + 验证
│   │   ├── srv/LoadWaypoints.srv      # 加载服务（返回 PoseArray）
│   │   ├── srv/SaveWaypoints.srv
│   │   ├── srv/ClearWaypoints.srv
│   │   └── CMakeLists.txt
│   ├── rviz_waypoint_panel/           # ★ RViz 面板插件
│   │   ├── README.md                  # 包详细文档
│   │   ├── src/waypoint_panel.cpp     # Qt5 面板实现
│   │   ├── src/waypoint_panel.h       # 面板头文件
│   │   ├── config/uav_navigation.rviz # RViz 布局配置
│   │   ├── launch/rviz_ground_station.launch
│   │   ├── plugin_description.xml
│   │   └── CMakeLists.txt
```

## 更多信息

- 各包详细文档：`src/<包名>/README.md`
- 修改历史：[CHANGELOG.md](CHANGELOG.md)
- AI 开发指令：[CLAUDE.md](CLAUDE.md)
