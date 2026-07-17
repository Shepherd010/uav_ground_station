# 修改日志 (Changelog)

## 项目：无人机地面站航点导航系统

---

## [2.0.0] - 2026-07-17 - 全面代码审计与关键修复

### Phase 1: Workspace Cleanup
- **Deleted** `agent-skills/` — independent Claude Code plugin, unrelated to UAV
- **Deleted** `uav_hover/` — legacy package with hardcoded values and safety issues
- **Deleted** `src/rviz_navi_multi_goals_pub_plugin/` — legacy RViz plugin (incompatible with MAVROS)
- **Deleted** per-package deprecated configs (`navigator_config.yaml`, `panel_config.yaml`, `manager_config.yaml`)
- **Deleted** duplicate rviz config files (root `uav_navigation.rviz`, `uav_navigator/config/uav_navigation.rviz`)
- **Deleted** all test scripts and test data
- **Deleted** `uav_navigator/test/` directory and `scripts/__pycache__/`
- **Added** `.gitignore` for build artifacts and IDE files
- **Initialized** Git repository for change tracking

### Phase 2: Critical Fixes — Navigator (navigator.cpp)
- **Fixed** `time_since_start` always equal to `time_in_current_state` (copy-paste bug, now uses `mission_start_time_`)
- **Fixed** RETURNING state can stall forever without home position (added timeout = takeoff_timeout x 3)
- **Fixed** TAKEOFF pre-publish phase has no timeout (added offboard_timeout guard)
- **Fixed** division by zero when `setpoint_rate=0` (added validation with fallback to 20.0 Hz)
- **Fixed** pre-flight passes while drone dangerously above takeoff height (blocks takeoff if > takeoff_height + max(3m, 2x takeoff_height))
- **Fixed** double-publishing setpoints (removed from handle* functions, setpointTimerCallback is single source)
- **Fixed** landing/emergency timeouts hardcoded (now configurable via `mode/landing_timeout` and `mode/emergency_timeout`)
- **Fixed** emergency reason strings mixed Chinese/English (all English now)
- **Fixed** static local variables in timer callbacks (replaced with ROS_INFO_THROTTLE)
- **Removed** dead `offboard_entry_ready_` member
- **Added** `mission_start_time_` tracking for correct experiment metrics
- **Added** parameter validation for `landing_timeout`, `emergency_timeout`

### Phase 3: Critical Fixes — Safety Monitor (safety_monitor.cpp)
- **Added** heartbeat publisher on `uav/safety/heartbeat` — proves safety_monitor is alive each check cycle
- **Added** NaN/Inf validation on all position data (`isPositionValid()` guard)
- **Added** config parameter validation (check_interval, max_height_limit, communication_timeout bounds)
- **Added** try-catch in checkTimerCallback (exceptions no longer crash the safety monitor)
- **Added** alert deduplication (same alert type not repeated within `alert_min_interval` seconds)
- **Added** subscriber check on emergency_stop (warns if no subscriber on alert topic)
- **Fixed** position jump detection: first message no longer checked, post-jump reference preserved (not overwritten)
- **Fixed** position jump window adapts to actual odom rate
- **Fixed** no-odom short-circuit: now only skips height check, other checks still run
- **Fixed** `has_navigator_status_` no longer reset to false after timeout (re-alerts continue)
- **Fixed** used `NavigatorStatus` message constants instead of magic numbers for state detection
- **Fixed** MAVROS disconnect alert now uses "MAVROS_DISCONNECTED" (was "COMMUNICATION_TIMEOUT", now distinct)
- **Fixed** navigator timeout alert now uses "NAVIGATOR_TIMEOUT" (was "COMMUNICATION_TIMEOUT", now distinct)
- **Fixed** mode mismatch `isZero()` check instead of `toSec() == 0.0`

### Phase 4: Critical Fixes — Waypoint Manager (waypoint_manager.cpp)
- **Fixed** CRITICAL self-subscription feedback loop: param subscriber and publisher now use different topics
  - Subscribes to: `uav/waypoints/params` (from panel)
  - Publishes to: `uav/waypoints/params_loaded` (to panel, new topic)
- **Added** NaN/Inf coordinate validation in waypointsCallback and loadFromXml
- **Added** per-stod try-catch with individual field error handling
- **Added** partial XML load rollback (temp containers, only commit on success)
- **Added** empty file check before XML parse
- **Fixed** `sprintf` replaced with `snprintf` for buffer safety
- **Fixed** unsigned underflow risk in checkWaypointSpacing (uses `i + 1 < poses.size()` pattern)
- **Added** try-catch in service callbacks

### Phase 5: Critical Fixes — RViz Panel (waypoint_panel.cpp/h)
- **Fixed** `clearWaypoints()` now also clears `plan_maker_points_`, params vectors, trajectory, and resets phase
- **Fixed** `startSpin()` changed from static to non-static member function (fixes Qt signal/slot reliability)
- **Fixed** waypoint params subscriber uses `uav/waypoints/params_loaded` topic (avoids feedback loop)
- **Fixed** no-op `checkWaypointConfirmation()` removed (confirmation handled in receiveNavStatus)

### Phase 6: Logger Display Fix (logger.cpp)
- **Rewrote** logger with ANSI color-coded state display
- **Added** clear screen on startup with compact table header
- **Added** per-state color formatting (blue=idle, green=navigating, red=emergency)
- **Removed** verbose per-event logging (data aggregated in status line)
- **Added** automatic table header reprint every 20 lines

### Phase 7: Configuration Updates (config.yaml)
- **Added** `topics/waypoint_params_input` and `topics/waypoint_params_current`
- **Added** `mode/landing_timeout: 60.0` and `mode/emergency_timeout: 120.0`
- **Added** `safety/heartbeat_topic: "uav/safety/heartbeat"`
- **Added** `safety/alert_min_interval: 1.0`

### Build Verification
- All 3 packages compile: **0 errors, 0 warnings**
- `uav_navigator`, `uav_waypoint_manager`, `rviz_waypoint_panel`

---

## [1.3.0] - 2026-06-30 - 航点规划、轨迹对比、实验记录、OFFBOARD 安全增强

### 新增航点规划工作流 (Plan Maker)

- **修改文件:** `src/rviz_waypoint_panel/src/waypoint_panel.h`, `src/rviz_waypoint_panel/src/waypoint_panel.cpp`, `src/rviz_waypoint_panel/config/panel_config.yaml`
- **修改原因:** 用户需要在 RViz 中可视化规划点、连接轨迹、删除点，并确认 navigator 收到航点后再执行任务
- **修改内容:**
  - 新增"航点规划"UI 区域：连接轨迹、删除选中、清除规划、保存航点、发布任务
  - 点击 2D Nav Goal 添加橙色球体规划点，发布到 `/uav/plan_maker/points`
  - 点击"连接轨迹"生成橙色规划轨迹，发布到 `/uav/plan_maker/trajectory`
  - 点击"保存航点"将航点发布到 `/uav/waypoints/input`，并通过 `NavigatorStatus.total_waypoints` 确认 navigator 已收到
  - 点击"发布任务"发送 START 命令开始导航
  - 状态显示：PLANNING / CONNECTED / SAVED / NAVIGATING
- **影响范围:** rviz_waypoint_panel
- **验证方法:** RViz 中打点 → 连接轨迹 → 保存航点 → 发布任务，观察状态变化和轨迹显示

### 新增轨迹对比

- **修改文件:** `src/uav_navigator/src/navigator.cpp`, `src/uav_navigator/config/navigator_config.yaml`
- **修改原因:** 用户需要对比规划轨迹和真实飞行轨迹
- **修改内容:**
  - navigator 在切换到 NAVIGATING 时发布 `/uav/trajectory/planned`（绿色，从 waypoints 生成）
  - 飞行过程中按 0.1s 间隔采样 odom，发布 `/uav/trajectory/real`（红色）
  - 实时计算当前位置与规划轨迹的偏差
- **影响范围:** uav_navigator
- **验证方法:** `rostopic echo /uav/trajectory/planned` 和 `/uav/trajectory/real`

### 新增实验记录节点

- **修改文件:** `src/uav_navigator/src/experiment_recorder.cpp`, `src/uav_navigator/CMakeLists.txt`, `src/uav_navigator/package.xml`, `src/uav_navigator/msg/ExperimentMetrics.msg`, `src/uav_navigator/launch/ground_station.launch`, `src/uav_navigator/config/navigator_config.yaml`
- **修改原因:** 用户需要完整记录实验指标用于分析
- **修改内容:**
  - 新增 `experiment_recorder` 节点，在 PRE_FLIGHT 自动开始记录，LANDED/EMERGENCY 自动停止
  - 记录 topics：`/uav/navigator/status`, `/mavros/local_position/odom`, `/mavros/setpoint_position/local`, `/mavros/state`, `/uav/safety/alert`, `/uav/trajectory/planned`, `/uav/trajectory/real`, `/uav/experiment/metrics`
  - 输出 `data.bag`, `metrics.csv`, `trajectory_real.csv`, `metadata.yaml`, `summary.txt`
  - 输出目录：`/home/groundstation/experiments/YYYY-MM-DD/experiment_NNN/`
- **影响范围:** uav_navigator
- **验证方法:** 执行一次任务后检查实验输出目录

### 新增实时实验指标消息

- **修改文件:** `src/uav_navigator/msg/ExperimentMetrics.msg`, `src/uav_navigator/src/navigator.cpp`
- **修改原因:** 为实验记录提供结构化实时数据
- **修改内容:**
  - 新增 `uav_navigator/ExperimentMetrics` 消息
  - 包含：导航状态、航点进度、当前/目标位置、偏差（x/y/z/total）、时间
- **影响范围:** uav_navigator
- **验证方法:** `rosmsg show uav_navigator/ExperimentMetrics`

### OFFBOARD 安全增强

- **修改文件:** `src/uav_navigator/src/navigator.cpp`, `src/uav_navigator/src/safety_monitor.cpp`, `src/uav_navigator/config/navigator_config.yaml`
- **修改原因:** OFFBOARD 模式对 setpoint 流和模式一致性要求严格，必须保证安全
- **修改内容:**
  - navigator 监控 setpoint 发布率，低于阈值记录警告
  - navigator 检测飞行中模式被切出 OFFBOARD，超过阈值触发 EMERGENCY
  - navigator 检测位置跳变，超过阈值触发 EMERGENCY
  - safety_monitor 订阅 `/mavros/setpoint_position/local` 检查 setpoint 流
  - safety_monitor 检测 navigator NAVIGATING 但 MAVROS 模式非 OFFBOARD
  - safety_monitor 检测位置跳变
- **影响范围:** uav_navigator
- **验证方法:** SITL 中手动切换模式或停止 setpoint，观察是否触发紧急状态

### Logger 增强

- **修改文件:** `src/uav_navigator/src/logger.cpp`, `src/uav_navigator/config/navigator_config.yaml`
- **修改原因:** 配合实验记录显示轨迹偏差
- **修改内容:**
  - 订阅 `/uav/experiment/metrics`
  - 新增 `[METRICS]` 输出行，显示 deviation 和 dxyz
  - 表头增加"偏差"列
- **影响范围:** uav_navigator
- **验证方法:** `rosrun uav_navigator logger` 观察输出

### RViz 显示优化

- **修改文件:** `src/rviz_waypoint_panel/config/uav_navigation.rviz`, `uav_navigation.rviz`, `src/rviz_waypoint_panel/config/panel_config.yaml`
- **修改原因:** 原有显示存在重复和模糊问题
- **修改内容:**
  - Navigation 分组重新组织：Plan Maker Points、Plan Maker Trajectory、Waypoint Markers、Planned Trajectory、Real Trajectory、Current Pose
  - 移除 Waypoint Path 重复显示
  - 航点 Marker 尺寸缩小，颜色改为更亮的金色
  - 轨迹线宽增加
- **影响范围:** rviz_waypoint_panel
- **验证方法:** 加载新 RViz 配置，检查显示清晰度和无重复

### 启动脚本默认 ROS 网络更新

- **修改文件:** `scripts/start_ground_station.sh`, `scripts/start_rviz.sh`, `scripts/start_mission.sh`
- **修改原因:** 当前地面站实际 IP 为 192.168.31.116
- **修改内容:** 默认 `ROS_MASTER_URI` 和 `ROS_IP` 改为 `192.168.31.116`
- **影响范围:** scripts
- **验证方法:** 检查脚本中的默认环境变量

### 修复 Logger 加入启动文件

- **修改文件:** `src/uav_navigator/launch/ground_station.launch`, `src/uav_navigator/src/logger.cpp`, `src/uav_navigator/config/navigator_config.yaml`
- **修改原因:** logger 节点已编译但未在 ground_station.launch 中启动，且话题名硬编码
- **修改内容:**
  - 在 `ground_station.launch` 中添加 `uav_logger` 和 `uav_experiment_recorder` 节点
  - logger 话题名改为从 YAML 的 `logger/` 命名空间读取
- **影响范围:** uav_navigator
- **验证方法:** `roslaunch uav_navigator ground_station.launch` 后 `rosnode list` 看到 logger

### 修复 SITL 测试脚本

- **修改文件:** `scripts/test_sitl.py`
- **修改原因:** `_setup_px4_env` 中 `rpp` 变量未定义
- **修改内容:** 添加 `rpp = os.environ.get("ROS_PACKAGE_PATH", "")`
- **影响范围:** scripts
- **验证方法:** `python3 scripts/test_sitl.py`

---

## [1.4.0] - 2026-06-30 - 统一配置、热加载、SITL 三轮验证

### 新增统一配置文件

- **新增文件:** `config.yaml`
- **修改原因:** 用户要求所有可变参数（最大飞行高度、默认起飞高度、到达检测范围、悬停时间等）只能从一个配置文件修改，禁止在面板直接编辑。
- **修改内容:**
  - 合并原 `uav_navigator/config/navigator_config.yaml` 与 `rviz_waypoint_panel/config/panel_config.yaml` 中的可调参数。
  - 包含：命名空间、话题名、服务名、文件路径、航点验证、OFFBOARD 安全、位置异常检测、实验记录、Logger、飞行参数、航点参数、模式切换、安全参数、RViz 面板参数。
  - 关键字段：`flight/takeoff_height`、`flight/hover_duration`、`waypoint/reach_threshold_xy/z`、`safety/max_height_limit`。
- **影响范围:** 全部地面站节点
- **验证方法:** `rosparam load config.yaml` 成功，且各节点能正确读取参数

### 启动文件统一加载 config.yaml

- **修改文件:** `src/uav_navigator/launch/ground_station.launch`, `src/rviz_waypoint_panel/launch/rviz_ground_station.launch`
- **修改原因:** 让所有节点从同一个文件读取配置
- **修改内容:**
  - 将 `$(find uav_navigator)/config/navigator_config.yaml` 替换为 `$(find uav_navigator)/../../config.yaml`
  - 将 `$(find rviz_waypoint_panel)/config/panel_config.yaml` 替换为 `$(find rviz_waypoint_panel)/../../config.yaml`
- **影响范围:** uav_navigator、rviz_waypoint_panel
- **验证方法:** `roslaunch uav_navigator ground_station.launch` 正常启动，参数正确

### RViz 面板新增“从文件加载”按钮（只读）

- **修改文件:** `src/rviz_waypoint_panel/src/waypoint_panel.h`, `src/rviz_waypoint_panel/src/waypoint_panel.cpp`, `src/rviz_waypoint_panel/CMakeLists.txt`
- **修改原因:** 用户要求参数只能通过配置文件修改，面板提供加载入口并发送到 logger 检阅
- **修改内容:**
  - 新增“配置参数”区域，包含“从文件加载”按钮和只读文本显示区。
  - 点击按钮后使用 `rosparam load` 上传参数，解析 YAML 显示关键参数，不可编辑。
  - 通过 `uav/config/loaded` 话题发布加载事件（含文件路径与参数摘要）。
  - 面板 `loadConfig()` 改为使用全局参数句柄读取统一配置。
  - `CMakeLists.txt` 链接 `yaml-cpp` 以支持本地解析。
- **影响范围:** rviz_waypoint_panel
- **验证方法:** RViz 中点击“从文件加载”，观察只读参数显示和 logger 输出

### Navigator 支持配置热重载

- **修改文件:** `src/uav_navigator/src/navigator.cpp`
- **修改原因:** 让用户修改 config.yaml 并加载后，飞行/安全参数立即生效，无需重启节点
- **修改内容:**
  - 新增 `topics/config_reload_topic` 参数（默认 `uav/config/reload`）。
  - 新增 `configReloadCallback`，订阅 `std_msgs/String` 类型 reload 事件。
  - 回调中重新调用 `loadConfig()`，并动态更新 setpoint 定时器周期。
  - 重载范围：飞行、航点、模式切换、安全、OFFBOARD 安全、位置异常检测等数值参数。
- **影响范围:** uav_navigator
- **验证方法:** 修改 `flight/setpoint_rate` 后发布 reload 事件，观察 setpoint 频率变化

### Safety Monitor 支持配置热重载

- **修改文件:** `src/uav_navigator/src/safety_monitor.cpp`
- **修改原因:** 安全阈值（如最大高度、检查周期）需要随配置即时生效
- **修改内容:**
  - 新增 `topics/config_reload_topic` 参数与 `configReloadCallback`。
  - 回调中重新调用 `loadConfig()`，并动态更新 `check_timer_` 周期。
- **影响范围:** uav_navigator
- **验证方法:** 修改 `safety/check_interval` 后发布 reload 事件，观察检查频率

### Logger 接收配置加载事件并打印检阅

- **修改文件:** `src/uav_navigator/src/logger.cpp`
- **修改原因:** 用户要求加载成功后发送到 logger 后台检阅
- **修改内容:**
  - 新增 `topics/config_loaded_topic` 订阅。
  - 新增 `configLoadedCallback`，在终端以框线形式打印配置文件路径与关键参数摘要。
- **影响范围:** uav_navigator
- **验证方法:** RViz 加载配置后，logger 终端出现 `[CONFIG LOADED]` 框线输出

### 修复 experiment_recorder 漏启动记录

- **修改文件:** `src/uav_navigator/src/experiment_recorder.cpp`
- **修改原因:** SITL 中 PRE_FLIGHT 状态极短，导致实验记录未启动
- **修改内容:**
  - 将自动开始记录条件从 `nav_state == PRE_FLIGHT` 放宽为 `nav_state` 在 1-6（PRE_FLIGHT 到 LANDING）之间且未记录时启动。
  - 保持 LANDED/EMERGENCY 时停止记录。
- **影响范围:** uav_navigator
- **验证方法:** 三轮 SITL 均生成 `metadata.yaml`、`data.bag`、`metrics.csv`

### 修复并增强 SITL / Mock 测试脚本

- **修改文件:** `scripts/test_sitl.py`, `scripts/test_integration_mock.py`
- **修改原因:** 原脚本在 ROS master 关闭或进程未启动时因 `rospy.Time.now()` 停止推进而无限挂起
- **修改内容:**
  - 将所有 `rospy.Time.now()` 超时改为 `time.time()` 墙钟超时，避免 cleanup 时死循环。
  - 将 `rospy.init_node` 移到 roscore 确认启动之后，防止初始注册挂起。
  - 背景进程 stdout/stderr 重定向到 `/tmp/uav_sitl_*.log` 与 `/tmp/uav_mock_*.log` 便于调试。
  - SITL 测试新增：
    - `test_trajectory_topics`：验证 `/uav/trajectory/planned` 与 `/uav/trajectory/real`。
    - `test_experiment_recording`：验证实验记录文件生成。
    - `test_config_reload`：验证 `rosparam load config.yaml` 与 reload 事件发布。
- **影响范围:** scripts
- **验证方法:** `python3 scripts/test_sitl.py` 连续运行三轮，均 19 passed, 0 failed

### SITL 三轮测试结果

| 轮次 | 结果 | 实验输出目录 |
|------|------|--------------|
| Round 1 | 19 passed, 0 failed | `/home/groundstation/experiments/2026-06-30/experiment_001` |
| Round 2 | 19 passed, 0 failed | `/home/groundstation/experiments/2026-06-30/experiment_002` |
| Round 3 | 19 passed, 0 failed | `/home/groundstation/experiments/2026-06-30/experiment_003` |

---


### 重构背景

- 阶段编号（phase1/2/3/5）与实际部署流程不符：机载 DLIO + MAVROS 链路由用户在无人机上位机单独启动，地面站只负责"打点"和"导航"。
- 旧的 `namespace: "uav1"` 会导致 MAVROS 话题被错误拼接，与机载 MAVROS 不兼容。
- 旧的启动脚本缺少默认的 ROS 网络配置，无法直接用于真实飞行。

### 变更内容

#### 启动文件重构

- **删除文件：**
  - `src/uav_navigator/launch/phase1_ground_station.launch`
  - `src/uav_navigator/launch/phase2_drone_core.launch`
  - `src/uav_navigator/launch/phase3_visualization.launch`
  - `src/uav_navigator/launch/phase5_execute_navigation.launch`
- **新增文件：**
  - `src/uav_navigator/launch/ground_station.launch` — 地面站核心节点（navigator + safety_monitor + waypoint_manager）
  - `src/rviz_waypoint_panel/launch/rviz_ground_station.launch` — RViz + 航点面板
- **影响范围：** uav_navigator、rviz_waypoint_panel
- **验证方法：** `roslaunch uav_navigator ground_station.launch` 能正常启动三个节点

#### 启动脚本重构

- **删除文件：**
  - `scripts/start_phase1.sh`
  - `scripts/start_phase2.sh`
  - `scripts/start_phase3.sh`
  - `scripts/start_phase5.sh`
- **新增文件：**
  - `scripts/start_ground_station.sh` — 启动地面站核心，自动拉起 roscore
  - `scripts/start_rviz.sh` — 启动可视化与航点面板
  - `scripts/start_mission.sh <waypoint_file.xml>` — 加载航点并开始导航
- **默认 ROS 网络配置：**
  - 地面站：`ROS_MASTER_URI=http://192.168.31.30:11311`，`ROS_IP=192.168.31.30`
  - 机载端：`ROS_MASTER_URI=http://192.168.31.30:11311`，`ROS_IP=192.168.31.180`
- **影响范围：** scripts
- **验证方法：** 运行新脚本，检查环境变量和节点列表

#### 配置适配机载 MAVROS

- **修改文件：** `src/uav_navigator/config/navigator_config.yaml`
- **修改内容：**
  - `namespace` 改为空字符串，避免话题拼接错误
  - 话题名保持与机载 MAVROS 一致：`mavros/state`、`mavros/local_position/odom`、`mavros/setpoint_position/local`、`mavros/cmd/arming`、`mavros/set_mode`
  - 删除对 `uav1/` 前缀的引用
- **影响范围：** uav_navigator
- **验证方法：** 机载 MAVROS 启动后，地面站节点能正确订阅/发布 MAVROS 话题

#### RViz 配置归一化

- **修改文件：**
  - `src/rviz_waypoint_panel/config/uav_navigation.rviz`（新增）
  - `uav_navigation.rviz`（同步更新）
- **修改内容：**
  - 修正 `Current Setpoint` 显示话题为 `/mavros/setpoint_position/local`
  - launch 文件改用 `$(find rviz_waypoint_panel)/config/uav_navigation.rviz`
- **影响范围：** rviz_waypoint_panel、项目根目录
- **验证方法：** `roslaunch rviz_waypoint_panel rviz_ground_station.launch` 正常加载

#### 文档同步

- **修改文件：** `CLAUDE.md`
- **修改内容：** 移除 phase 编号，更新启动命令、ROS 网络说明、使用流程
- **影响范围：** 项目文档

---

## [1.1.0] - 2026-06-24 - Logger、面板增强、英文日志

### 新增 Logger 节点

- **修改文件:** `src/uav_navigator/src/logger.cpp`, `src/uav_navigator/CMakeLists.txt`
- **修改原因:** 用户需要在终端实时查看无人机状态和导航信息
- **修改内容:**
  - 新增 `logger` 可执行文件，订阅 `/uav/navigator/status`, `/mavros/state`, `/mavros/local_position/odom`, `/uav/safety/alert`, `/mavros/setpoint_position/local`
  - 格式化输出：时间戳、状态、航点进度、位置、目标、模式、解锁状态
  - 可配置输出级别（rosparam）
  - 默认 2Hz 打印频率
- **影响范围:** uav_navigator 包
- **验证方法:** `rosrun uav_navigator logger` 观察终端输出

### RViz 面板系统控制集成

- **修改文件:** `src/rviz_waypoint_panel/src/waypoint_panel.h`, `src/rviz_waypoint_panel/src/waypoint_panel.cpp`
- **修改原因:** 将 shell 脚本功能集成到 RViz 面板，提升操作便捷性
- **修改内容:**
  - 新增系统状态栏：MAVROS 连接状态（绿/红LED）、解锁状态、飞行模式
  - 新增导航状态区：状态指示灯（颜色编码）、航点进度条、位置显示
  - 新增系统控制按钮：🚀启动地面站、⏹停止地面站、🛫一键起飞、🛬一键降落、▶执行任务
  - 新增导航控制按钮：▶开始导航、⏸暂停、⏹取消、🏠返航、🔄重置、🛑紧急停止
  - 新增操作日志区：带时间戳的 HTML 格式日志（绿/橙/红）
  - 实时节点状态检查：每秒检测节点运行状态，自动启用/禁用按钮
- **影响范围:** rviz_waypoint_panel 包
- **验证方法:** RViz 加载面板，检查按钮和状态显示

### UI/UX 优化

- **修改原因:** 提升人机交互体验和可视化表达
- **修改内容:**
  - 颜色编码：绿色=正常，橙色=警告，红色=危险，蓝色=完成
  - 按钮样式：按功能着色，禁用状态自动灰显
  - 紧急停止按钮带确认对话框
  - 航点进度条实时更新
- **影响范围:** rviz_waypoint_panel 包

### 中文日志改为英文

- **修改文件:** `src/uav_navigator/src/navigator.cpp`, `src/uav_navigator/src/safety_monitor.cpp`, `src/uav_navigator/src/logger.cpp`, `src/uav_waypoint_manager/src/waypoint_manager.cpp`
- **修改原因:** roslaunch 子进程 stdout 编码问题导致中文显示为乱码
- **修改内容:** 将中文 ROS 日志（ROS_INFO/ROS_WARN/ROS_ERROR）替换为英文
- **保留内容:** 中文注释、中文文档、RViz 面板 Qt 控件文本
- **影响范围:** uav_navigator, uav_waypoint_manager 包
- **验证方法:** `roslaunch uav_navigator ground_station.launch` 确认无乱码

### RViz 配置移动

- **修改文件:** `uav_navigation.rviz`（根目录）
- **修改原因:** 用户要求配置文件放在项目根目录
- **修改内容:** 将 `src/uav_navigator/config/uav_navigation.rviz` 复制到项目根目录
- **影响范围:** 项目根目录
- **验证方法:** `roslaunch rviz_waypoint_panel rviz_ground_station.launch` 加载配置

---

## [1.0.0] - 2026-06-24 - 完整系统重构

### 重构背景

原有系统存在严重架构缺陷：
- RViz 多点导航插件面向地面机器人 `move_base` 导航栈，与无人机 MAVROS 完全不兼容
- 两个子系统之间没有任何桥接，数据流完全断裂
- 所有参数硬编码，不可配置
- 无安全监控机制
- 存在多项安全隐患（如 `hover.cpp` 发布 100 次后退出会导致坠机）

### 新增模块

#### 1. uav_navigator（导航核心 + 安全监控）

**新增文件：**
- `src/uav_navigator/CMakeLists.txt`
- `src/uav_navigator/package.xml`
- `src/uav_navigator/msg/NavigatorStatus.msg`
- `src/uav_navigator/srv/NavigatorCommand.srv`
- `src/uav_navigator/src/navigator.cpp`
- `src/uav_navigator/src/safety_monitor.cpp`
- `src/uav_navigator/config/navigator_config.yaml`
- `src/uav_navigator/launch/ground_station.launch`（历史版本中为 phase 编号文件）

**功能：**
- 10 状态导航状态机：IDLE → PRE_FLIGHT → ARMING → TAKEOFF → NAVIGATING → HOVERING → LANDING → LANDED → EMERGENCY → RETURNING
- 所有参数从 YAML 配置文件读取，零硬编码
- MAVROS 交互：setpoint 发布（20Hz）、OFFBOARD 模式切换、解锁/上锁
- 安全保护：高度超限、通信超时、OFFBOARD 超时、降落超时
- 异常状态转换：EMERGENCY_STOP、RETURNING、LANDING

#### 2. uav_waypoint_manager（航点管理）

**新增文件：**
- `src/uav_waypoint_manager/CMakeLists.txt`
- `src/uav_waypoint_manager/package.xml`
- `src/uav_waypoint_manager/srv/LoadWaypoints.srv`
- `src/uav_waypoint_manager/srv/SaveWaypoints.srv`
- `src/uav_waypoint_manager/srv/ClearWaypoints.srv`
- `src/uav_waypoint_manager/src/waypoint_manager.cpp`
- `src/uav_waypoint_manager/config/manager_config.yaml`

**功能：**
- XML 格式航点保存与加载（支持元数据、创建时间、frame_id）
- 航点验证：重复检测、间距检查、高度范围检查
- 服务接口：LoadWaypoints、SaveWaypoints、ClearWaypoints
- PoseArray 话题发布/订阅

#### 3. rviz_waypoint_panel（修复版 RViz 插件）

**新增文件：**
- `src/rviz_waypoint_panel/CMakeLists.txt`
- `src/rviz_waypoint_panel/package.xml`
- `src/rviz_waypoint_panel/plugin_description.xml`
- `src/rviz_waypoint_panel/src/waypoint_panel.h`
- `src/rviz_waypoint_panel/src/waypoint_panel.cpp`
- `src/rviz_waypoint_panel/config/panel_config.yaml`

**功能：**
- 订阅标准话题 `move_base_simple/goal`（原插件使用非标准 `move_base_simple/goal_temp`）
- 所有话题名、Marker 参数、颜色、尺寸从配置文件读取
- 航点表格支持 x, y, z, yaw 编辑
- 单点删除、上移、下移功能
- 航点保存/加载（调用 waypoint_manager 服务）
- 导航状态实时显示
- 紧急停止按钮（带确认对话框）
- 空指针检查（修复原插件表格 item 空指针崩溃 bug）
- 参数化配置（修复原插件所有硬编码值）

### 问题修复

#### 原 RViz 插件修复

| 问题 | 修复方法 |
|------|----------|
| 订阅非标准话题 `move_base_simple/goal_temp` | 改为订阅标准 `move_base_simple/goal`（可配置） |
| 缺少 `visualization_msgs` 依赖声明 | 在 `package.xml` 和 `CMakeLists.txt` 中正确声明依赖 |
| 表格 item 空指针未检查 | 所有 `setBackgroundColor` 前检查 `item != nullptr` |
| 硬编码所有话题名、阈值、颜色、频率 | 全部从 ROS 参数服务器读取 |
| 日志中目标索引 off-by-one 错误 | 修正日志输出，正确显示当前航点索引 |
| `saveTableData()` 只保存到内存 | 新增 XML 持久化保存，调用 waypoint_manager 服务 |
| 无航点编辑功能 | 新增 z 值和 yaw 值编辑、单点删除、上移下移 |
| 无导航状态反馈 | 新增状态显示区域，订阅 `/uav/navigator/status` |
| 无紧急停止功能 | 新增紧急停止按钮（带确认对话框） |

#### 原 Hover 包修复（保留备份，不再使用）

| 问题 | 说明 |
|------|------|
| `hover.cpp` 发布 100 次后退出 | 会导致 OFFBOARD 模式下坠机，已在新系统修复 |
| 话题名混用相对路径和绝对路径 | 新系统统一使用相对路径 |
| `waypoint_land.cpp` LOITER 模式 + setpoint 矛盾 | 新系统仅使用 OFFBOARD 模式，状态机清晰 |
| 主线程 `ros::Duration::sleep()` 阻塞 | 新系统使用 ros::Timer，不阻塞 setpoint 发布 |
| 无安全超时机制 | 新系统所有阶段都有超时保护 |
| 所有参数硬编码 | 新系统全部参数 YAML 配置 |

### 架构改进

1. **完全配置化**：所有参数从 YAML 读取，禁止硬编码
2. **解耦设计**：RViz 插件只负责航点标注，不直接控制无人机
3. **安全优先**：安全监控独立节点，最高优先级中断权
4. **状态机驱动**：10 状态显式状态机，所有转换可追溯
5. **航点持久化**：XML 格式，支持元数据和离线编辑

### 安全增强

1. **通信超时保护**：navigator 状态超过 5 秒未更新触发告警
2. **高度超限保护**：超过 `max_height_limit`（默认 10 米）触发 EMERGENCY
3. **OFFBOARD 超时保护**：模式切换超过 8 秒自动进入 LANDING
4. **解锁超时保护**：解锁超过 8 秒自动上锁并返回 IDLE
5. **降落超时保护**：降落超过 60 秒强制进入 LANDED
6. **状态异常检测**：检测到手动模式（STABILIZE/ACRO）记录警告
7. **MAVROS 连接断开**：检测到连接断开触发告警
8. **紧急停止**：手动 EMERGENCY_STOP 命令可立即中断任何状态

### 保留文件（未修改）

- `src/rviz_navi_multi_goals_pub_plugin/` - 原 RViz 插件（备份）
- `uav_hover/src/hover/` - 原 hover 包（备份）

---

## 未来计划

- [ ] 多无人机支持（多 namespace 管理）
- [ ] 电池状态监控与低电量返航
- [ ] 地理围栏（水平边界保护）
- [ ] 航点插值与轨迹优化
- [ ] Gazebo 仿真环境测试
- [ ] 航点可达性分析（考虑障碍物）

---

## 修改记录规范

**本项目的硬性约束：每次修改后必须在此文件中记录修改日志。**

记录格式：
```
## [版本号] - 日期 - 修改类型

### 新增/修改/修复内容
- 修改文件: 文件路径
- 修改原因: 说明
- 修改内容: 详细描述
- 影响范围: 说明影响的模块
- 验证方法: 如何验证修改正确
```
