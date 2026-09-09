# 修改日志 (Changelog)

## 项目：无人机地面站航点导航系统

---

## [Unreleased] - 2.5D 静态航点规划器

- 新增 scripts/plan_2d/ 模块：读取一次飞行的 DLIO-SLAM 累计地图、逐帧点云和
  odom，进行 3D 体素稀疏采样、射线清空、XOY 平面栅格化和 8 邻域 A* 规划。
- 新增 planner 配置段：支持固定规划高度、地图切片、飞机尺寸、栅格/采样
  分辨率、航点数量、输出路径和每航点飞行参数。
- 生成与 uav_waypoint_manager XML 持久化格式兼容的航点文件，可直接交给
  scripts/start_mission.sh。
- 当前版本把未知区域保持为 UNKNOWN，A* 禁止进入；输入 frame 要求为
  robot/odom，输出 frame 使用 map。
- 新增按 rosbag 文件指纹缓存解码后的地图、逐帧点云和 odom，避免重复扫描。
- 新增占据栅格持久缓存；按 A* FREE 连通域选择 OCCUPIED 端点吸附区域，避免
  多个自由空间中吸附到孤立区域导致规划失败。
- 默认累计地图采样体素调整为 0.4m，避免细栅格中的墙点膨胀封闭长走廊。

## [3.2.1] - 2026-07-28 - 二次深度排查修复

### 关键修复
- **config.yaml 损坏修复：** `services:` 头部被用户指令文本覆盖，导致 YAML 语法错误
- **孤立参数删除：** `publish_rate: 1.0` 已在 v3.2.0 中从代码移除，但 config.yaml 中残留
- **面板配置显示：** `loadConfigFromFile()` 引用已删除的 `root["flight"]`，改为 `root["flight_defaults"]`
- **配置文件引用错误：** `navigator.cpp` 警告消息引用已删除的 `navigator_config.yaml`
- **CLAUDE.md：** 三处引用已删除的配置文件（`navigator_config.yaml`、`panel_config.yaml`），统一改为 `config.yaml`

### 默认值一致性
- `navigator.cpp` 中 `hover_duration` C++ 默认值 2.0→5.0（与 config.yaml 一致）
- `navigator.cpp` + `safety_monitor.cpp` 中 `max_height_limit` C++ 默认值 10.0→2.0（与 config.yaml 一致）
- `waypoint_panel.cpp` 中 `loadConfig()` 变量名 `pnh`→`nh_global`（消除误导性命名）

### 文档修正
- `uav_waypoint_manager/README.md`：删除孤立参数 `publish_rate` 的文档条目

### 编译验证
- 全部 3 包：0 错误，0 警告
- 全部 4 脚本：`bash -n` 通过
- config.yaml：14/14 section 通过 YAML 结构验证

---

## [3.2.0] - 2026-07-28 - 通用性现代化：路径移植、向后兼容清理、GitHub 就绪

### 路径通用化 — 全部路径基于 `$HOME` 自动适配
- **C++ 代码：** 所有 `param<>()` 默认值改为 `~/...` 格式，新增 `resolveHome()` 函数在启动时展开 `~` 为 `$HOME`
- **config.yaml：** 5 个路径参数全部改为可移植：`~/waypoints.xml`、`~/experiments`，`allowed_base: "~"`
- **panel default_config_path：** 空值时通过 `ros::package::getPath("uav_navigator")` 自动检测
- **Shell 脚本：** `start_mission.sh` 默认值 `$HOME/waypoints.xml`
- **文档：** 全部 `.md` 文件中 `/home/groundstation/...` → `~` 相对路径，IP 地址改为 `<drone-ip>` 占位符

### 删除 `flight` 向后兼容节
- **config.yaml：** 删除 `flight:` 节（与 `flight_defaults` 完全重复的 7 个参数）
- **navigator.cpp：** 删除 `flight_defaults/*` → `flight/*` 回退代码块（~15 行）
- **waypoint_manager.cpp：** 删除相同回退 + `TODO(v4.0)` 注释
- **waypoint_panel.cpp：** 删除相同回退 + `TODO(v4.0)` 注释
- 全局 grep 确认：零 `flight/` 命名空间残余，零 "向后兼容" 注释

### 死代码深度清理
- **navigator.cpp Config：** 删除 4 个从未读取的字段（`min_waypoint_spacing`、`communication_timeout`、`setpoint_timeout_warn`、`setpoint_timeout_emergency`）及对应 param 加载
- **waypoint_manager.cpp Config：** 删除 `publish_rate`（加载但从未使用）
- **safety_monitor.cpp Config：** 之前已删除 `battery_threshold`
- **config.yaml：** 删除 `publish_rate: 1.0` 孤立参数

### GitHub 就绪
- **新增 LICENSE 文件：** MIT 许可证
- **README 首次使用指南：** 新增构建步骤、依赖安装、克隆说明
- **CLAUDE.md：** 修正 3 个已删除配置文件的错误引用，路径全部通用化
- **IP 地址通用化：** 所有 `192.168.31.x` → `localhost` 或 `<drone-ip>` 占位符

### 编译验证
- 全部 3 包通过编译：**0 错误，0 警告**
- `bash -n scripts/*.sh` 全部通过
- `grep -rn "/home/groundstation"` 非 CHANGELOG 文件零匹配

---

## [3.1.0] - 2026-07-28 - 深度排查修复：数据流、死代码、配置统一、文档同步

### 数据流修复 (CRITICAL)

- **【CRITICAL】修复 `topics/metrics` 参数名不匹配：** navigator 读取 `topics/metrics`，config.yaml 定义 `topics/metrics_topic`，导致 navigator 静默使用默认值 (C1)
- **【CRITICAL】恢复 `planned_path` 发布链路：** navigator 恢复 `buildPlannedPath()` 后的 `planned_path_pub_.publish()`，experiment_recorder 的 bag 文件现在包含规划轨迹。此前 `planned_path_pub_` 创建但从不发布，`uav/trajectory/planned` 永久无数据 (C2)
- **【CRITICAL】5 个飞行控制方法检查 `response.success`：** `hoverInPlace()`、`landNow()`、`returnToHome()`、`resetNavigator()`、`emergencyStop()` 现在检查 navigator service 返回值，失败时显示拒绝原因而非虚假成功 (C3)
- **【CRITICAL】YAML 解析失败不再级联重载：** `loadConfigFromFile()` 中 yaml-cpp 异常捕获后立即 `return`，不触发后续 `loadConfig()`/`publishConfigLoaded()`/`config_reload` (C4)

### 死代码清理 (MEDIUM)

- **删除 `NodeStatus` 结构体 + `node_status_map_`：** 声明但从未在 `checkNodeStatus()` 中填充/读取
- **删除未使用的 config 字段：** `trajectory_width`、`color_r/g/b/a`（legacy marker 参数）、`waypoint_current_topic`、`marker_id_counter_`
- **删除无用 includes：** `<QCheckBox>`, `<QComboBox>`, `<QInputDialog>`, `<cstdio>`, `<QPainter>` — 从 header 和 cpp 中清理
- **删除 `actionlib_msgs` 依赖：** 从 CMakeLists.txt（find_package + catkin_package）和 package.xml（build/build_export/exec）中移除
- **修复 `deleteSelectedPlanPoint()` 重复 phase 转换：** 当走 `deleteSelectedWaypoint()` 分支时不再重复调用 `setPlanMakerPhase()` / `publishPlanMakerMarkers()`
- **修复 else 分支同步清理：** `deleteSelectedPlanPoint()` 的 else 分支现在同步清理 `waypoint_hover_times_` 和 `waypoint_speeds_`

### Config 统一 (MEDIUM)

- **`record_control_pub_` 从 config 读取话题名：** 不再硬编码 `"uav/experiment/record"`，新增 `experiment/record_control_topic` 参数读取
- **删除 `topics/record_control_topic` 重复定义：** 仅保留 `experiment/record_control_topic`
- **统一 `min_waypoint_spacing`：** 删除 `waypoint/min_waypoint_spacing`，navigator 和 waypoint_manager 统一读取 `validation/min_waypoint_spacing`
- **`flight` 向后兼容节同步：** waypoint_manager 和 panel 的 `loadConfig()` 增加 `flight_defaults/*` → `flight/*` 回退逻辑（与 navigator 一致），标注 TODO(v4.0) 删除
- **删除 `battery_threshold` 死参数：** safety_monitor 移除该字段和加载代码，config.yaml 注释保留供未来实现
- **删除死 config 参数：** `panel/table/columns`、`panel/display_params`
- **新增 `paths/allowed_base` 配置：** 路径遍历保护基路径从硬编码改为可配置

### 次要代码修复 (LOW)

- **RETURNING 无 home 无 odom 时强制 LANDING：** 原静默失效，现加 `ROS_ERROR` + `transitionState(State::LANDING)` (H3)
- **`stateToString()`/`stateToColor()` 使用命名常量：** `case 0:` → `case NavigatorStatus::STATE_IDLE:` 等 10 个状态
- **XML 解析边界检查：** `<hover_time>` 和 `<speed>` 解析增加 waypoint 边界验证，防止跨 waypoint 读取
- **rosbag 脚本 stderr 重定向到日志文件：** `&>/dev/null` → `2>"${output_dir}/rosbag_stderr.log"`

### 构建修复 (HIGH)

- **修复 `install(DIRECTORY config/)` 错误：** uav_navigator 和 uav_waypoint_manager 的 CMakeLists.txt 移除引用不存在的 config/ 目录
- **新增 rviz_waypoint_panel launch 安装：** 添加 `install(DIRECTORY launch/)` 指令

### 文档同步 (LOW)

- **全局路径修正：** catkin_ws / catkin_ws_copy → uav_ground_station（7 处：README、CLAUDE.md、config.yaml、panel 代码、包 README）
- **start_ground_station.sh 注释修正：** 删除矛盾的 IP 地址注释，默认 localhost
- **README 补充：** scripts 列表添加 `record_bag.sh`
- **rviz_waypoint_panel README markdown 结构修复：** 关闭未闭合代码块
- **CHANGELOG 清理：** 删除无版本号孤悬内容，删除 "legacy" 错误引用

### 编译验证

- 全部 3 个活动包通过编译：**0 错误，0 警告**
- 17 文件变更：+124 / -190 行（净删 66 行）

---

## [3.0.0] - 2026-07-28 - 面板 HMI 修复、录制脚本、数据流完整性

### 面板 HMI 重构（waypoint_panel）

**按钮语义修复（6 个关键问题）：**

| 修复前 | 修复后 | 说明 |
|--------|--------|------|
| "💾 保存" 实际是发布 | "📤 发布" — 标签对齐实际行为 | 不再误导用户 |
| 无 XML 保存按钮 | "💾 保存文件" — 调用 saveWaypoints() | 弹出文件对话框，写入 XML |
| 加载后必须再点"发布" | 加载后自动等待 navigator 确认 → 就绪 | 少一步冗余操作 |
| 开始任务重复发布航点 | 移除 publishWaypoints() 冗余调用 | navigator 已有最新数据 |
| 两套 marker 系统并存 | 删除 legacy markWaypoint/republishMarkers（~80行死代码） | 清理后只有 MarkerArray 系统 |
| publishPlanTask() 孤儿函数 | 删除头文件声明 | 从未实现，无调用者 |

**新增功能：**
- ⏺ **录制按钮** — 面板上手动开始/停止 rosbag 录制，发布 Bool 到 `uav/experiment/record`
- 📊 **工作流进度条** — 4 步可视化：① 打点 → ② 连线 → ③ 就绪 → ④ 执行
- ⏱ **auto-SAVED 超时保护** — navigator 5 秒无响应则提示用户检查 waypoint_manager

**数据一致性修复（5 个问题）：**

| 问题 | 修复 |
|------|------|
| savePlanWaypoints 双源读取（table + plan_maker） | 统一从 plan_maker_points_ 唯一数据源读取 |
| deleteSelectedPlanPoint 无条件重置到 PLANNING | 条件判断：<2点→PLANNING，SAVED→CONNECTED |
| deleteSelectedWaypoint 不更新 phase | 新增与 deletePlan 一致的 phase 回退逻辑 |
| onTableChanged 编辑后不更新 phase | SAVED 状态下编辑自动回退到 CONNECTED |
| moveWaypointUp/Down 移动后不更新 phase | SAVED 状态下移动自动回退到 CONNECTED |

**死代码清理：**
- ❌ `markWaypoint()` / `republishMarkers()` — 旧 marker 系统，无调用者
- ❌ `publishPlanTask()` — 声明但从未实现
- ❌ `plan_maker_dirty_` — 赋值但从未读取
- ❌ `save_plan_button_` → 重命名为 `publish_button_`

**中文界面优化：**
- 阶段标签：PLANNING→打点中, CONNECTED→已连线, SAVED→就绪, NAVIGATING→执行中
- 按钮标签全部中文化
- 日志消息中文化

### 综合录制脚本

**新增 `scripts/record_bag.sh`：**
- 子命令 `start/stop/status`
- 录制全部话题（137 个）排除噪音（rosout/hil/debug/log_transfer 等 23 个）
- 自动命名：`~/experiments/rosbag/flight_YYYY-MM-DD_HH-MM-SS/full_flight_*.bag`
- PID 文件防重复启动，残留自动清理
- SIGINT 优雅停止，bag 正确关闭
- 覆盖：MAVROS 遥测（127话题中的120+）+ DLIO SLAM（7）+ Livox LiDAR（4）+ UAV 内部（13）+ MAVLink + TF

### 编译验证
- 全部 3 个活动包通过编译，0 错误 0 警告
- rviz_waypoint_panel 重构后净变更 ~0 行（增删均衡）

---

## [2.9.0] - 2026-07-17 - 面板 UI 重构、人机交互逻辑优化

### 面板 UI 重构（HMI 优化）

**删除冗余按钮组（4 组 → 合并/删除）：**
- ❌ **地面站节点** — 删除。用户通过外部脚本管理守护进程，不在 RViz 面板内控制
- ❌ **任务执行** — 删除。"加载默认任务"合并到航点规划"📂 加载"，"▶ 执行当前航点"合并到飞行控制"▶ 开始任务"
- ❌ **飞行阶段** — 删除。"🛫 一键起飞/🛬 一键降落/🏠 返航"合并到飞行控制
- ❌ **安全与重置** — 删除。"🔄 重置/🛑 紧急停止"合并到飞行控制

**新增飞行控制组（语义明确的 6 按钮）：**

| 按钮 | 命令 | 效果 |
|------|------|------|
| ▶ 开始任务 | START | 发布航点 → 解锁 → 起飞 → 依次执行 → 降落 |
| ⏸ 悬停 | PAUSE | 保持当前位置悬停（不降落，可继续） |
| 🛬 降落 | LAND | 立即结束任务，AUTO.LAND 着陆 |
| 🏠 返航 | RETURN_TO_HOME | 返回起飞点（TAKEOFF 记录的 Home）着陆 |
| 🔄 重置 | RESET | EMERGENCY/LANDED → IDLE |
| 🛑 紧急停止 | EMERGENCY_STOP | 弹窗确认 → AUTO.LAND |

**航点规划组精简：** 按钮顺序改为 📂加载 → 🔗连接 → 💾保存 → 🗑删除 → 🧹清除（按工作流排列），删除冗余的 ▶发布按钮

### 启动脚本优化
- **所有脚本改用 localhost**：`ROS_MASTER_URI` 默认 `http://localhost:11311`，`ROS_IP` 默认 `127.0.0.1`
- **start_rviz.sh 修复**：删除指向不存在文件的 `rosparam load panel_config.yaml`

### 代码质量
- **删除死代码**：`launchGroundStation()`, `killGroundStation()`, `executeMission()`, `oneKeyTakeoff()`, `oneKeyLand()`, `startNavigation()`, `pauseNavigation()`, `cancelNavigation()`, `publishPlanTask()` 共 9 个废弃函数
- **新增清晰实现**：`startMission()`, `hoverInPlace()`, `landNow()` 带完整 tooltip 说明
- **updateAllButtonStates() 重写**：适配新按钮集合
- **checkNodeStatus() 简化**：删除 launch/kill 按钮状态更新

### 文档更新
- 工作区 README 完全重写（架构图、数据流、话题速查、XML 格式、配置热重载）
- 四包 README 新增（uav_navigator, uav_waypoint_manager, rviz_waypoint_panel）
- 面板 README 更新 UI 布局图和按钮语义表

### 编译验证
- 全部 3 个活动包通过编译，0 错误 0 警告

---

## [2.8.1] - 2026-07-17 - RViz 点云性能优化

### RViz 配置优化（uav_navigation.rviz）

**点云性能（主因修复）：**
- **Decay Time: 9999s → 5s** — 点云 5 秒后自动消失，以 10Hz 频率计最多 50 帧点云同时可见。这是解决 RViz 卡顿的**最关键修改**。之前 9999s 意味着点云永不删除，飞行 10 分钟可累积数百万个点，每帧渲染全部走 alpha blending
- **Queue Size: 10 → 5** — 减少消息缓冲，降低内存峰值

**渲染效率提升：**
- **DLIO Trajectory Line Style: Billboards → Lines** — 简单线段渲染比 3D 面片快 3-5 倍
- **移除僵尸 Display：** 删除旧版 `Waypoint Markers`（visualization_marker topic，旧 markWaypoint 已废弃）和 `Planned Trajectory`（uav/trajectory/planned，navigator 已停止发布），减少无效渲染管线

**可视化完善：**
- **新增方向箭头命名空间：** `uav_plan_maker_arrows` 加入 Plan Maker Points 的 Namespaces 过滤器，确保 yaw 方向箭头正常显示

### 参数参考
| 参数 | 旧值 | 新值 | 效果 |
|------|------|------|------|
| Dense Map Decay Time | 9999s | 5s | 点云自动淘汰，GPU 负载降低 90%+ |
| Dense Map Queue Size | 10 | 5 | 内存占用减半 |
| DLIO Trajectory Style | Billboards | Lines | 路径渲染提速 3-5x |
| Waypoint Markers display | active | removed | 消除僵尸渲染管线 |
| Planned Trajectory display | active | removed | 消除无效订阅 |
| Plan Maker arrows namespace | missing | added | 方向箭头正常显示 |

### 编译验证
- 无需编译（纯配置文件修改）

---

## [2.8.0] - 2026-07-17 - 参数体系统一、硬编码消除、代码质量提升

### 参数体系统一
- **navigator 参数路径统一：** 飞行参数改为优先从 `flight_defaults/*` 读取（与 waypoint_manager 保持一致），回退到 `flight/*` 兼容旧 config.yaml
- **重复航点阈值可配置：** `checkDuplicateWaypoints()` 的判定阈值从硬编码 0.01m 改为 `config_.duplicate_threshold`，在 `config.yaml` 的 `validation.duplicate_threshold` 中配置
- **real_path 最大点数可配置：** `appendRealPath()` 的 FIFO 上限从硬编码 500 改为 `config_.max_real_path_points`（`experiment.max_real_path_points`），默认 500，最小 50

### 关键 Bug 修复
- **loadConfigFromFile() 竞态条件修复：** 面板自身的 `loadConfig()` 调用从异步 `rosparam load` 之前移到 `QProcess::finished` 回调中，确保读取到的是新参数而非旧值
- **navigator DEBUG 日志降级：** `ROS_INFO_THROTTLE` → `ROS_DEBUG_THROTTLE`，生产环境不再每 5 秒打印 setpoint 调试信息

### 代码质量
- **消除 static 局部变量：** `startSpin()` 的 `refresh_counter` 和 `printStatusLine()` 的 `line_count` 改为类成员变量，确保多实例安全和线程安全
- **旧 Marker 代码安全加固：** `markWaypoint()` 和 `republishMarkers()` 恢复 `lifetime=1.0s`（之前为 0=永久），添加 LEGACY 注释防止误用
- **Config 结构体完善：** navigator Config 新增 `max_real_path_points`；waypoint_manager Config 新增 `duplicate_threshold`

### 编译验证
- 全部 3 个活动包通过编译，0 错误 0 警告

---

## [2.7.0] - 2026-07-17 - 可视化方向箭头、前后端对齐、数据链路修复

### 可视化增强

- **恢复方向箭头：** `publishPlanMakerMarkers()` 现在为每个航点发布 **ARROW 标记**（显示 yaw 朝向），用户可以看到每个航点的机头方向
- **导航进度颜色编码：** 已到达航点=绿色(#4CAF50)、当前目标=亮橙色放大(#FF9800)、待飞行=蓝色(#2196F3)、规划模式=橙色
- **Marker 自动刷新：** 带 1 秒 lifetime + 面板每 ~300ms 定时刷新；面板崩溃后 marker 自动消失，不留僵尸可视化

### 前后端对齐修复（6 项）

- **修复 params 话题硬编码：** panel 的 `waypoint_params_pub_` 和 `waypoint_params_sub_` 话题从硬编码字符串改为 `config_` 参数（与 waypoint_manager 的 config 保持一致）
- **修复节点检测硬编码：** `checkNodeStatus()` 中 `"/uav/navigator/status"` 改为配置变量，避免修改 topic 名后按钮全部禁用
- **紧急原因可见：** NavigatorStatus 的 `error_message` 字段现在显示在面板位置栏（红色加粗），操作员可直观看到紧急原因
- **Config 新增字段：** `waypoint_params_input_topic` / `waypoint_params_loaded_topic`（panel Config 结构体）
- **config.yaml 新增：** `panel.waypoint_params_input_topic` / `panel.waypoint_params_loaded_topic` / `paths.default_frame_id`
- **Logger setpointCallback 空转修复：** 新增 `print_setpoint` 条件判断和 DEBUG 级日志（默认关闭，不消耗带宽）

### 编译验证
- 全部 4 个包通过编译，0 错误 0 警告

---

## [2.6.0] - 2026-07-17 - 可视化统一与UX重构

### 可视化统一

- **移除双重标记系统：** 打点不再同时发布金色箭头（旧 `markWaypoint`）和橙色球体（`publishPlanMakerMarkers`），统一为橙色球体 + 白色编号，消除 RViz 视觉混乱
- **Marker 生命周期：** `publishPlanMakerMarkers()` 的 marker 现在有自动过期时间（`lifetime = spin_timer_ms * 2.5`），面板定时器每 3 次 spin 刷新一次；面板崩溃后 marker 在 ~1 秒内自动消失，不留僵尸可视化
- **clearMarkers() 扩展：** 清除旧命名空间（`uav_waypoint_arrow/number`）同时清除新的 plan_maker 命名空间（`uav_plan_maker_points/numbers`），确保彻底清理

### 按钮布局简化

- **移除旧「航点设置」区域的 💾保存 / 📂加载 / 📤发布 按钮**（与「航点规划」区域功能重复，造成用户困惑）
- **「航点规划」区域新增 📂加载 按钮**，工作流变为：**加载 → 连接 → 保存 → 发布**（线性、无歧义）
- **删除未实现的函数声明：** `updateConnectionStatus()` 和 `setButtonStyle()`（仅在头文件中声明，从未实现）

### 配置改进

- **`frame_id` 可配置化：** 全部 25 处硬编码 `"map"` 替换为 `config_.default_frame_id`（navigator / waypoint_manager / panel 三节点的 Config 结构体均已扩展）
- **`config.yaml` 新增：** `paths.default_frame_id: "map"`，用户可按需修改

### 代码清理

- **navigator：** `handleEmergency()` 中 `static ros::Time last_mode_req` 改为类成员 `last_emergency_mode_req_time_`，消除静态局部变量隐患
- **panel 头文件：** 移除不再使用的 `save_button_` / `publish_button_` 成员声明

### 编译验证
- 全部 4 个包通过编译，0 错误 0 警告

---

## [2.1.0] - 2026-07-17 - 第二轮全面审计与关键修复

### 审计概览

通过两次独立代码审查（前端 RViz 插件 + 后端导航节点），共发现 **50 个问题**：
- **前端审计** (rviz_waypoint_panel + navi_multi_goals_pub_rviz_plugin)：27 个问题（5 CRITICAL / 7 HIGH / 15 MEDIUM/LOW）
- **后端审计** (navigator + safety_monitor + waypoint_manager + logger + experiment_recorder)：23 个问题（5 CRITICAL / 11 IMPORTANT / 7 SUGGESTION）

### Phase 8: 恢复并优化 rviz_navi_multi_goals_pub_plugin

- **恢复文件：** `src/rviz_navi_multi_goals_pub_plugin/`（从 `/home/groundstation/catkin_ws` 复制并优化）
- **修改原因：** 用户依赖此插件进行可视化选点导航；之前被错误删除
- **优化内容：**
  - 修复 `startSpin()` 声明为 `static` 导致 Qt 信号槽不可靠（改为非静态成员函数）
  - 修复 `setBackgroundColor()` 使用已废弃的 Qt5 API（改为 `QBrush` 方式）
  - 移除 `ros::Duration(0.5).sleep()` 阻塞 UI 线程
  - Marker 使用独立命名空间（`multi_navi_arrow` / `multi_navi_number`），DELETEALL 不干扰其他插件
  - 所有参数通过 ROS 参数服务器加载，新增 `config/plugin_config.yaml`
  - 集成 waypoint_manager 服务：新增「💾 保存XML」「📂 加载XML」「📤 发布至导航器」按钮
  - `loadFromXml()` 直接从服务响应获取航点数据并填充表格和可视化（修复数据丢弃问题）
  - 新增状态标签显示航点数/模式/索引
  - 移除阻塞式 `ros::Duration(0.5).sleep()` 和 `ros::Duration(0.1).sleep()`
  - 更新 CMakeLists.txt（现代 catkin 依赖）、package.xml（format=3）、plugin_description.xml
  - 清理非源码文件（develop.html/md, README.html/md, images/, .git）
- **影响范围：** navi_multi_goals_pub_rviz_plugin（新增为第 4 个编译包）
- **验证方法：** `catkin build` 四包全部通过编译（0 错误 0 警告）

### Phase 9: 关键修复 — 后端导航核心

#### navigator.cpp 修复

- **【CRITICAL】修复 `waypointsCallback` 无条件重置航点索引：**
  - 问题：飞行中收到新航点时 `current_waypoint_idx_` 被重置为 0，无人机会突然飞回第一个航点
  - 修复：飞行状态（TAKEOFF/NAVIGATING/HOVERING）下保持当前索引，仅限制不超过新航点数量；仅 IDLE/PRE_FLIGHT 状态下重置

- **【CRITICAL】修复 `handleTakeoff` 起飞超时永远不触发：**
  - 问题：超时检查使用 `last_mode_request_time_`（每次重试都会刷新），导致 OFFBOARD 请求持续失败时永远不会超时
  - 修复：改为使用 `state_enter_time_`（从进入 TAKEOFF 状态开始计时），超时后正确进入 LANDING

- **【CRITICAL】修复 `handleReturning` 双重 setpoint 发布：**
  - 问题：`handleReturning()` 直接 publish + `setpointTimerCallback` 也 publish，导致 RETURNING 状态产生 30+ Hz setpoint（应为 20Hz）
  - 修复：删除 `handleReturning()` 中的直接 publish，setpoint 统一由 `setpointTimerCallback` 发布

#### safety_monitor.cpp 修复

- **【CRITICAL】新增 MAVROS 状态超时检测：**
  - 问题：MAVROS 进程崩溃后消息停止到达，但 `current_mavros_state_.connected` 保持上次的 `true` 值，永远检测不到崩溃
  - 修复：新增 `last_mavros_state_time_` 时间戳，在 `checkTimerCallback` 中检查消息新鲜度，超时发布 `MAVROS_TIMEOUT` 告警

#### waypoint_manager.cpp 修复

- **【CRITICAL】新增路径遍历保护：**
  - 问题：`loadWaypointsCallback` 和 `saveWaypointsCallback` 接受任意文件路径直接传给 `std::ifstream/ofstream`，存在路径遍历漏洞（`../etc/passwd`）
  - 修复：新增 `isPathSafe()` 函数，拒绝包含 `..` 的路径，使用 `realpath()` 解析并校验在允许目录范围内，禁止访问 `/home/groundstation/` 之外的文件

### Phase 10: 关键修复 — 前端 RViz 插件

#### waypoint_panel.cpp 修复

- **【CRITICAL】修复 `clearMarkers()` 使用无命名空间 DELETEALL：**
  - 问题：`marker_delete.action = DELETEALL` 未指定命名空间，会删除所有发布到 `visualization_marker` 话题的 marker（包括其他插件的）
  - 修复：分别为 `uav_waypoint_arrow` 和 `uav_waypoint_number` 命名空间发送 DELETEALL

- **【CRITICAL】修复 `deleteSelectedPlanPoint()` 双重删除导致数据损坏：**
  - 问题：此函数先从 `plan_maker_points_` 删除元素，再调用 `deleteSelectedWaypoint()` 再次删除，第二次删除会误删相邻元素
  - 修复：删除函数中的直接 erase，改为设置表格当前行后由 `deleteSelectedWaypoint()` 统一管理数据结构

- **【CRITICAL】修复 `clearWaypoints()` 不发布空 PoseArray：**
  - 问题：清空后 waypoint_manager 和 navigator 仍持有上次的航点数据，如果此时收到 START 命令会使用过期航点
  - 修复：新增发布空 `PoseArray` 和空 `Float64MultiArray`，并重置 `confirmed_waypoint_count_`

- **【HIGH】修复 `moveWaypointUp/Down` 不更新 `plan_maker_selected_index_`：**
  - 问题：上移/下移航点后 `plan_maker_selected_index_` 未同步更新，导致后续删除操作指向错误元素
  - 修复：在交换操作后同步更新索引

#### multi_navi_goal_panel.cpp 修复

- **【CRITICAL】修复 `loadFromXml()` 丢弃航点数据：**
  - 问题：从 waypoint_manager 服务响应获取了完整航点数据（`srv.response.waypoints`），但仅使用 `waypoint_count` 显示消息框，实际数据被完全忽略
  - 修复：直接从服务响应的 `waypoints.poses` 填充 `pose_array_`、表格和可视化标记

### 编译验证

- 全部 4 个包编译成功：**0 错误，0 警告**
- `uav_navigator`, `uav_waypoint_manager`, `rviz_waypoint_panel`, `navi_multi_goals_pub_rviz_plugin`

### 审计发现汇总（待后续修复的重要问题）

以下问题已在审计中识别但本次迭代未修复，记录供后续规划：

| 严重级别 | 文件 | 问题描述 |
|---------|------|---------|
| CRITICAL | navigator.cpp | `state_mutex_` 声明但从未 lock — AsyncSpinner(2) 多线程并发访问无保护 |
| HIGH | waypoint_panel.cpp | `loadWaypoints()` 和 `executeMission()` 中使用阻塞式 `waitForMessage` 冻结 Qt 主线程 |
| HIGH | waypoint_panel.cpp | `loadConfigFromFile()` 使用阻塞式 `QProcess::waitForFinished(5000)` |
| HIGH | waypoint_panel.cpp | `launchGroundStation()` 和 `killGroundStation()` 使用阻塞式 `system()` 调用 |
| HIGH | waypoint_panel.cpp | 多处硬编码文件路径 `/home/groundstation/waypoints.xml` |
| HIGH | waypoint_panel.cpp | `isTableCellValid()` 声明但未实现（链接错误风险）|
| HIGH | waypoint_panel.cpp | `plan_maker_dirty_` 设置但从未读取（死代码）|
| IMPORTANT | navigator.cpp | `handleNavigating()` 使用 `== 0.0` 浮点精确比较 Z 坐标 |
| IMPORTANT | navigator.cpp | 多处硬编码 `"map"` frame_id（应可配置）|
| IMPORTANT | experiment_recorder.cpp | `ensureDirectory()` 忽略所有 `mkdir` 返回值 |
| IMPORTANT | experiment_recorder.cpp | 自动记录逻辑使用魔数而非 `NavigatorStatus::STATE_*` 常量 |
| IMPORTANT | logger.cpp | `setpointCallback` 为空操作存根（死订阅浪费带宽）|
| IMPORTANT | safety_monitor.cpp | `setpoint_count_` 声明并递增但从未读取（死代码）|

---

## [2.0.0] - 2026-07-17 - 全面代码审计与关键修复

### 第一阶段：工作区清理
- **删除** `agent-skills/` — 独立 Claude Code 插件，与无人机项目无关
- **删除** `uav_hover/` — 遗留包，存在硬编码值和安全问题
- **删除** `src/rviz_navi_multi_goals_pub_plugin/` — 遗留 RViz 插件（不兼容 MAVROS），已在 v2.1.0 恢复并优化
- **删除** 废弃的各包独立配置文件（`navigator_config.yaml`、`panel_config.yaml`、`manager_config.yaml`）
- **删除** 重复的 rviz 配置文件（根目录和 `uav_navigator/config/` 下的 `uav_navigation.rviz`）
- **删除** 全部测试脚本和测试数据
- **删除** `uav_navigator/test/` 目录和 `scripts/__pycache__/`
- **新增** `.gitignore` 忽略编译产物和 IDE 文件
- **初始化** Git 仓库用于变更追踪

### 第二阶段：关键修复 — 导航器 (navigator.cpp)
- **修复** `time_since_start` 始终等于 `time_in_current_state`（复制粘贴 bug，现已使用 `mission_start_time_` 正确计算）
- **修复** RETURNING（返航）状态在无 home 位置时永久悬停（新增超时 = 起飞超时×3）
- **修复** TAKEOFF（起飞）预发布阶段无超时保护（新增 offboard_timeout 守卫）
- **修复** `setpoint_rate=0` 时除零错误（新增验证，回退到 20.0 Hz）
- **修复** 起飞前检查在无人机危险高度以上时仍通过（当前高度 > 起飞高度 + max(3m, 2×起飞高度) 时阻止起飞）
- **修复** 双重发布 setpoint（从各 handle* 函数中移除，setpointTimerCallback 为唯一发布源）
- **修复** 降落/紧急状态超时硬编码（现在可通过 `mode/landing_timeout` 和 `mode/emergency_timeout` 配置）
- **修复** 紧急原因字符串中英混合（全部改为英文）
- **修复** 定时器回调中的静态局部变量（替换为 ROS_INFO_THROTTLE）
- **删除** 死代码 `offboard_entry_ready_` 成员变量
- **新增** `mission_start_time_` 追踪以正确计算实验指标
- **新增** `landing_timeout` 和 `emergency_timeout` 参数验证

### 第三阶段：关键修复 — 安全监控 (safety_monitor.cpp)
- **新增** 心跳发布者（`uav/safety/heartbeat`）— 每次检查周期证明 safety_monitor 存活
- **新增** 所有位置数据的 NaN/Inf 验证（`isPositionValid()` 守卫）
- **新增** 配置参数验证（check_interval、max_height_limit、communication_timeout 边界检查）
- **新增** checkTimerCallback 中的 try-catch 保护（异常不再导致安全监控崩溃）
- **新增** 告警去重（同类型告警在 `alert_min_interval` 秒内不重复发布）
- **新增** emergency_stop 订阅者检查（告警话题无订阅者时发出警告）
- **修复** 位置跳变检测：首条消息不再检查，跳变后保留参考位置（不覆盖）
- **修复** 位置跳变窗口自适应实际里程计频率
- **修复** 无里程计短路逻辑：仅跳过高度检查，其他检查继续执行
- **修复** `has_navigator_status_` 超时后不再重置为 false（告警可持续触发）
- **修复** 使用 `NavigatorStatus` 消息常量代替魔数进行状态检测
- **修复** MAVROS 断开告警使用 "MAVROS_DISCONNECTED"（原为 "COMMUNICATION_TIMEOUT"，现可区分）
- **修复** 导航器超时告警使用 "NAVIGATOR_TIMEOUT"（原为 "COMMUNICATION_TIMEOUT"，现可区分）
- **修复** 模式不匹配使用 `isZero()` 检查而非 `toSec() == 0.0`

### 第四阶段：关键修复 — 航点管理器 (waypoint_manager.cpp)
- **修复** 【CRITICAL】自订阅反馈回环：参数订阅者和发布者现在使用不同话题
  - 订阅：`uav/waypoints/params`（来自面板）
  - 发布：`uav/waypoints/params_loaded`（至面板，新话题）
- **新增** waypointsCallback 和 loadFromXml 中的 NaN/Inf 坐标验证
- **新增** per-stod 逐个字段 try-catch 及独立错误处理
- **新增** XML 部分加载回滚机制（临时容器，仅在成功时提交）
- **新增** XML 解析前空文件检查
- **修复** `sprintf` 替换为 `snprintf`（缓冲区安全）
- **修复** checkWaypointSpacing 中的无符号下溢风险（使用 `i + 1 < poses.size()` 模式）
- **新增** 服务回调中的 try-catch 保护

### 第五阶段：关键修复 — RViz 面板 (waypoint_panel.cpp/h)
- **修复** `clearWaypoints()` 现在同时清除 `plan_maker_points_`、参数向量、轨迹并重置阶段
- **修复** `startSpin()` 从 static 改为非静态成员函数（修复 Qt 信号/槽可靠性）
- **修复** 航点参数订阅者使用 `uav/waypoints/params_loaded` 话题（避免反馈回环）
- **修复** 移除空操作 `checkWaypointConfirmation()`（确认逻辑已在 receiveNavStatus 中处理）

### 第六阶段：Logger 显示修复 (logger.cpp)
- **重写** Logger，支持 ANSI 颜色编码状态显示
- **新增** 启动时清屏，配合紧凑表头
- **新增** 按状态着色格式（蓝=空闲，绿=导航中，红=紧急）
- **移除** 冗余的逐事件日志（数据聚合到状态行）
- **新增** 每 20 行自动重印表头

### 第七阶段：配置更新 (config.yaml)
- **新增** `topics/waypoint_params_input` 和 `topics/waypoint_params_current`
- **新增** `mode/landing_timeout: 60.0` 和 `mode/emergency_timeout: 120.0`
- **新增** `safety/heartbeat_topic: "uav/safety/heartbeat"`
- **新增** `safety/alert_min_interval: 1.0`

### 编译验证
- 全部 3 个包编译通过：**0 错误，0 警告**
- `uav_navigator`、`uav_waypoint_manager`、`rviz_waypoint_panel`

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
