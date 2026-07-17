# rviz_waypoint_panel — RViz 航点标注面板

RViz 固定面板插件（Qt5），提供航点规划、编辑、可视化、保存/加载、导航控制、系统管理的完整人机交互界面。

## 界面布局

```
┌─ 飞行状态 ─────────────────────────────────────────────┐
│ ● IDLE  MAVROS: 已连接  已解锁  模式: OFFBOARD  航点: 2/5 │
│ [████████████████████░░░░░░░░░░░░] 40%                  │
│ 位置: 1.50, 0.20, 2.10 | 目标: 3.00, 0.00, 2.00        │
└────────────────────────────────────────────────────────┘

┌─ 配置参数 (可折叠) ────────────────────────────────────┐
│ [📂 从文件加载]                                         │
│ (参数只读显示区)                                        │
└────────────────────────────────────────────────────────┘

┌─ 航点规划 ─────────────────────────────────────────────┐
│ [📂 加载] [🔗 连接] [💾 保存] [🗑 删除] [🧹 清除]         │
│ 状态: CONNECTED | 5                                     │
└────────────────────────────────────────────────────────┘

┌─ 航点列表 ─────────────────────────────────────────────┐
│ 最大航点: [10] [确认]                                   │
│ # │ x    │ y    │ z    │ yaw° │ 悬停s │ 速度m/s        │
│ 1 │ 1.00 │ 0.00 │ 2.00 │ 0.00 │ 5.0   │ 2.0           │
│ 2 │ 3.00 │ 0.00 │ 2.00 │ 0.00 │ 3.0   │ 2.0           │
│ [🗑 删除选中] [▲ 上移] [▼ 下移] [🧹 清除全部]            │
└────────────────────────────────────────────────────────┘

┌─ 飞行控制 ─────────────────────────────────────────────┐
│ [▶ 开始任务] [⏸ 悬停] [🛬 降落]                         │
│ [🏠 返航] [🔄 重置] [🛑 紧急停止]                        │
└────────────────────────────────────────────────────────┘

┌─ 操作日志 ─────────────────────────────────────────────┐
│ [12:30:01] INFO 航点面板已就绪                           │
│ [12:30:15] INFO 接收到新规划点: (1.00, 0.00, 2.00)      │
└────────────────────────────────────────────────────────┘
```

### 飞行控制按钮语义

| 按钮 | 导航命令 | 效果 | 可用条件 |
|------|---------|------|---------|
| ▶ 开始任务 | START | 发布航点 → 解锁 → 起飞 → 依次执行 → 降落 | SAVED 阶段 |
| ⏸ 悬停 | PAUSE | 保持当前位置悬停，不降落 | NAVIGATING 阶段 |
| 🛬 降落 | LAND | 立即结束任务，AUTO.LAND 着陆 | 飞行中 |
| 🏠 返航 | RETURN_TO_HOME | 返回起飞点（TAKEOFF 记录的 Home）着陆 | NAVIGATING/HOVERING |
| 🔄 重置 | RESET | EMERGENCY/LANDED → IDLE | 任意 |
| 🛑 紧急停止 | EMERGENCY_STOP | 弹窗确认 → AUTO.LAND 紧急着陆 | 始终可用 |

## 数据流

```
┌────────────────────────────────────────────────────────────────────┐
│                          RViz Panel                                 │
│                                                                     │
│  ┌─────────────┐   ┌───────────────┐   ┌───────────────────────┐  │
│  │ 2D Nav Goal │   │ plan_maker_   │   │   waypoint_table_     │  │
│  │ (RViz tool) │──→│   points_     │←─→│   (6 列: xyz+         │  │
│  │             │   │ (唯一数据源)   │   │    yaw°+hover+speed)  │  │
│  └─────────────┘   └───────┬───────┘   └───────────────────────┘  │
│                            │                                        │
│          ┌─────────────────┼─────────────────┐                     │
│          ▼                 ▼                  ▼                     │
│   MarkerArray         nav_msgs/Path     PoseArray +                │
│   /plan_maker/points  /plan_maker/      Float64MultiArray          │
│   (球+箭头+编号)       trajectory       → /waypoints/input         │
│   → RViz 渲染         (轨迹线)          → /waypoints/params        │
│                        → RViz 渲染      → waypoint_manager         │
│                                                                     │
│  Service Clients:                                                   │
│    load_waypoints ──→ waypoint_manager (返回 PoseArray 直接渲染)   │
│    save_waypoints ──→ waypoint_manager                             │
│    nav_command ────→ navigator (START/PAUSE/CANCEL/...)            │
│                                                                     │
│  Subscribe (回显):                                                  │
│    /uav/navigator/status ──→ 状态显示 + 进度条 + marker 颜色编码    │
│    /uav/waypoints/params_loaded ──→ 表格 hover/speed 列更新        │
│    /mavros/state ──→ 连接/解锁/模式显示                            │
│    /mavros/local_position/odom ──→ 位置显示                        │
└────────────────────────────────────────────────────────────────────┘
```

## 工作流

### 标准规划→执行流程

```
1. 打点（2D Nav Goal 工具 / 手动输入表格）
   └→ plan_maker_points_ 填充 → publishPlanMakerMarkers() → RViz 显示橙色球+箭头+编号

2. 🔗 连接（≥2 个点）
   └→ publishPlanTrajectory() → RViz 显示黄色轨迹线
   └→ 状态: PLANNING → CONNECTED

3. 💾 保存
   └→ savePlanWaypoints()
   └→ 发布 PoseArray → uav/waypoints/input → waypoint_manager
   └→ 发布 Float64MultiArray → uav/waypoints/params
   └→ waypoint_manager 验证后 publish 到 uav/waypoints/current → navigator 收到
   └→ 状态: CONNECTED → SAVED
   └→ navigator 确认后自动标记（receiveNavStatus 中检查 total_waypoints 匹配）

4. ▶ 开始任务
   └→ startMission() → publishWaypoints() + nav_command START
   └→ navigator 进入 PRE_FLIGHT → ... → NAVIGATING
   └→ 状态: SAVED → NAVIGATING

5. 执行中
   └→ RViz: 已到达=绿色球, 当前目标=橙色放大球+箭头, 待飞行=蓝色球
   └→ 进度条 + 航点计数实时更新
```

### 加载已有航点执行

```
📂 加载
└→ loadWaypoints()
└→ load_waypoints_client CALL → waypoint_manager 返回 PoseArray
└→ 填充 plan_maker_points_ + 表格
└→ publishPlanMakerMarkers() → RViz 显示
└→ 自动连接轨迹 (≥2 点)
└→ 状态: CONNECTED
→ 然后继续 💾 保存 → ▶ 发布 → 执行
```

## 飞行控制按钮

| 面板按钮 | 等价命令 | 说明 |
|---------|---------|------|
| ▶ 开始任务 | `rosservice call /uav/navigator/command "{command: 'START'}"` | 发布航点并启动导航 |
| ⏸ 悬停 | `rosservice call /uav/navigator/command "{command: 'PAUSE'}"` | 保持当前位置悬停 |
| 🛬 降落 | `rosservice call /uav/navigator/command "{command: 'LAND'}"` | 立即着陆 |
| 🏠 返航 | `rosservice call /uav/navigator/command "{command: 'RETURN_TO_HOME'}"` | 返回起飞点着陆 |
| 🔄 重置 | `rosservice call /uav/navigator/command "{command: 'RESET'}"` | EMERGENCY/LANDED→IDLE |
| 🛑 紧急停止 | `rosservice call /uav/navigator/command "{command: 'EMERGENCY_STOP'}"` | 弹窗确认→紧急着陆 |
| 💾 保存 | `rosservice call /uav/waypoint_manager/save_waypoints "..."` | 航点写入 XML |
| 📂 加载 | `rosservice call /uav/waypoint_manager/load_waypoints "..."` | 从 XML 加载航点 |

## Marker 可视化系统

### 当前系统（统一 MarkerArray，topic: `/uav/plan_maker/points`）

每个航点发布三个 marker：

| Marker 类型 | namespace | 视觉 | 说明 |
|------------|-----------|------|------|
| SPHERE | `uav_plan_maker_points` | 彩色球体 | 标记航点精确位置 |
| ARROW | `uav_plan_maker_arrows` | 方向箭头 | 显示 yaw 朝向（机头方向） |
| TEXT_VIEW_FACING | `uav_plan_maker_numbers` | 数字编号 | 始终面向相机 |

**导航进度颜色编码**：

| 状态 | 颜色 | 说明 |
|------|------|------|
| 已到达 | 🟢 绿色 `#4CAF50` | `index < nav_current_waypoint_idx_` |
| 当前目标 | 🟠 亮橙色 `#FF9800` + 放大 1.5x | `index == nav_current_waypoint_idx_` |
| 待飞行 | 🔵 蓝色 `#2196F3` | `index > nav_current_waypoint_idx_` |
| 规划中 | 🟠 橙色（默认） | 未开始导航时 |

**生命周期**：marker 设 `lifetime=1.0s`，面板每 ~300ms 刷新重发。面板崩溃后 marker 1 秒内自动消失，不产生僵尸可视化。

**选中高亮**：当前选中的航点绿色通道→1.0，尺寸放大 1.3x。

### 轨迹线

| 话题 | 类型 | 颜色 | 含义 |
|------|------|------|------|
| `uav/plan_maker/trajectory` | `nav_msgs/Path` | 🟡 黄色 `#FFC107` | 规划轨迹（用户编辑） |
| `uav/trajectory/real` | `nav_msgs/Path` | 🔴 红色 `#F44336` | 实际飞行轨迹 |

## 按钮状态管理

`updateAllButtonStates()` 根据以下条件统一控制所有按钮启用/禁用：

- `navigator_running_` — navigator 节点是否存活
- `plan_maker_phase_` — 当前规划阶段（PLANNING/CONNECTED/SAVED/NAVIGATING）
- `plan_maker_points_` — 是否有航点数据

调用时机：航点变化、阶段切换、节点状态检测（每秒）、导航状态变化。

## 配置热重载

面板"📂 从文件加载"功能流程：

```
1. QFileDialog 选择 YAML
2. QProcess 异步执行 rosparam load（不阻塞 UI）
3. 完成后：
   a. loadConfig() — 更新面板本地参数
   b. publish config_loaded 事件
   c. publish config_reload → navigator + safety_monitor 重新 loadConfig()
   d. 显示参数摘要
4. 5 秒超时保护
```

## 配置

所有参数从 `config.yaml` 全局命名空间加载：

| 参数路径 | 默认值 | 说明 |
|---------|--------|------|
| `panel/goal_topic` | `move_base_simple/goal` | RViz 2D Nav Goal 话题 |
| `panel/plan_maker/sphere_scale` | 0.15 | 航点球体尺寸 |
| `panel/marker/arrow_scale_x` | 0.6 | 方向箭头长度 |
| `panel/marker/number_scale` | 0.8 | 数字标签大小 |
| `panel/spin_timer_ms` | 100 | UI 刷新间隔 (ms) |
| `panel/default_config_path` | `/home/groundstation/catkin_ws/config.yaml` | 默认配置路径 |

## 依赖

- `rviz`（面板插件框架）
- `uav_navigator`（NavigatorStatus、NavigatorCommand 消息/服务定义）
- `uav_waypoint_manager`（LoadWaypoints、SaveWaypoints 服务定义）
- Qt5（Widgets、QProcess）
- yaml-cpp（配置解析）
