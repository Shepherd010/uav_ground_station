# navi_multi_goals_pub_rviz_plugin — LEGACY（已废弃）

> ⚠️ **此包已被 `rviz_waypoint_panel` + `uav_waypoint_manager` + `uav_navigator` 替代。**
>
> 请使用新的工作流：
> 1. `roslaunch uav_navigator ground_station.launch` — 启动地面站核心
> 2. `roslaunch rviz_waypoint_panel rviz_ground_station.launch` — 启动 RViz 面板
>
> 此包仅保留作为代码参考，不应再用于实际飞行操作。

## 弃用原因

| 问题 | 说明 |
|------|------|
| 为 `move_base` 设计 | 基于地面机器人 `move_base` 架构，不兼容 MAVROS / OFFBOARD 模式 |
| 话题非标准 | 使用 `move_base_simple/goal_temp`（标准为 `move_base_simple/goal`） |
| 无安全监控 | 无独立的 safety_monitor，无高度保护、通信超时检测 |
| 无状态机 | 直接发布航点，无 PRE_FLIGHT / ARMING / TAKEOFF 等安全状态 |
| 无配置热重载 | 参数硬编码在代码中 |
| 无 per-waypoint 参数 | 不支持逐航点的悬停时间/飞行速度 |

## 与新系统对比

| 功能 | 旧包 | 新系统 |
|------|------|--------|
| 航点可视化 | 简单箭头 | 球体+方向箭头+数字编号+进度颜色编码 |
| 导航控制 | 直接发布 | 10 状态状态机 + 独立安全监控 |
| XML 持久化 | 无 | 完整 save/load + per-waypoint 参数 |
| OFFBOARD 预发布 | 无 | 100 次预发布 + 频率监控 |
| 紧急停止 | 无 | 多级安全：高度/通信/位置跳变/模式 |
| 配置管理 | 硬编码 | 统一 config.yaml + 热重载 |
