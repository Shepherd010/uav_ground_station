# uav_waypoint_manager — 航点持久化包

航点数据的持久化存储层，负责 XML 格式的保存/加载、航点验证、以及向 navigator 和 panel 发布航点数据。

## 架构

```
┌──────────────────────────────────────────────────────────┐
│                  uav_waypoint_manager                     │
│                                                           │
│   ┌─────────────┐    ┌──────────────┐    ┌────────────┐ │
│   │ XML 读写    │    │  航点验证     │    │ 发布/订阅   │ │
│   │ saveToXml() │    │ validate()   │    │ pub/sub    │ │
│   │ loadFromXml()│   │ duplicate✓   │    │            │ │
│   └──────┬──────┘    │ spacing✓     │    └─────┬──────┘ │
│          │           │ height✓      │          │         │
│          │           │ coordinate✓  │          │         │
│          │           └──────────────┘          │         │
│          └────────────────────────────────────┘         │
└──────────────────────────────────────────────────────────┘
```

## 数据流

```
 panel                     waypoint_manager                  navigator
 ──────                    ────────────────                  ─────────
                                                                  
 publishWaypoints() ───→ uav/waypoints/input ───→ waypointsCallback()
                          (PoseArray)               │
                                                    ├→ 验证 (NaN/Inf/间距/重复/高度)
 publish params ───────→ uav/waypoints/params ───→  waypointParamsCallback()
                          (Float64MultiArray)        │
                                                    ├→ waypoints_pub_.publish()
                                                    │   └→ uav/waypoints/current ──→ navigator waypointsCallback()
                                                    │
 loadWaypoints() ───→ load_waypoints srv ──→ loadWaypointsCallback()
                                          ←── response {waypoints, success}
                         (panel 直接从 response 获取数据，不用 waitForMessage)
                                                    │
                                                    └→ params_current_pub_.publish()
                                                        └→ uav/waypoints/params_loaded ──→ panel receiveWaypointParams()
```

### 话题

| 话题 | 方向 | 消息类型 | 说明 |
|------|------|---------|------|
| `uav/waypoints/input` | panel → manager | `PoseArray` | 接收面板发布的原始航点 |
| `uav/waypoints/params` | panel → manager | `Float64MultiArray` | 接收面板发布的 per-waypoint 参数 (hover_time, speed) |
| `uav/waypoints/current` | manager → navigator | `PoseArray` (latched) | 发布验证后的航点到导航器 |
| `uav/waypoints/params_loaded` | manager → panel | `Float64MultiArray` (latched) | 发布存储的 per-waypoint 参数到面板（独立话题，避免自订阅回环） |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `uav/waypoint_manager/load_waypoints` | `LoadWaypoints` | 从 XML 加载航点，响应直接返回 PoseArray |
| `uav/waypoint_manager/save_waypoints` | `SaveWaypoints` | 保存航点到 XML 文件 |
| `uav/waypoint_manager/clear_waypoints` | `ClearWaypoints` | 清除所有航点 |

## XML 格式

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
    <hover_time>5.0</hover_time>   <!-- 在该点悬停时间（秒），可选，默认 5.0 -->
    <speed>2.0</speed>             <!-- 飞行速度（m/s），可选，默认 2.0 -->
    <orientation>                  <!-- 四元数（从 yaw 自动计算） -->
      <x>0.0</x>
      <y>0.0</y>
      <z>0.0</z>
      <w>1.0</w>
    </orientation>
  </waypoint>
  <!-- 更多 waypoint ... -->
</waypoints>
```

## 航点验证规则

| 验证项 | 条件 | 行为 |
|--------|------|------|
| 坐标有效性 | NaN/Inf 检查 | **拒绝**：包含无效坐标的航点返回 false |
| 高度检查 | `min_height ≤ z ≤ max_height` | **警告**：超出范围仅警告，不阻止 |
| 航点间距 | 相邻航点距离 ≥ `min_waypoint_spacing` | **警告**：间距过小仅警告 |
| 重复航点 | 任意两航点距离 < `duplicate_threshold` | **警告**：仅警告（可能有意悬停） |

**设计原则**：验证是非阻塞的（warn, don't block），仅对数据完整性问题（NaN/Inf）硬拒绝。

## 路径安全

XML 文件读写受路径遍历保护：
- 禁止包含 `..` 的相对路径
- 使用 `realpath()` 解析后检查是否在 `/home/groundstation` 范围内
- 加载失败时回滚到加载前的状态

## 命令行操作

```bash
# 加载航点
rosservice call /uav/waypoint_manager/load_waypoints "{file_path: '/home/groundstation/waypoints.xml'}"

# 保存航点
rosservice call /uav/waypoint_manager/save_waypoints "{file_path: '/home/groundstation/waypoints.xml'}"

# 清除航点
rosservice call /uav/waypoint_manager/clear_waypoints

# 查看当前航点
rostopic echo /uav/waypoints/current
```

## 配置

所有参数从 `config.yaml` 全局命名空间加载：

| 参数路径 | 默认值 | 说明 |
|---------|--------|------|
| `validation/min_waypoint_spacing` | 0.3 | 最小航点间距 (m) |
| `validation/duplicate_threshold` | 0.01 | 重复判定阈值 (m) |
| `validation/max_height` | 2.0 | 建议最大高度 (m) |
| `validation/min_height` | 0.5 | 建议最小高度 (m) |
| `publish_rate` | 1.0 | 发布频率 (Hz) |
| `paths/default_save` | `/home/groundstation/waypoints.xml` | 默认保存路径 |
| `paths/default_load` | `/home/groundstation/waypoints.xml` | 默认加载路径 |
| `flight_defaults/hover_duration` | 5.0 | 全局默认悬停时间 (s) |
| `flight_defaults/travel_speed` | 2.0 | 全局默认飞行速度 (m/s) |

## 依赖

- `uav_navigator`（无直接代码依赖，但 navigator 是航点数据的消费者）
- `geometry_msgs` / `std_msgs` / `tf`
