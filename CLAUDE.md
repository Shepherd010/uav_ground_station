# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace Overview

This is a **ROS (Robot Operating System) catkin workspace** (Noetic, catkin_tools v0.9.4) for a UAV ground station that controls PX4-based drones via MAVROS. The system is split into two responsibilities:

1. **Ground station (this workspace)** — waypoint annotation, mission control, navigator state machine, safety monitoring, and RViz visualization.
2. **Onboard computer (drone)** — DLIO SLAM + MAVROS bridge. This is started by the user via `~/uav_scripts/start_full.sh` on the onboard computer (`uav@192.168.31.180`).

**Active packages:**
1. **`uav_navigator`** — State-machine-driven navigation core + safety monitor + logger. Communicates with MAVROS to manage OFFBOARD mode, arming, takeoff, waypoint navigation, hovering, landing, and emergency states.
2. **`uav_waypoint_manager`** — Waypoint persistence layer. Handles XML save/load, waypoint validation (duplicate detection, spacing checks, height range checks), and PoseArray publishing.
3. **`rviz_waypoint_panel`** — Fixed RViz panel plugin (Qt5). Receives `move_base_simple/goal`, visualizes waypoints with arrows+numbers, edits (x,y,z,yaw), supports save/load via service calls, shows real-time nav status, and has system control buttons.

**LEGACY (retained as backups, do not use):**
- `uav_hover/src/hover/` — Old MAVROS nodes with hardcoded values and safety issues.
- `src/rviz_navi_multi_goals_pub_plugin/` — Old RViz plugin for ground-robot `move_base` (incompatible with MAVROS).

The `agent-skills/` directory is an independent Claude Code plugin project with its own `CLAUDE.md`.

## Build System

```bash
# Build all packages in the root workspace
catkin build
# Or: catkin_make

# Source the workspace
source devel/setup.bash

# Clean build artifacts
rm -rf build/ devel/ logs/ .catkin_tools/

# Build a single package
catkin build --no-deps uav_navigator
```

## Operational Launchers

The old "phase 1/2/3/5" naming has been removed. Ground station operation uses two launchers:

| Responsibility | Command | Location | Notes |
|---|---|---|---|
| Ground station core | `./scripts/start_ground_station.sh` or `roslaunch uav_navigator ground_station.launch` | Ground station | Starts roscore (if needed), navigator, safety_monitor, waypoint_manager |
| Visualization & waypoint annotation | `./scripts/start_rviz.sh` or `roslaunch rviz_waypoint_panel rviz_ground_station.launch` | Ground station | Starts RViz with the waypoint panel plugin |
| Execute mission | `./scripts/start_mission.sh <waypoint_file.xml>` | Ground station | Loads waypoints and sends START command |

### Onboard setup

The onboard computer (`uav@192.168.31.180`) runs the SLAM + MAVROS link. The default environment on the onboard machine points its ROS master to the ground station:

```bash
export ROS_MASTER_URI=http://192.168.31.30:11311
export ROS_IP=192.168.31.180
export ROS_HOSTNAME=192.168.31.180
```

The ground station scripts default to:

```bash
export ROS_MASTER_URI=http://192.168.31.30:11311
export ROS_IP=192.168.31.30
export ROS_HOSTNAME=groundstation
```

These defaults can be overridden by setting the environment variables before running the scripts.

### Usage flow

1. **Start the onboard link** (on the drone, via SSH):
   ```bash
   ssh uav@192.168.31.180
   ~/uav_scripts/start_full.sh
   ```
2. **Start the ground station core** (on the ground station):
   ```bash
   cd /home/groundstation/catkin_ws
   ./scripts/start_ground_station.sh
   ```
3. **Open RViz for waypoint annotation** (on the ground station):
   ```bash
   ./scripts/start_rviz.sh
   ```
4. **Create / load waypoints** in the RViz panel, save to XML.
5. **Execute the mission**:
   ```bash
   ./scripts/start_mission.sh /home/groundstation/waypoints.xml
   ```

## Package Details

### `uav_navigator` — Navigation Core

**State machine:** `IDLE → PRE_FLIGHT → ARMING → TAKEOFF → NAVIGATING → HOVERING → LANDING → LANDED`. Emergency states: `EMERGENCY`, `RETURNING`.

**Nodes:**
- `navigator` — 10-state machine, publishes `mavros/setpoint_position/local` at 20Hz, handles OFFBOARD/ARM/LAND mode switching via services.
- `safety_monitor` — Independent node monitoring communication timeout, height limits, MAVROS connection, mode anomalies. Publishes alerts to `/uav/safety/alert`.
- `logger` — Subscribes to key topics and prints a formatted status summary to the terminal.

**All parameters** are loaded from `uav_navigator/config/navigator_config.yaml` (via `rosparam load`). No hardcoded values in the source code.

**Key topics:**
- Subscribe: `mavros/state` (State), `mavros/local_position/odom` (Odometry), `uav/waypoints/current` (PoseArray), `uav/safety/alert` (String)
- Publish: `mavros/setpoint_position/local` (PoseStamped, 20Hz), `uav/navigator/status` (NavigatorStatus)
- Service: `mavros/cmd/arming` (CommandBool), `mavros/set_mode` (SetMode), `/uav/navigator/command` (NavigatorCommand)

### `uav_waypoint_manager` — Waypoint Persistence

**Node:** `waypoint_manager` — Receives PoseArray from the panel, validates, saves to XML, loads from XML, publishes current waypoints.

**Services:**
- `/uav/waypoint_manager/load_waypoints` (LoadWaypoints) — loads from XML file path
- `/uav/waypoint_manager/save_waypoints` (SaveWaypoints) — saves to XML file path
- `/uav/waypoint_manager/clear_waypoints` (ClearWaypoints) — clears all waypoints

**XML format:**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<waypoints>
  <metadata>
    <created>2026-06-24 10:30:00</created>
    <frame_id>map</frame_id>
    <count>3</count>
  </metadata>
  <waypoint id="1">
    <x>1.0</x><y>0.0</y><z>2.0</z><yaw>0.0</yaw>
  </waypoint>
</waypoints>
```

### `rviz_waypoint_panel` — RViz Plugin

**Plugin class:** `rviz_waypoint_panel::WaypointPanel` (base: `rviz::Panel`)

**UI:** Status display area (state, current waypoint, position), max-goals input, waypoint table (x,y,z,yaw), delete/up/down buttons, save/load buttons, publish/start-nav/emergency-stop buttons, system control buttons (launch/kill ground station, MAVROS status LED).

**Topics:**
- Subscribe: `move_base_simple/goal` (PoseStamped, from RViz 2D Nav Goal tool), `uav/navigator/status` (NavigatorStatus)
- Publish: `visualization_marker` (Marker, arrows + numbers), `uav/waypoints/input` (PoseArray)
- Service clients: `uav/waypoint_manager/save_waypoints`, `uav/waypoint_manager/load_waypoints`

**All parameters** from `rviz_waypoint_panel/config/panel_config.yaml`.

## Configuration Files

| File | Purpose |
|------|---------|
| `uav_navigator/config/navigator_config.yaml` | Namespace, topic names, flight params (takeoff height, hover duration, setpoint rate, thresholds), mode-switch timeouts, safety limits |
| `uav_waypoint_manager/config/manager_config.yaml` | Topic names, file paths, waypoint validation thresholds |
| `rviz_waypoint_panel/config/panel_config.yaml` | Goal/marker/status topics, marker colors/scales, table defaults, spin timer interval |
| `rviz_waypoint_panel/config/uav_navigation.rviz` | RViz display configuration (also kept at repository root as `uav_navigation.rviz`) |

## Service Commands

```bash
# Start navigation (requires waypoints loaded first)
rosservice call /uav/navigator/command "{command: 'START'}"

# Pause (hover at current position)
rosservice call /uav/navigator/command "{command: 'PAUSE'}"

# Cancel and land
rosservice call /uav/navigator/command "{command: 'CANCEL'}"

# Land immediately
rosservice call /uav/navigator/command "{command: 'LAND'}"

# Return to home and land
rosservice call /uav/navigator/command "{command: 'RETURN_TO_HOME'}"

# Emergency stop (immediate, highest priority)
rosservice call /uav/navigator/command "{command: 'EMERGENCY_STOP'}"

# Reset from EMERGENCY or LANDED
rosservice call /uav/navigator/command "{command: 'RESET'}"

# Load waypoints from XML
rosservice call /uav/waypoint_manager/load_waypoints "{file_path: '/home/groundstation/waypoints.xml'}"

# Save waypoints to XML
rosservice call /uav/waypoint_manager/save_waypoints "{file_path: '/home/groundstation/waypoints.xml'}"

# Emergency stop via safety monitor
rosservice call /uav/safety/emergency_stop
```

## Architecture Rules

1. **No hardcoded values.** All topics, thresholds, timeouts, rates, and colors are loaded from YAML config files via `ros::NodeHandle("~")` / `pnh`. Every parameter has a default fallback.
2. **Safety monitor is independent.** It runs as a separate node and can trigger emergency actions even if the navigator crashes.
3. **State machine is explicit.** All state transitions in `navigator.cpp` go through `transitionState()`. Every state has enter/exit logic and timeout handling.
4. **Setpoints are published via `ros::Timer`.** Never use `ros::Duration::sleep()` or blocking calls in the main thread. The setpoint timer runs at the configured rate (default 20Hz) independently of the state machine timer (10Hz).
5. **Namespace consistency.** All topic names in config files are relative. The `namespace` parameter in `navigator_config.yaml` determines the root namespace. Do not use absolute paths with leading `/` in topic names.
6. **Waypoint validation.** The waypoint manager validates duplicates, spacing, and height range before publishing. Warnings are logged but do not block execution (non-fatal validation).

## Common Development Tasks

### Adding a new config parameter
1. Add the parameter with default value to the YAML config file.
2. Add `pnh_.param<T>("path/to/param", config_.field, default)` in the node's `loadConfig()` method.
3. Use `config_.field` in the code.
4. Update `README.md` and `CHANGELOG.md`.

### Adding a new state to the navigator state machine
1. Add the state enum to `Navigator` class in `navigator.cpp`.
2. Add the state name to `stateToString()`.
3. Add `handle<NewState>()` method.
4. Add case in `processStateMachine()` switch.
5. Add enter logic in `transitionState()`.
6. Add transition conditions from relevant existing states.

### Modifying the RViz plugin UI
1. Edit `waypoint_panel.h` to add new widgets and slots.
2. Edit `waypoint_panel.cpp` constructor for layout, `connect()` signals, and slot implementations.
3. Rebuild with `catkin build --no-deps rviz_waypoint_panel`.
4. Restart RViz to see changes.

## Debugging

```bash
# Check navigator status
rostopic echo /uav/navigator/status

# Check safety alerts
rostopic echo /uav/safety/alert

# Check MAVROS state
rostopic echo /mavros/state

# Check current position
rostopic echo /mavros/local_position/odom

# Check waypoints
rostopic echo /uav/waypoints/current

# List all nodes
rosnode list

# List all topics
rostopic list
```

## Important Notes

- **OFFBOARD mode requires >2Hz setpoint stream.** The navigator publishes at 20Hz, but if the timer or node dies, the drone will fall out of the sky. The safety monitor detects this via communication timeout.
- **RC takeover is always possible.** PX4 allows the RC transmitter to override OFFBOARD mode at any time. This is a hardware-level safety feature, not software.
- **The legacy `hover.cpp` node stops publishing after 100 messages.** This was a critical bug that would cause crashes in OFFBOARD mode. The new `navigator` node never stops publishing setpoints while active.
- **The legacy plugin used `move_base_simple/goal_temp` (non-standard).** The new plugin uses the standard `move_base_simple/goal` topic that RViz's 2D Nav Goal tool publishes to by default.
- **Legacy and new packages coexist.** The old `hover` and `rviz_navi_multi_goals_pub_plugin` packages are kept as backups. Do not modify them. Use the new `uav_navigator`, `uav_waypoint_manager`, and `rviz_waypoint_panel` packages instead.
- **Phase-numbered launch files are deprecated.** Use `ground_station.launch` and `rviz_ground_station.launch` instead. The old `phase*.launch` files are kept for reference but should not be used.
