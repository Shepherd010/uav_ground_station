# Repository Guidelines

## Project Structure & Module Organization

This is a ROS Noetic catkin workspace. Keep application code under `src/`:

- `src/uav_navigator/`: navigator state machine, independent safety monitor, logger, experiment recorder, and custom `msg/`/`srv/` definitions.
- `src/uav_waypoint_manager/`: XML persistence, waypoint validation, and services.
- `src/rviz_waypoint_panel/`: Qt/RViz plugin, RViz configuration, and launch files.

The root `config.yaml` is shared runtime configuration; `scripts/` contains startup, mission, and rosbag helpers. Package documentation lives beside each package. `build/`, `devel/`, `logs/`, and `.catkin_tools/` are generated and ignored.

## Build, Test, and Development Commands

```bash
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
catkin build --no-deps uav_navigator
```

Build all packages with `catkin build`; use the targeted command for faster package iteration. Start the core with `./scripts/start_ground_station.sh`, RViz with `./scripts/start_rviz.sh`, and a mission with `./scripts/start_mission.sh ~/waypoints.xml`. Record data with `./scripts/record_bag.sh start|status|stop`. These commands require ROS Noetic, MAVROS, a running ROS master, and suitable PX4/onboard connectivity; test flight-related changes only in a controlled environment.

## Coding Style & Naming Conventions

Use C++14, four-space indentation, and the surrounding brace style. Match existing names: `PascalCase` classes, `camelCase` methods, `snake_case_` members, and lower `snake_case` ROS/YAML keys. Keep topics, thresholds, timeouts, rates, and paths in `config.yaml` with fallback defaults in code, and use relative ROS names. No repository formatter or linter is configured, so preserve nearby style and keep diffs focused.

## Testing Guidelines

No automated tests or coverage threshold are currently committed; `uav_navigator` retains only commented rostest scaffolding. At minimum, run `catkin build` and inspect ROS startup and log output. For behavior changes, exercise relevant topics/services with `rostopic` and `rosservice` using SITL or a hardware-safe setup. Add rostest coverage for new testable behavior.

## Architecture & Configuration

Route navigator transitions through `transitionState()`, publish setpoints from timers, and keep `safety_monitor` independent. Update `config.yaml`, package documentation, and `CHANGELOG.md` together when adding parameters or interfaces.

## Commit & Pull Request Guidelines

Recent history uses concise imperative or versioned subjects, sometimes with prefixes such as `build:` and `docs:`. Use a short subject that states the change, for example `fix: guard waypoint path traversal`, and keep unrelated changes separate. PRs should explain behavior and safety impact, list validation commands and results, identify configuration or interface changes, link an issue when applicable, and include RViz screenshots or logs for UI or observability changes.
