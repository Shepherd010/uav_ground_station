"""Keyboard-driven curses panel for editing and running the planner."""

from __future__ import annotations

import curses
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional

from .bag_reader import find_bags, read_bag
from .config import ConfigManager
from .models import BagCandidate, BagData, PlanResult, PlannerConfig, PlanningError
from .planner import plan_route
from .xml_writer import write_waypoints_xml


DEFAULT_BAG_ROOT = Path("/home/groundstation/experiments/rosbag")


@dataclass
class Field:
    key: str
    label: str
    kind: str
    editable: bool = True


FIELDS = [
    Field("map_topic", "地图话题", "text"),
    Field("scan_topic", "逐帧点云话题", "text"),
    Field("odom_topic", "odom 话题", "text"),
    Field("input_frame_id", "输入 frame", "text"),
    Field("output_frame_id", "输出 frame", "text", editable=False),
    Field("output_path", "XML 输出路径", "text"),
    Field("cache_dir", "bag 缓存目录", "text"),
    Field("z", "规划高度 z (m)", "float"),
    Field("slice_half_height", "切片半厚度 (m)", "float"),
    Field("aircraft_length", "飞机长度 (m)", "float"),
    Field("aircraft_width", "飞机宽度 (m)", "float"),
    Field("aircraft_height", "飞机高度 (m)", "float"),
    Field("grid_resolution", "规划栅格 (m)", "float"),
    Field("map_voxel_size", "地图采样体素 (m)", "float"),
    Field("scan_voxel_size", "逐帧采样体素 (m)", "float"),
    Field("min_waypoints", "最小航点数", "int"),
    Field("max_waypoints", "最大航点数", "int"),
    Field("min_height", "最小飞行高度 (m)", "float"),
    Field("max_height", "最大飞行高度 (m)", "float"),
    Field("min_waypoint_spacing", "最小航点间距 (m)", "float"),
    Field("hover_time", "悬停时间 (s)", "float"),
    Field("speed", "飞行速度 (m/s)", "float"),
    Field("start_x", "起点 X (m)", "float"),
    Field("start_y", "起点 Y (m)", "float"),
    Field("goal_x", "终点 X (m)", "float"),
    Field("goal_y", "终点 Y (m)", "float"),
]


@dataclass
class TuiState:
    config: PlannerConfig
    bag_root: Path
    bag_path: Optional[Path] = None
    bag_data: Optional[BagData] = None
    result: Optional[PlanResult] = None
    start_x: float = 0.0
    start_y: float = 0.0
    goal_x: float = 1.0
    goal_y: float = 1.0
    status: List[str] = None

    def __post_init__(self) -> None:
        if self.status is None:
            self.status = []

    def set_status(self, *lines: str) -> None:
        self.status = list(lines)[-4:]


def _safe_add(
    screen,
    row: int,
    column: int,
    text: str,
    width: int,
    attribute: int = 0,
) -> None:
    if row < 0 or column >= width:
        return
    try:
        screen.addnstr(row, column, text, max(0, width - column - 1), attribute)
    except curses.error:
        pass


def _set_cursor(visible: bool) -> None:
    try:
        curses.curs_set(1 if visible else 0)
    except curses.error:
        pass


def _field_value(state: TuiState, field: Field) -> Any:
    if field.key in ("start_x", "start_y", "goal_x", "goal_y"):
        return getattr(state, field.key)
    return getattr(state.config, field.key)


def _format_value(value: Any) -> str:
    if isinstance(value, float):
        return "%.3f" % value
    return str(value)


def _draw_panel(
    screen,
    state: TuiState,
    selected: int,
    title: str = "2.5D 静态航点规划器",
) -> None:
    screen.erase()
    height, width = screen.getmaxyx()
    _safe_add(screen, 0, 0, title, width, curses.A_BOLD)
    bag_text = str(state.bag_path) if state.bag_path else "未选择"
    _safe_add(screen, 1, 0, "bag: %s" % bag_text, width)
    _safe_add(
        screen,
        2,
        0,
        "OCCUPIED=地图命中  FREE=射线观测  UNKNOWN=禁止规划",
        width,
        curses.A_DIM,
    )

    first_column_count = (len(FIELDS) + 1) // 2
    # Keep the panel usable on the usual 80-column SSH terminal.  The
    # parameter rows are deliberately flat; a second column is only a
    # layout choice, not a separate confirmation workflow.
    two_columns = width >= 70 and height >= first_column_count + 8
    column_width = width // 2 if two_columns else width
    for index, field in enumerate(FIELDS):
        if two_columns:
            column = 0 if index < first_column_count else 1
            row_index = index if column == 0 else index - first_column_count
        else:
            column = 0
            row_index = index
        row = 4 + row_index
        if row >= height - 6:
            continue
        column_start = column * column_width
        marker = ">" if index == selected else " "
        readonly = " (只读)" if not field.editable else ""
        text = "%s %-23s : %s%s" % (
            marker,
            field.label,
            _format_value(_field_value(state, field)),
            readonly,
        )
        attribute = curses.A_REVERSE if index == selected else 0
        _safe_add(screen, row, column_start, text, column_start + column_width, attribute)

    action_row = height - 5
    _safe_add(
        screen,
        action_row,
        0,
        "↑↓/←→选择  Enter/e编辑  p规划  x保存XML  w保存配置  a另存  l加载配置  b选bag  h帮助  q退出",
        width,
        curses.A_BOLD,
    )
    if state.result is not None:
        result = state.result
        result_text = (
            "最近结果：航点 %d | 路径 %.2fm | FREE %d | OCCUPIED %d | "
            "UNKNOWN %d | FREE连通域 %d"
            % (
                len(result.waypoints),
                result.path_length,
                result.free_cells,
                result.occupied_cells,
                result.unknown_cells,
                result.free_components,
            )
        )
        _safe_add(screen, action_row + 1, 0, result_text, width, curses.A_DIM)
    for offset, line in enumerate(state.status):
        _safe_add(screen, action_row + 2 + offset, 0, line, width)
    screen.refresh()


def _read_line(screen, prompt: str) -> Optional[str]:
    height, width = screen.getmaxyx()
    row = height - 2
    _safe_add(screen, row, 0, " " * max(1, width - 1), width)
    _safe_add(screen, row, 0, prompt, width, curses.A_BOLD)
    _set_cursor(True)
    curses.echo()
    try:
        prefix_length = min(width - 2, len(prompt))
        raw = screen.getstr(row, prefix_length).decode(
            "utf-8", errors="replace"
        )
    except (curses.error, UnicodeDecodeError):
        raw = ""
    finally:
        curses.noecho()
        _set_cursor(False)
    return raw.strip() if raw.strip() else None


def _confirm(screen, prompt: str, default: bool = False) -> bool:
    height, width = screen.getmaxyx()
    text = "%s [%s] " % (prompt, "Y/n" if default else "y/N")
    _safe_add(screen, height - 2, 0, " " * max(1, width - 1), width)
    _safe_add(screen, height - 2, 0, text, width, curses.A_BOLD)
    answer = screen.getch()
    if answer in (ord("y"), ord("Y")):
        return True
    if answer in (ord("n"), ord("N")):
        return False
    return default


def _edit_field(screen, state: TuiState, index: int) -> None:
    field = FIELDS[index]
    if not field.editable:
        state.set_status("%s 为只读字段。" % field.label)
        return
    old_value = _field_value(state, field)
    answer = _read_line(
        screen,
        "修改 %s（当前=%s；回车取消）： "
        % (field.label, _format_value(old_value)),
    )
    if answer is None:
        return
    try:
        if field.kind == "float":
            value = float(answer)
        elif field.kind == "int":
            value = int(answer)
        else:
            value = answer
        if field.key in ("start_x", "start_y", "goal_x", "goal_y"):
            setattr(state, field.key, value)
        else:
            setattr(state.config, field.key, value)
            state.config.validate()
        state.result = None
        state.set_status("%s 已修改。" % field.label)
    except (TypeError, ValueError, PlanningError) as exc:
        if field.key not in ("start_x", "start_y", "goal_x", "goal_y"):
            setattr(state.config, field.key, old_value)
        state.set_status("修改失败：%s" % exc)


def _show_help(screen) -> None:
    screen.erase()
    height, width = screen.getmaxyx()
    lines = [
        "键盘说明",
        "",
        "方向键：在参数面板中移动",
        "Enter / e：编辑当前参数，输入后回车生效",
        "p：用当前 bag、参数、起终点执行规划",
        "x：保存最近一次规划结果为 XML",
        "w：保存当前 planner 配置到当前 YAML",
        "a：将当前 planner 配置另存为新的 YAML",
        "l：加载另一个 YAML（不会重复确认每个参数）",
        "b：选择另一个 rosbag",
        "q：退出",
        "",
        "UNKNOWN 栅格不会进入 A*；UNKNOWN 不自动吸附。",
        "占据区端点自动吸附到与另一端连通的 FREE 区域。",
        "按任意键返回。",
    ]
    for row, line in enumerate(lines):
        _safe_add(screen, row + 1, 2, line, width)
    screen.refresh()
    screen.getch()


def _choose_bag(screen, root: Path, config: PlannerConfig) -> Path:
    height, width = screen.getmaxyx()
    _safe_add(screen, height - 2, 0, "扫描满足 map+scan+odom 的 rosbag……", width)
    screen.refresh()
    candidates = find_bags(root, config)
    if not candidates:
        raise PlanningError("没有找到同时包含 map、逐帧点云和 odom 的 rosbag")
    if len(candidates) == 1:
        return candidates[0].path

    selected = 0
    while True:
        screen.erase()
        _safe_add(screen, 0, 0, "选择 rosbag（Enter确认，Esc取消）", width, curses.A_BOLD)
        for index, candidate in enumerate(candidates):
            row = 2 + index
            if row >= height - 2:
                break
            text = "%s %s  map=%d scan=%d odom=%d %.1fs" % (
                ">" if index == selected else " ",
                candidate.path,
                candidate.map_messages,
                candidate.scan_messages,
                candidate.odom_messages,
                candidate.duration,
            )
            _safe_add(
                screen,
                row,
                0,
                text,
                width,
                curses.A_REVERSE if index == selected else 0,
            )
        screen.refresh()
        key = screen.getch()
        if key in (curses.KEY_UP, ord("k")):
            selected = (selected - 1) % len(candidates)
        elif key in (curses.KEY_DOWN, ord("j")):
            selected = (selected + 1) % len(candidates)
        elif key in (curses.KEY_ENTER, 10, 13):
            return candidates[selected].path
        elif key == 27:
            raise PlanningError("已取消 rosbag 选择")


def _load_config(
    screen,
    manager: ConfigManager,
    state: TuiState,
    output_override: Optional[str],
) -> ConfigManager:
    answer = _read_line(screen, "新的 config.yaml 路径：")
    if answer is None:
        return manager
    loaded = ConfigManager.load(Path(os.path.expanduser(answer)))
    state.config = loaded.config
    if output_override:
        state.config.output_path = output_override
    state.bag_data = None
    state.result = None
    state.set_status("已加载配置：%s" % loaded.path)
    return loaded


def _save_config(
    screen,
    manager: ConfigManager,
    state: TuiState,
) -> None:
    manager.save(state.config)
    state.set_status("已保存 planner 配置：%s" % manager.path)


def _save_config_as(
    screen,
    manager: ConfigManager,
    state: TuiState,
) -> None:
    answer = _read_line(screen, "另存为 config.yaml 路径：")
    if answer is None:
        return
    target = Path(os.path.expanduser(answer))
    if target.exists() and not _confirm(screen, "文件已存在，确认覆盖？"):
        state.set_status("未覆盖配置文件。")
        return
    manager.save(state.config, target)
    state.set_status("已另存配置：%s" % target)


def _run_plan(screen, state: TuiState) -> None:
    if state.bag_path is None:
        state.bag_path = _choose_bag(screen, state.bag_root, state.config)
    state.config.validate()
    height, width = screen.getmaxyx()
    requested_start = (state.start_x, state.start_y)
    requested_goal = (state.goal_x, state.goal_y)
    _safe_add(screen, height - 2, 0, "加载 bag 缓存或解析 rosbag……", width)
    screen.refresh()
    state.bag_data = read_bag(state.bag_path, state.config)
    _safe_add(screen, height - 2, 0, "加载或构建三态占据栅格……", width)
    screen.refresh()
    state.result = plan_route(
        state.bag_data,
        state.config,
        requested_start,
        requested_goal,
    )
    state.start_x = state.result.waypoints[0].x
    state.start_y = state.result.waypoints[0].y
    state.goal_x = state.result.waypoints[-1].x
    state.goal_y = state.result.waypoints[-1].y

    target = state.config.output_file
    write_waypoints_xml(target, state.result, state.config, overwrite=True)
    snap_lines = []
    if state.result.start_snapped:
        snap_lines.append(
            "起点已从 OCCUPIED 自动吸附到 (%.3f, %.3f)。"
            % (state.start_x, state.start_y)
        )
    if state.result.goal_snapped:
        snap_lines.append(
            "终点已从 OCCUPIED 自动吸附到 (%.3f, %.3f)。"
            % (state.goal_x, state.goal_y)
        )
    cache_text = (
        "使用已有 bag/栅格缓存"
        if getattr(state.bag_data, "cache_hit", False)
        and state.result.occupancy_cache_hit
        else "使用 bag 缓存，首次构建栅格"
        if getattr(state.bag_data, "cache_hit", False)
        else "首次解析 bag 并构建栅格"
    )
    state.set_status(
        "规划成功：%d 个航点，路径 %.2fm；已自动保存 XML：%s。" % (
            len(state.result.waypoints),
            state.result.path_length,
            target,
        ),
        "%s；FREE=%d，UNKNOWN=%d，FREE连通域=%d。"
        % (
            cache_text,
            state.result.free_cells,
            state.result.unknown_cells,
            state.result.free_components,
        ),
        "地图 %d点→%d点，逐帧采样 %d点。"
        % (
            state.result.map_points,
            state.result.map_sampled_points,
            state.result.scan_sampled_points,
        ),
        *snap_lines,
    )


def _save_xml(screen, state: TuiState) -> None:
    if state.result is None:
        state.set_status("还没有规划结果，请先按 p。")
        return
    target = state.config.output_file
    if target.exists() and not _confirm(screen, "XML 已存在，确认覆盖？"):
        state.set_status("未覆盖 XML。")
        return
    write_waypoints_xml(target, state.result, state.config, overwrite=True)
    state.set_status("已保存 XML：%s" % target)


def run_tui(
    screen,
    manager: ConfigManager,
    bag_argument: Optional[Path],
    output_override: Optional[str] = None,
) -> int:
    config = manager.config
    if output_override:
        config.output_path = output_override
    bag_root = (
        bag_argument
        if bag_argument is not None and bag_argument.is_dir()
        else DEFAULT_BAG_ROOT
    )
    state = TuiState(config=config, bag_root=bag_root)
    if bag_argument is not None and bag_argument.is_file():
        state.bag_path = bag_argument

    selected = 0
    _set_cursor(False)
    while True:
        _draw_panel(screen, state, selected)
        key = screen.getch()
        if key in (curses.KEY_UP, ord("k")):
            selected = (selected - 1) % len(FIELDS)
        elif key in (curses.KEY_DOWN, ord("j")):
            selected = (selected + 1) % len(FIELDS)
        elif key == curses.KEY_LEFT:
            if selected >= (len(FIELDS) + 1) // 2:
                selected -= (len(FIELDS) + 1) // 2
        elif key == curses.KEY_RIGHT:
            split = (len(FIELDS) + 1) // 2
            if selected < split and selected + split < len(FIELDS):
                selected += split
        elif key in (curses.KEY_ENTER, 10, 13, ord("e")):
            _edit_field(screen, state, selected)
        elif key in (ord("p"), ord("r")):
            try:
                _run_plan(screen, state)
            except (PlanningError, OSError) as exc:
                state.set_status("规划失败：%s" % exc)
        elif key == ord("x"):
            try:
                _save_xml(screen, state)
            except (PlanningError, OSError) as exc:
                state.set_status("XML 保存失败：%s" % exc)
        elif key == ord("w"):
            try:
                _save_config(screen, manager, state)
            except (PlanningError, OSError) as exc:
                state.set_status("配置保存失败：%s" % exc)
        elif key == ord("a"):
            try:
                _save_config_as(screen, manager, state)
            except (PlanningError, OSError) as exc:
                state.set_status("配置另存失败：%s" % exc)
        elif key == ord("l"):
            try:
                manager = _load_config(screen, manager, state, output_override)
            except (PlanningError, OSError) as exc:
                state.set_status("加载配置失败：%s" % exc)
        elif key == ord("b"):
            try:
                state.bag_path = _choose_bag(screen, state.bag_root, state.config)
                state.bag_data = None
                state.result = None
                state.set_status("已选择 bag：%s" % state.bag_path)
            except (PlanningError, OSError) as exc:
                state.set_status("选择 bag 失败：%s" % exc)
        elif key in (ord("h"), ord("?")):
            _show_help(screen)
        elif key in (ord("q"), 27):
            return 0
