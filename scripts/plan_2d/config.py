"""YAML loading and planner-only configuration persistence."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, Optional

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

from .models import PlannerConfig, PlanningError


DEFAULT_MAP_TOPIC = "/robot/dlio/map_node/map"
DEFAULT_SCAN_TOPIC = "/robot/dlio/odom_node/pointcloud/deskewed"
DEFAULT_ODOM_TOPIC = "/robot/dlio/odom_node/odom"
DEFAULT_INPUT_FRAME = "robot/odom"


def planner_config_from_yaml(root: Dict[str, Any]) -> PlannerConfig:
    planner = root.get("planner") or {}
    validation = root.get("validation") or {}
    flight_defaults = root.get("flight_defaults") or {}
    paths = root.get("paths") or {}
    for name, section in (
        ("planner", planner),
        ("validation", validation),
        ("flight_defaults", flight_defaults),
        ("paths", paths),
    ):
        if not isinstance(section, dict):
            raise PlanningError("%s 必须是 YAML mapping" % name)

    def value(name: str, fallback: Any) -> Any:
        return planner[name] if name in planner else fallback

    return PlannerConfig(
        map_topic=str(value("map_topic", DEFAULT_MAP_TOPIC)),
        scan_topic=str(value("scan_topic", DEFAULT_SCAN_TOPIC)),
        odom_topic=str(value("odom_topic", DEFAULT_ODOM_TOPIC)),
        input_frame_id=str(value("input_frame_id", DEFAULT_INPUT_FRAME)),
        z=float(value("z", flight_defaults.get("takeoff_height", 1.0))),
        slice_half_height=float(value("slice_half_height", 0.3)),
        aircraft_length=float(value("aircraft_length", 0.6)),
        aircraft_width=float(value("aircraft_width", 0.6)),
        aircraft_height=float(value("aircraft_height", 0.45)),
        grid_resolution=float(value("grid_resolution", 0.1)),
        map_voxel_size=float(value("map_voxel_size", 0.4)),
        scan_voxel_size=float(value("scan_voxel_size", 0.3)),
        min_waypoints=int(value("min_waypoints", 2)),
        max_waypoints=int(value("max_waypoints", 20)),
        min_height=float(value("min_height", validation.get("min_height", 0.5))),
        max_height=float(value("max_height", validation.get("max_height", 2.0))),
        min_waypoint_spacing=float(
            value(
                "min_waypoint_spacing",
                validation.get("min_waypoint_spacing", 0.3),
            )
        ),
        output_frame_id=str(value("output_frame_id", "map")),
        output_path=str(
            value("output_path", paths.get("default_save", "~/waypoints.xml"))
        ),
        cache_dir=str(
            value("cache_dir", "~/.cache/uav_ground_station/plan_2d")
        ),
        hover_time=float(
            value("hover_time", flight_defaults.get("hover_duration", 5.0))
        ),
        speed=float(value("speed", flight_defaults.get("travel_speed", 2.0))),
    )


class ConfigManager:
    """Preserve all YAML text outside the root-level planner block."""

    _root_key = re.compile(r"^[^ \t#][^:]*:")

    def __init__(self, path: Path, text: str, root: Dict[str, Any]):
        self.path = path
        self.text = text
        self.root = root
        self.config = planner_config_from_yaml(root)

    @classmethod
    def load(cls, path: Path) -> "ConfigManager":
        if yaml is None:
            raise PlanningError("缺少 PyYAML，请安装 python3-yaml")
        if not path.is_file():
            raise PlanningError("配置文件不存在：%s" % path)
        text = path.read_text(encoding="utf-8")
        try:
            root = yaml.safe_load(text) or {}
        except yaml.YAMLError as exc:
            raise PlanningError("配置文件解析失败：%s" % exc)
        if not isinstance(root, dict):
            raise PlanningError("配置文件根节点必须是 YAML mapping")
        try:
            return cls(path, text, root)
        except (TypeError, ValueError) as exc:
            raise PlanningError("规划器配置存在非法数值：%s" % exc)

    def save(self, config: PlannerConfig, path: Optional[Path] = None) -> Path:
        if yaml is None:
            raise PlanningError("缺少 PyYAML，请安装 python3-yaml")
        config.validate()
        target = path or self.path
        block = yaml.safe_dump(
            {"planner": config.to_mapping()},
            allow_unicode=True,
            default_flow_style=False,
            sort_keys=False,
        )
        if not block.endswith("\n"):
            block += "\n"

        if target == self.path:
            original = self.text
        elif target.exists():
            original = target.read_text(encoding="utf-8")
        else:
            original = self.text

        lines = original.splitlines(keepends=True)
        planner_start = None
        for index, line in enumerate(lines):
            if re.match(r"^planner[ \t]*:", line):
                planner_start = index
                break

        if planner_start is None:
            if original and not original.endswith("\n"):
                original += "\n"
            updated = original + "\n" + block if original else block
        else:
            planner_end = len(lines)
            for index in range(planner_start + 1, len(lines)):
                if self._root_key.match(lines[index]):
                    planner_end = index
                    break
            updated = "".join(lines[:planner_start]) + block + "".join(
                lines[planner_end:]
            )

        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(updated, encoding="utf-8")
        if target == self.path:
            self.text = updated
            try:
                self.root = yaml.safe_load(updated) or {}
                self.config = planner_config_from_yaml(self.root)
            except yaml.YAMLError:
                pass
        return target
