"""Shared data models and validation for the 2.5D planner."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple


class PlanningError(RuntimeError):
    """An error that can be corrected by changing input or configuration."""


@dataclass
class PlannerConfig:
    map_topic: str = "/robot/dlio/map_node/map"
    scan_topic: str = "/robot/dlio/odom_node/pointcloud/deskewed"
    odom_topic: str = "/robot/dlio/odom_node/odom"
    input_frame_id: str = "robot/odom"
    z: float = 1.0
    slice_half_height: float = 0.3
    aircraft_length: float = 0.6
    aircraft_width: float = 0.6
    aircraft_height: float = 0.45
    grid_resolution: float = 0.1
    map_voxel_size: float = 0.4
    scan_voxel_size: float = 0.3
    min_waypoints: int = 2
    max_waypoints: int = 20
    min_height: float = 0.5
    max_height: float = 2.0
    min_waypoint_spacing: float = 0.3
    output_frame_id: str = "map"
    output_path: str = "~/waypoints.xml"
    cache_dir: str = "~/.cache/uav_ground_station/plan_2d"
    hover_time: float = 5.0
    speed: float = 2.0

    def to_mapping(self) -> Dict[str, Any]:
        return {
            "map_topic": self.map_topic,
            "scan_topic": self.scan_topic,
            "odom_topic": self.odom_topic,
            "input_frame_id": self.input_frame_id,
            "z": self.z,
            "slice_half_height": self.slice_half_height,
            "aircraft_length": self.aircraft_length,
            "aircraft_width": self.aircraft_width,
            "aircraft_height": self.aircraft_height,
            "grid_resolution": self.grid_resolution,
            "map_voxel_size": self.map_voxel_size,
            "scan_voxel_size": self.scan_voxel_size,
            "min_waypoints": self.min_waypoints,
            "max_waypoints": self.max_waypoints,
            "min_height": self.min_height,
            "max_height": self.max_height,
            "min_waypoint_spacing": self.min_waypoint_spacing,
            "output_frame_id": self.output_frame_id,
            "output_path": self.output_path,
            "cache_dir": self.cache_dir,
            "hover_time": self.hover_time,
            "speed": self.speed,
        }

    def validate(self) -> None:
        for name in (
            "map_topic",
            "scan_topic",
            "odom_topic",
            "input_frame_id",
            "output_frame_id",
            "output_path",
            "cache_dir",
        ):
            if not str(getattr(self, name)).strip():
                raise PlanningError("%s 不能为空" % name)

        positive = (
            ("slice_half_height", self.slice_half_height),
            ("aircraft_length", self.aircraft_length),
            ("aircraft_width", self.aircraft_width),
            ("aircraft_height", self.aircraft_height),
            ("grid_resolution", self.grid_resolution),
            ("map_voxel_size", self.map_voxel_size),
            ("scan_voxel_size", self.scan_voxel_size),
            ("speed", self.speed),
        )
        for name, value in positive:
            if not math.isfinite(value) or value <= 0.0:
                raise PlanningError("%s 必须是大于 0 的有限数值" % name)

        for name, value in (
            ("z", self.z),
            ("min_height", self.min_height),
            ("max_height", self.max_height),
            ("min_waypoint_spacing", self.min_waypoint_spacing),
            ("hover_time", self.hover_time),
        ):
            if not math.isfinite(value):
                raise PlanningError("%s 必须是有限数值" % name)

        if self.min_height > self.max_height:
            raise PlanningError("最小飞行高度不能大于最大飞行高度")
        if self.z < self.min_height or self.z > self.max_height:
            raise PlanningError(
                "规划高度 z=%.3f 不在 [%.3f, %.3f] 内"
                % (self.z, self.min_height, self.max_height)
            )
        if self.min_waypoints < 2:
            raise PlanningError("min_waypoints 至少为 2（包含起点和终点）")
        if self.max_waypoints < self.min_waypoints:
            raise PlanningError("max_waypoints 不能小于 min_waypoints")
        if self.min_waypoint_spacing < 0.0:
            raise PlanningError("min_waypoint_spacing 必须是非负数")
        if self.hover_time < 0.0:
            raise PlanningError("hover_time 必须是非负数")

    @property
    def output_file(self) -> Path:
        return Path(os.path.expanduser(self.output_path))


@dataclass
class OdomSample:
    stamp: float
    x: float
    y: float
    z: float
    frame_id: str


@dataclass
class ScanData:
    stamp: float
    frame_id: str
    points: List[Tuple[float, float, float]]
    raw_point_count: int


@dataclass
class BagData:
    path: Path
    map_points: List[Tuple[float, float, float]]
    map_frame_id: str
    map_message_count: int
    scans: List[ScanData]
    odom: List[OdomSample]
    cache_hit: bool = False
    cache_path: Optional[Path] = None

    @property
    def final_map_points(self) -> int:
        return len(self.map_points)

    @property
    def scan_point_count(self) -> int:
        return sum(len(scan.points) for scan in self.scans)


@dataclass
class BagCandidate:
    path: Path
    map_messages: int
    scan_messages: int
    odom_messages: int
    duration: float


@dataclass
class GridMap:
    xmin: float
    ymin: float
    resolution: float
    nx: int
    ny: int
    occupied: Set[Tuple[int, int]]
    free: Set[Tuple[int, int]]

    def in_bounds(self, cell: Tuple[int, int]) -> bool:
        return 0 <= cell[0] < self.nx and 0 <= cell[1] < self.ny

    def is_occupied(self, cell: Tuple[int, int]) -> bool:
        return self.in_bounds(cell) and cell in self.occupied

    def is_free(self, cell: Tuple[int, int]) -> bool:
        return (
            self.in_bounds(cell)
            and cell in self.free
            and cell not in self.occupied
        )

    def is_unknown(self, cell: Tuple[int, int]) -> bool:
        return (
            self.in_bounds(cell)
            and not self.is_free(cell)
            and not self.is_occupied(cell)
        )

    def cell_for_xy(self, x: float, y: float) -> Tuple[int, int]:
        ix = int(math.floor((x - self.xmin) / self.resolution))
        iy = int(math.floor((y - self.ymin) / self.resolution))
        return (
            min(self.nx - 1, max(0, ix)),
            min(self.ny - 1, max(0, iy)),
        )

    def contains_xy(self, x: float, y: float) -> bool:
        return (
            self.xmin <= x <= self.xmin + self.nx * self.resolution
            and self.ymin <= y <= self.ymin + self.ny * self.resolution
        )

    def center(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        return (
            self.xmin + (cell[0] + 0.5) * self.resolution,
            self.ymin + (cell[1] + 0.5) * self.resolution,
        )

    @property
    def unknown_count(self) -> int:
        return self.nx * self.ny - len(self.free) - len(self.occupied)


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float
    hover_time: float
    speed: float


@dataclass
class PlanResult:
    waypoints: List[Waypoint]
    raw_cells: int
    compressed_cells: int
    occupied_cells: int
    free_cells: int
    unknown_cells: int
    grid_size: Tuple[int, int]
    path_length: float
    map_points: int
    map_sampled_points: int
    scan_messages: int
    scan_sampled_points: int
    occupancy_cache_hit: bool = False
    start_snapped: bool = False
    goal_snapped: bool = False
    free_components: int = 0
