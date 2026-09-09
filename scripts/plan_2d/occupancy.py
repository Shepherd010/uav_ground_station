"""Three-state 2.5D occupancy construction with ray clearing."""

from __future__ import annotations

import math
import pickle
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

from .bag_reader import OdomTimeline
from .cache import (
    load_cached_occupancy,
    occupancy_cache_path,
    save_occupancy_cache,
)
from .models import BagData, GridMap, PlannerConfig, PlanningError
from .sampling import voxel_sample


Cell = Tuple[int, int]
Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]


def supercover_cells(start: Cell, goal: Cell) -> Iterable[Cell]:
    """Yield every grid cell touched by an integer-grid line segment."""
    x0, y0 = start
    x1, y1 = goal
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    step_x = 1 if x1 >= x0 else -1
    step_y = 1 if y1 >= y0 else -1
    x, y = x0, y0
    ix = iy = 0
    yield (x, y)

    while ix < dx or iy < dy:
        left = (1 + 2 * ix) * dy
        right = (1 + 2 * iy) * dx
        if ix < dx and iy < dy and left == right:
            yield (x + step_x, y)
            yield (x, y + step_y)
            x += step_x
            y += step_y
            ix += 1
            iy += 1
        elif iy >= dy or (ix < dx and left < right):
            x += step_x
            ix += 1
        else:
            y += step_y
            iy += 1
        yield (x, y)


def centerline_cells(start: Cell, goal: Cell) -> Iterable[Cell]:
    """Yield centerline cells without an extra corner-clearance rule.

    The MVP already rasterizes the aircraft footprint into OCCUPIED cells.
    It intentionally permits a diagonal connection at a grid corner so a
    long corridor is not rejected only by a separate safety constraint.
    """
    x0, y0 = start
    x1, y1 = goal
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    step_x = 1 if x0 < x1 else -1
    step_y = 1 if y0 < y1 else -1
    error = dx - dy
    x, y = x0, y0

    while True:
        yield (x, y)
        if x == x1 and y == y1:
            return
        doubled = 2 * error
        if doubled > -dy:
            error -= dy
            x += step_x
        if doubled < dx:
            error += dx
            y += step_y


@dataclass
class OccupancyBuild:
    grid: GridMap
    map_sampled_points: int
    map_slice_points: int
    scan_rays: int
    cache_hit: bool = False
    cache_path: Optional[Path] = None


def _grid_for_scene(
    bag: BagData,
    config: PlannerConfig,
) -> GridMap:
    scene_x = [point[0] for point in bag.map_points]
    scene_y = [point[1] for point in bag.map_points]
    for sample in bag.odom:
        scene_x.append(sample.x)
        scene_y.append(sample.y)
    for scan in bag.scans:
        # The scan is already voxel-sampled, so this does not duplicate the
        # full raw PointCloud2. It keeps bounds correct for partial maps.
        for x, y, _ in scan.points:
            scene_x.append(x)
            scene_y.append(y)
    if not scene_x or not scene_y:
        raise PlanningError("地图和逐帧点云都没有可用于建立场景边界的点")

    margin = max(config.aircraft_length, config.aircraft_width) / 2.0
    margin += config.grid_resolution
    xmin = min(scene_x) - margin
    xmax = max(scene_x) + margin
    ymin = min(scene_y) - margin
    ymax = max(scene_y) + margin
    nx = max(1, int(math.ceil((xmax - xmin) / config.grid_resolution)))
    ny = max(1, int(math.ceil((ymax - ymin) / config.grid_resolution)))
    return GridMap(
        xmin=xmin,
        ymin=ymin,
        resolution=config.grid_resolution,
        nx=nx,
        ny=ny,
        occupied=set(),
        free=set(),
    )


def _mark_occupied_point(
    grid: GridMap,
    point_xy: Point2,
    config: PlannerConfig,
) -> None:
    """Inflate one surface hit by the rectangular aircraft footprint."""
    half_x = config.aircraft_length / 2.0
    half_y = config.aircraft_width / 2.0
    px, py = point_xy
    base_x, base_y = grid.cell_for_xy(px, py)
    radius_x = int(math.ceil((half_x + grid.resolution) / grid.resolution))
    radius_y = int(math.ceil((half_y + grid.resolution) / grid.resolution))

    for ix in range(base_x - radius_x, base_x + radius_x + 1):
        for iy in range(base_y - radius_y, base_y + radius_y + 1):
            cell = (ix, iy)
            if not grid.in_bounds(cell):
                continue
            center_x, center_y = grid.center(cell)
            if (
                abs(center_x - px) <= half_x + grid.resolution / 2.0
                and abs(center_y - py) <= half_y + grid.resolution / 2.0
            ):
                grid.occupied.add(cell)


def _slice_interval(
    origin_z: float,
    point_z: float,
    low: float,
    high: float,
) -> Optional[Tuple[float, float, bool]]:
    """Return the ray parameter interval crossing the planning z slice."""
    dz = point_z - origin_z
    if abs(dz) <= 1e-12:
        if low <= origin_z <= high:
            return 0.0, 1.0, low <= point_z <= high
        return None

    first = (low - origin_z) / dz
    second = (high - origin_z) / dz
    start = max(0.0, min(first, second))
    end = min(1.0, max(first, second))
    if end < start:
        return None
    return start, end, low <= point_z <= high


def _ray_cells(
    grid: GridMap,
    origin: Point3,
    point: Point3,
    config: PlannerConfig,
) -> Optional[List[Cell]]:
    interval = _slice_interval(
        origin[2],
        point[2],
        config.z - config.slice_half_height,
        config.z + config.slice_half_height,
    )
    if interval is None:
        return None

    start_t, end_t, hit_in_slice = interval
    dx = point[0] - origin[0]
    dy = point[1] - origin[1]
    start_xy = (
        origin[0] + start_t * dx,
        origin[1] + start_t * dy,
    )
    end_xy = (
        origin[0] + end_t * dx,
        origin[1] + end_t * dy,
    )
    start_cell = grid.cell_for_xy(*start_xy)
    end_cell = grid.cell_for_xy(*end_xy)
    cells = list(supercover_cells(start_cell, end_cell))
    if hit_in_slice:
        # A point returned inside the planning slice is a surface hit.  Its
        # cell is not free merely because the lidar ray reaches it; the
        # cumulative map must classify it as OCCUPIED (otherwise it remains
        # UNKNOWN).  This prevents a missing/downsampled map hit from turning
        # a wall endpoint into FREE space.
        cells = [cell for cell in cells if cell != end_cell]
    return cells


def build_occupancy(
    bag: BagData,
    config: PlannerConfig,
    start_xy: Optional[Point2] = None,
    goal_xy: Optional[Point2] = None,
) -> OccupancyBuild:
    """Build OCCUPIED/FREE/UNKNOWN using map hits and scan ray evidence."""
    config.validate()
    cacheable = bag.path is not None and Path(bag.path).is_file()
    if cacheable:
        cached = load_cached_occupancy(bag.path, config)
        if isinstance(cached, OccupancyBuild):
            cached.cache_hit = True
            cached.cache_path = occupancy_cache_path(bag.path, config)
            return cached

    # Endpoint arguments are retained for API compatibility.  The grid is
    # intentionally based only on the recorded scene so it can be reused for
    # different start/goal selections without rebuilding the ray map.
    del start_xy, goal_xy
    grid = _grid_for_scene(bag, config)
    low = config.z - config.slice_half_height
    high = config.z + config.slice_half_height

    map_sampled = voxel_sample(bag.map_points, config.map_voxel_size)
    map_slice_points = 0
    for px, py, pz in map_sampled:
        if low <= pz <= high:
            map_slice_points += 1
            _mark_occupied_point(grid, (px, py), config)

    timeline = OdomTimeline(bag.odom)
    scan_rays = 0
    for scan in bag.scans:
        origin = timeline.position_at(scan.stamp)
        ray_keys = set()
        for point in scan.points:
            endpoint_cell = grid.cell_for_xy(point[0], point[1])
            # For a 2.5D grid one ray per projected endpoint cell and scan is
            # sufficient. Skipping repeated endpoints keeps dense Livox scans
            # tractable; omitted evidence remains UNKNOWN, never FREE.
            key = endpoint_cell
            if key in ray_keys:
                continue
            ray_keys.add(key)

            ray = _ray_cells(grid, origin, point, config)
            if ray is None:
                continue
            cells = ray
            scan_rays += 1
            for cell in cells:
                if grid.in_bounds(cell):
                    grid.free.add(cell)

    # An occupied hit always wins over a free ray from another observation.
    grid.free.difference_update(grid.occupied)
    result = OccupancyBuild(
        grid=grid,
        map_sampled_points=len(map_sampled),
        map_slice_points=map_slice_points,
        scan_rays=scan_rays,
    )
    if cacheable:
        try:
            result.cache_path = save_occupancy_cache(result, bag.path, config)
        except (OSError, ValueError, TypeError, pickle.PickleError) as exc:
            print("警告：占据栅格缓存写入失败：%s" % exc, file=sys.stderr)
    return result
