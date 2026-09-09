"""A* search over the FREE part of a three-state 2.5D occupancy grid."""

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

from .models import (
    BagData,
    GridMap,
    PlanResult,
    PlannerConfig,
    PlanningError,
    Waypoint,
)
from .occupancy import OccupancyBuild, build_occupancy, centerline_cells


Cell = Tuple[int, int]
Point2 = Tuple[float, float]


FREE_NEIGHBORS = (
    (-1, 0),
    (1, 0),
    (0, -1),
    (0, 1),
    (-1, -1),
    (-1, 1),
    (1, -1),
    (1, 1),
)


def line_is_free(grid: GridMap, start: Cell, goal: Cell) -> bool:
    return all(grid.is_free(cell) for cell in centerline_cells(start, goal))


def astar(
    grid: GridMap,
    start: Cell,
    goal: Cell,
) -> Optional[List[Cell]]:
    if not grid.is_free(start) or not grid.is_free(goal):
        return None
    if start == goal:
        return [start]

    directions = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    )
    counter = 0
    open_set = [(0.0, counter, start)]
    came_from: Dict[Cell, Cell] = {}
    cost: Dict[Cell, float] = {start: 0.0}

    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        current_cost = cost[current]
        for dx, dy, step_cost in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if not grid.is_free(neighbor):
                continue
            new_cost = current_cost + step_cost
            if new_cost >= cost.get(neighbor, float("inf")):
                continue
            cost[neighbor] = new_cost
            came_from[neighbor] = current
            counter += 1
            heuristic = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
            heapq.heappush(open_set, (new_cost + heuristic, counter, neighbor))
    return None


def free_components(
    grid: GridMap,
) -> Tuple[Dict[Cell, int], Dict[int, List[Cell]]]:
    """Label FREE cells using the same 8-neighbor graph as A*.

    Endpoint snapping must know which FREE area can actually reach the other
    endpoint.  A global nearest-cell search is insufficient when the scan
    produces several disconnected FREE islands.
    """
    remaining = {cell for cell in grid.free if grid.is_free(cell)}
    labels: Dict[Cell, int] = {}
    cells_by_component: Dict[int, List[Cell]] = {}
    component_id = 0

    while remaining:
        seed = remaining.pop()
        stack = [seed]
        cells: List[Cell] = []
        while stack:
            current = stack.pop()
            labels[current] = component_id
            cells.append(current)
            for dx, dy in FREE_NEIGHBORS:
                neighbor = (current[0] + dx, current[1] + dy)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
        cells_by_component[component_id] = cells
        component_id += 1

    return labels, cells_by_component


def compress_path(grid: GridMap, cells: Sequence[Cell]) -> List[Cell]:
    if not cells:
        return []
    compressed = [cells[0]]
    current_index = 0
    while current_index < len(cells) - 1:
        next_index = current_index + 1
        for candidate in range(len(cells) - 1, current_index, -1):
            if line_is_free(grid, cells[current_index], cells[candidate]):
                next_index = candidate
                break
        compressed.append(cells[next_index])
        current_index = next_index
    return compressed


def distance_2d(first: Point2, second: Point2) -> float:
    return math.hypot(second[0] - first[0], second[1] - first[1])


def _nearest_distinct_pair(
    grid: GridMap,
    cells: Sequence[Cell],
    start: Point2,
    goal: Point2,
) -> Optional[Tuple[Cell, Cell, float]]:
    """Find a low-displacement pair of different cells in one component."""
    if len(cells) < 2:
        return None

    nearest_start = min(
        cells,
        key=lambda cell: distance_2d(start, grid.center(cell)),
    )
    nearest_goal = min(
        cells,
        key=lambda cell: distance_2d(goal, grid.center(cell)),
    )
    start_distance = distance_2d(start, grid.center(nearest_start))
    goal_distance = distance_2d(goal, grid.center(nearest_goal))
    if nearest_start != nearest_goal:
        return nearest_start, nearest_goal, start_distance + goal_distance

    alternatives_start = [cell for cell in cells if cell != nearest_start]
    alternatives_goal = [cell for cell in cells if cell != nearest_goal]
    second_start = min(
        alternatives_start,
        key=lambda cell: distance_2d(start, grid.center(cell)),
    )
    second_goal = min(
        alternatives_goal,
        key=lambda cell: distance_2d(goal, grid.center(cell)),
    )
    choices = (
        (
            second_start,
            nearest_goal,
            distance_2d(start, grid.center(second_start)) + goal_distance,
        ),
        (
            nearest_start,
            second_goal,
            start_distance + distance_2d(goal, grid.center(second_goal)),
        ),
    )
    return min(choices, key=lambda choice: choice[2])


def polyline_length(points: Sequence[Point2]) -> float:
    return sum(
        distance_2d(points[index - 1], points[index])
        for index in range(1, len(points))
    )


def resample_points(
    points: Sequence[Point2],
    target_count: int,
) -> List[Point2]:
    if len(points) >= target_count:
        return list(points)
    total = polyline_length(points)
    if total <= 1e-9:
        raise PlanningError("路径长度为 0，无法生成有效航点")

    cumulative = [0.0]
    for index in range(1, len(points)):
        cumulative.append(
            cumulative[-1] + distance_2d(points[index - 1], points[index])
        )

    result = []
    for sample_index in range(target_count):
        target = total * sample_index / float(target_count - 1)
        segment = 1
        while segment < len(cumulative) - 1 and cumulative[segment] < target:
            segment += 1
        before = cumulative[segment - 1]
        after = cumulative[segment]
        fraction = 0.0 if after == before else (target - before) / (after - before)
        result.append(
            (
                points[segment - 1][0]
                + fraction * (points[segment][0] - points[segment - 1][0]),
                points[segment - 1][1]
                + fraction * (points[segment][1] - points[segment - 1][1]),
            )
        )
    return result


def simplify_short_segments(
    grid: GridMap,
    points: Sequence[Point2],
    minimum_spacing: float,
) -> List[Point2]:
    """Remove an interior point only when the bypass remains FREE."""
    result = list(points)
    if minimum_spacing <= 0.0 or len(result) <= 2:
        return result

    changed = True
    while changed and len(result) > 2:
        changed = False
        for index in range(1, len(result) - 1):
            previous = result[index - 1]
            current = result[index]
            following = result[index + 1]
            if (
                distance_2d(previous, current) >= minimum_spacing
                and distance_2d(current, following) >= minimum_spacing
            ):
                continue
            if line_is_free(
                grid,
                grid.cell_for_xy(*previous),
                grid.cell_for_xy(*following),
            ):
                del result[index]
                changed = True
                break
    return result


def _make_waypoints(
    points: Sequence[Point2],
    config: PlannerConfig,
) -> List[Waypoint]:
    yaws = []
    previous_yaw = 0.0
    for index in range(len(points)):
        if index < len(points) - 1:
            dx = points[index + 1][0] - points[index][0]
            dy = points[index + 1][1] - points[index][1]
            if abs(dx) > 1e-12 or abs(dy) > 1e-12:
                previous_yaw = math.atan2(dy, dx)
        yaws.append(previous_yaw)
    return [
        Waypoint(x, y, config.z, yaw, config.hover_time, config.speed)
        for (x, y), yaw in zip(points, yaws)
    ]


def _resolve_endpoint(
    grid: GridMap,
    name: str,
    point: Point2,
    component_labels: Optional[Dict[Cell, int]] = None,
    cells_by_component: Optional[Dict[int, List[Cell]]] = None,
    allowed_component: Optional[int] = None,
    preferred_cell: Optional[Cell] = None,
) -> Tuple[Cell, Point2, bool]:
    if not grid.contains_xy(*point):
        raise PlanningError("%s 超出已观测场景范围，请重新选择" % name)
    cell = grid.cell_for_xy(*point)
    if grid.is_unknown(cell):
        raise PlanningError(
            "%s 落在 UNKNOWN 未观测栅格；未知区域不能规划，请重新选择"
            % name
        )
    if grid.is_free(cell):
        if (
            allowed_component is not None
            and component_labels is not None
            and component_labels.get(cell) != allowed_component
        ):
            raise PlanningError(
                "%s 所在 FREE 连通域与另一端不连通，无法保持规划可行性"
                % name
            )
        return cell, point, False

    if preferred_cell is not None:
        if not grid.is_free(preferred_cell):
            raise PlanningError("%s 的自动吸附候选不是 FREE 栅格" % name)
        if (
            allowed_component is not None
            and component_labels is not None
            and component_labels.get(preferred_cell) != allowed_component
        ):
            raise PlanningError("%s 的自动吸附候选不在目标 FREE 连通域" % name)
        return preferred_cell, grid.center(preferred_cell), True

    if cells_by_component is not None and allowed_component is not None:
        free_cells = cells_by_component.get(allowed_component, [])
    elif cells_by_component is not None:
        free_cells = [
            candidate
            for candidates in cells_by_component.values()
            for candidate in candidates
        ]
    else:
        # Keep this helper usable by small external callers that do not need
        # component-aware snapping.
        free_cells = [candidate for candidate in grid.free if grid.is_free(candidate)]
    if not free_cells:
        if allowed_component is None:
            raise PlanningError(
                "%s 所在 OCCUPIED 栅格附近没有可吸附的 FREE 栅格" % name
            )
        raise PlanningError(
            "%s 附近没有位于另一端 FREE 连通域的可吸附栅格；"
            "请调整起点/终点或栅格参数" % name
        )
    snapped_cell = min(
        free_cells,
        key=lambda candidate: distance_2d(point, grid.center(candidate)),
    )
    return snapped_cell, grid.center(snapped_cell), True


def plan_route(
    bag: BagData,
    config: PlannerConfig,
    start_xy: Point2,
    goal_xy: Point2,
) -> PlanResult:
    """Create a route using only ray-observed FREE cells."""
    config.validate()
    if not all(math.isfinite(float(value)) for value in start_xy + goal_xy):
        raise PlanningError("起点和终点必须是有限的 x y 数值")
    if distance_2d(start_xy, goal_xy) <= 1e-9:
        raise PlanningError("起点和终点不能相同")

    occupancy = build_occupancy(bag, config)
    grid = occupancy.grid
    component_labels, cells_by_component = free_components(grid)
    start_cell = grid.cell_for_xy(*start_xy)
    goal_cell = grid.cell_for_xy(*goal_xy)

    # Validate UNKNOWN/out-of-scene first.  UNKNOWN is never auto-snapped:
    # there is no observation proving that a nearby FREE cell is reachable.
    _resolve_endpoint(
        grid,
        "起点",
        start_xy,
        component_labels,
        cells_by_component,
    )
    _resolve_endpoint(
        grid,
        "终点",
        goal_xy,
        component_labels,
        cells_by_component,
    )
    start_free = grid.is_free(start_cell)
    goal_free = grid.is_free(goal_cell)

    if start_free and goal_free:
        # Keep explicitly selected FREE endpoints exact.  If they are in
        # different components, silently moving them would hide a mapping
        # problem and could turn UNKNOWN space into an apparent route.
        resolved_start = start_xy
        resolved_goal = goal_xy
        start_snapped = False
        goal_snapped = False
    elif start_free:
        start_component = component_labels[start_cell]
        start_cell, resolved_start, start_snapped = _resolve_endpoint(
            grid,
            "起点",
            start_xy,
            component_labels,
            cells_by_component,
            allowed_component=start_component,
        )
        goal_cell, resolved_goal, goal_snapped = _resolve_endpoint(
            grid,
            "终点",
            goal_xy,
            component_labels,
            cells_by_component,
            allowed_component=start_component,
        )
    elif goal_free:
        goal_component = component_labels[goal_cell]
        start_cell, resolved_start, start_snapped = _resolve_endpoint(
            grid,
            "起点",
            start_xy,
            component_labels,
            cells_by_component,
            allowed_component=goal_component,
        )
        goal_cell, resolved_goal, goal_snapped = _resolve_endpoint(
            grid,
            "终点",
            goal_xy,
            component_labels,
            cells_by_component,
            allowed_component=goal_component,
        )
    else:
        # Both endpoints are OCCUPIED.  Select one common FREE component,
        # minimizing total displacement first and preferring a larger area on
        # ties.  Any pair chosen from the same component is reachable in the
        # current A* graph; the ranking preserves the user's selections as
        # much as possible while avoiding an isolated FREE island.
        component_pairs = {}
        for component_id, cells in cells_by_component.items():
            pair = _nearest_distinct_pair(grid, cells, start_xy, goal_xy)
            if pair is not None:
                component_pairs[component_id] = pair
        common_components = set(component_pairs)
        if not common_components:
            raise PlanningError(
                "起点和终点附近没有包含两个不同栅格的共同 FREE 连通域，"
                "无法自动吸附出可行路径"
            )
        selected_component = min(
            common_components,
            key=lambda component_id: (
                component_pairs[component_id][2],
                -len(cells_by_component[component_id]),
            ),
        )
        start_candidate, goal_candidate, _ = component_pairs[selected_component]
        start_cell, resolved_start, start_snapped = _resolve_endpoint(
            grid,
            "起点",
            start_xy,
            component_labels,
            cells_by_component,
            allowed_component=selected_component,
            preferred_cell=start_candidate,
        )
        goal_cell, resolved_goal, goal_snapped = _resolve_endpoint(
            grid,
            "终点",
            goal_xy,
            component_labels,
            cells_by_component,
            allowed_component=selected_component,
            preferred_cell=goal_candidate,
        )

    if distance_2d(resolved_start, resolved_goal) <= 1e-9:
        raise PlanningError("起点和终点吸附后重合，无法生成有效路径")

    if (
        component_labels.get(start_cell) is not None
        and component_labels.get(start_cell) != component_labels.get(goal_cell)
    ):
        raise PlanningError(
            "起点和终点位于不同 FREE 连通域，UNKNOWN 区域不能作为连接通道"
        )

    raw = astar(grid, start_cell, goal_cell)
    if raw is None:
        raise PlanningError(
            "A* 未找到只经过 FREE 栅格的路径（FREE=%d，OCCUPIED=%d，"
            "UNKNOWN=%d，栅格=%d x %d）"
            % (
                len(grid.free),
                len(grid.occupied),
                grid.unknown_count,
                grid.nx,
                grid.ny,
            )
        )

    compressed = compress_path(grid, raw)
    if len(compressed) == 1:
        points = [resolved_start, resolved_goal]
    else:
        points = [grid.center(cell) for cell in compressed]
        points[0] = resolved_start
        points[-1] = resolved_goal
    points = simplify_short_segments(
        grid,
        points,
        config.min_waypoint_spacing,
    )

    if len(points) < config.min_waypoints:
        points = resample_points(points, config.min_waypoints)
    if len(points) > config.max_waypoints:
        raise PlanningError(
            "路径压缩后仍有 %d 个航点，超过 max_waypoints=%d"
            % (len(points), config.max_waypoints)
        )

    for first, second in zip(points, points[1:]):
        if not line_is_free(
            grid,
            grid.cell_for_xy(*first),
            grid.cell_for_xy(*second),
        ):
            raise PlanningError("航点直线段经过 UNKNOWN 或 OCCUPIED 栅格")

    return PlanResult(
        waypoints=_make_waypoints(points, config),
        raw_cells=len(raw),
        compressed_cells=len(compressed),
        occupied_cells=len(grid.occupied),
        free_cells=len(grid.free),
        unknown_cells=grid.unknown_count,
        grid_size=(grid.nx, grid.ny),
        path_length=polyline_length(points),
        map_points=bag.final_map_points,
        map_sampled_points=occupancy.map_sampled_points,
        scan_messages=len(bag.scans),
        scan_sampled_points=bag.scan_point_count,
        occupancy_cache_hit=occupancy.cache_hit,
        start_snapped=start_snapped,
        goal_snapped=goal_snapped,
        free_components=len(cells_by_component),
    )
