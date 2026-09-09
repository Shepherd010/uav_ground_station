"""Deterministic voxel downsampling helpers."""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from .models import PlanningError


Point3 = Tuple[float, float, float]


def voxel_sample(points: Sequence[Point3], voxel_size: float) -> List[Point3]:
    """Keep the first finite point in each 3D voxel."""
    if not math.isfinite(voxel_size) or voxel_size <= 0.0:
        raise PlanningError("体素边长必须是大于 0 的有限数值")

    sampled = []
    seen = set()
    for point in points:
        if not all(math.isfinite(float(value)) for value in point):
            continue
        key = tuple(int(math.floor(value / voxel_size)) for value in point)
        if key in seen:
            continue
        seen.add(key)
        sampled.append(point)
    return sampled
