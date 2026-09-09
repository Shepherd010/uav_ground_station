"""waypoint_manager-compatible XML serialization."""

from __future__ import annotations

import datetime
import math
import os
from pathlib import Path
from xml.sax.saxutils import escape

from .models import PlanResult, PlannerConfig, PlanningError


def _number(value: float) -> str:
    return "%.9f" % float(value)


def write_waypoints_xml(
    path: Path,
    result: PlanResult,
    config: PlannerConfig,
    overwrite: bool = False,
) -> Path:
    target = Path(os.path.expanduser(str(path)))
    if target.exists() and not overwrite:
        raise PlanningError("文件已存在，未覆盖：%s" % target)
    target.parent.mkdir(parents=True, exist_ok=True)

    created = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        "<waypoints>",
        "  <metadata>",
        "    <created>%s</created>" % created,
        "    <frame_id>%s</frame_id>" % escape(config.output_frame_id),
        "    <count>%d</count>" % len(result.waypoints),
        "  </metadata>",
    ]
    for index, waypoint in enumerate(result.waypoints, start=1):
        qz = math.sin(waypoint.yaw / 2.0)
        qw = math.cos(waypoint.yaw / 2.0)
        lines.extend(
            [
                '  <waypoint id="%d">' % index,
                "    <x>%s</x>" % _number(waypoint.x),
                "    <y>%s</y>" % _number(waypoint.y),
                "    <z>%s</z>" % _number(waypoint.z),
                "    <yaw>%s</yaw>" % _number(waypoint.yaw),
                "    <hover_time>%s</hover_time>"
                % _number(waypoint.hover_time),
                "    <speed>%s</speed>" % _number(waypoint.speed),
                "    <orientation>",
                "      <x>0.000000000</x>",
                "      <y>0.000000000</y>",
                "      <z>%s</z>" % _number(qz),
                "      <w>%s</w>" % _number(qw),
                "    </orientation>",
                "  </waypoint>",
            ]
        )
    lines.append("</waypoints>")
    target.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return target
