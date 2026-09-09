#!/usr/bin/env python3
"""Command-line entry point for the modular 2.5D planner."""

from __future__ import annotations

import argparse
import curses
import os
import sys
from pathlib import Path
from typing import Optional, Sequence

if __package__ in (None, ""):
    # Allow direct execution as ./scripts/plan_2d/main.py.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from plan_2d.bag_reader import require_ros
    from plan_2d.config import ConfigManager
    from plan_2d.models import PlanningError
    from plan_2d.tui import run_tui
else:
    from .bag_reader import require_ros
    from .config import ConfigManager
    from .models import PlanningError
    from .tui import run_tui


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config.yaml"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "使用 DLIO-SLAM 累计地图和逐帧点云生成三态 2.5D A* 航点 XML"
        )
    )
    parser.add_argument(
        "bag",
        nargs="?",
        help=(
            "rosbag 文件或目录；省略时由面板按默认目录选择 "
            "/home/groundstation/experiments/rosbag"
        ),
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="YAML 配置文件（默认：uav_ground_station/config.yaml）",
    )
    parser.add_argument(
        "--output",
        help="初始 XML 输出路径，覆盖 planner.output_path",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        require_ros()
        manager = ConfigManager.load(
            Path(os.path.expanduser(args.config))
        )
        bag = Path(os.path.expanduser(args.bag)) if args.bag else None
        return curses.wrapper(
            lambda screen: run_tui(screen, manager, bag, args.output)
        )
    except (PlanningError, OSError, curses.error) as exc:
        print("错误：%s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
