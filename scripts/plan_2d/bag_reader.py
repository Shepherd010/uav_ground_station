"""ROS bag discovery, PointCloud2 decoding, and odometry interpolation."""

from __future__ import annotations

import bisect
import math
import pickle
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

try:
    import rosbag
    from sensor_msgs import point_cloud2
except ImportError:  # pragma: no cover
    rosbag = None
    point_cloud2 = None

from .models import (
    BagCandidate,
    BagData,
    OdomSample,
    PlannerConfig,
    PlanningError,
    ScanData,
)
from .cache import load_cached_bag, save_bag_cache
from .sampling import voxel_sample


def require_ros() -> None:
    if rosbag is None or point_cloud2 is None or np is None:
        raise PlanningError(
            "缺少 ROS Noetic rosbag/sensor_msgs/numpy，请先执行 "
            "source /opt/ros/noetic/setup.bash"
        )


def _same_topic(first: str, second: str) -> bool:
    return first.rstrip("/") == second.rstrip("/")


def _message_stamp(message, bag_time) -> float:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    value = stamp.to_sec() if stamp is not None else 0.0
    if value <= 0.0:
        value = bag_time.to_sec()
    return float(value)


def _read_xyz(
    message, voxel_size: Optional[float] = None
) -> Tuple[List[Tuple[float, float, float]], int]:
    fields = {field.name for field in message.fields}
    missing = {"x", "y", "z"} - fields
    if missing:
        raise PlanningError(
            "PointCloud2 缺少字段：%s" % ", ".join(sorted(missing))
        )

    field_by_name = {field.name: field for field in message.fields}
    datatype_names = {
        1: "i1",
        2: "u1",
        3: "i2",
        4: "u2",
        5: "i4",
        6: "u4",
        7: "f4",
        8: "f8",
    }
    try:
        formats = []
        offsets = []
        for name in ("x", "y", "z"):
            field = field_by_name[name]
            if field.count != 1 or field.datatype not in datatype_names:
                raise ValueError("不支持的 PointField datatype/count")
            dtype_name = datatype_names[field.datatype]
            if dtype_name[-1:] not in ("1",):
                dtype_name = (">" if message.is_bigendian else "<") + dtype_name
            formats.append(dtype_name)
            offsets.append(field.offset)

        point_step = int(message.point_step)
        width = int(message.width)
        height = int(message.height)
        row_step = int(message.row_step)
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": formats,
                "offsets": offsets,
                "itemsize": point_step,
            }
        )
        data = memoryview(message.data)
        if height <= 1 or row_step == width * point_step:
            values = np.ndarray(
                shape=(width * max(1, height),),
                dtype=dtype,
                buffer=data,
            )
        else:
            rows = []
            for row in range(height):
                row_start = row * row_step
                row_data = data[row_start : row_start + width * point_step]
                rows.append(np.ndarray(shape=(width,), dtype=dtype, buffer=row_data))
            values = np.concatenate(rows) if rows else np.empty(0, dtype=dtype)

        raw_count = int(values.shape[0])
        coordinates = np.column_stack(
            (values["x"], values["y"], values["z"])
        ).astype(np.float64, copy=False)
        coordinates = coordinates[np.isfinite(coordinates).all(axis=1)]
        if voxel_size is not None and len(coordinates):
            keys = np.floor(coordinates / float(voxel_size)).astype(np.int64)
            _, first_indices = np.unique(keys, axis=0, return_index=True)
            coordinates = coordinates[np.sort(first_indices)]
        points = [tuple(float(value) for value in row) for row in coordinates]
    except Exception as exc:
        # Keep the ROS helper as a compatibility fallback for unusual
        # PointCloud2 layouts. Normal bags use the NumPy path above.
        points = []
        raw_count = 0
        try:
            for x, y, z in point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            ):
                raw_count += 1
                point = (float(x), float(y), float(z))
                if all(math.isfinite(value) for value in point):
                    points.append(point)
        except Exception as fallback_exc:
            raise PlanningError("解析 PointCloud2 失败：%s" % fallback_exc)
        if voxel_size is not None:
            points = voxel_sample(points, voxel_size)
    return points, raw_count


def find_bags(root: Path, config: PlannerConfig) -> List[BagCandidate]:
    """List only bags containing all inputs required by the planner."""
    require_ros()
    if root.is_file():
        candidates = [root]
    elif root.is_dir():
        candidates = sorted(root.rglob("*.bag"))
    else:
        raise PlanningError("rosbag 文件或目录不存在：%s" % root)

    result = []
    for bag_path in candidates:
        try:
            with rosbag.Bag(str(bag_path), "r") as bag:
                map_count = int(
                    bag.get_message_count(topic_filters=[config.map_topic])
                )
                scan_count = int(
                    bag.get_message_count(topic_filters=[config.scan_topic])
                )
                odom_count = int(
                    bag.get_message_count(topic_filters=[config.odom_topic])
                )
                if map_count and scan_count and odom_count:
                    duration = max(
                        0.0, bag.get_end_time() - bag.get_start_time()
                    )
                    result.append(
                        BagCandidate(
                            bag_path,
                            map_count,
                            scan_count,
                            odom_count,
                            duration,
                        )
                    )
        except Exception as exc:
            print(
                "跳过无法读取的 bag %s：%s" % (bag_path, exc),
                file=sys.stderr,
            )
    return result


def read_bag(path: Path, config: PlannerConfig) -> BagData:
    """Read the final cumulative map, per-scan clouds, and odometry."""
    require_ros()
    config.validate()
    path = path.expanduser()
    if not path.is_file():
        raise PlanningError("rosbag 文件不存在：%s" % path)

    cached = load_cached_bag(path, config)
    if cached is not None:
        return cached

    final_map_message = None
    map_message_count = 0
    scans: List[ScanData] = []
    odom: List[OdomSample] = []
    topics = [config.map_topic, config.scan_topic, config.odom_topic]
    try:
        with rosbag.Bag(str(path), "r") as bag:
            for topic, message, bag_time in bag.read_messages(topics=topics):
                if _same_topic(topic, config.map_topic):
                    final_map_message = message
                    map_message_count += 1
                    continue

                if _same_topic(topic, config.scan_topic):
                    frame_id = str(message.header.frame_id).strip()
                    if frame_id != config.input_frame_id:
                        raise PlanningError(
                            "逐帧点云 frame_id 为 %r，应为 %r"
                            % (frame_id, config.input_frame_id)
                        )
                    points, raw_count = _read_xyz(
                        message, voxel_size=config.scan_voxel_size
                    )
                    scans.append(
                        ScanData(
                            _message_stamp(message, bag_time),
                            frame_id,
                            points,
                            raw_count,
                        )
                    )
                    continue

                if _same_topic(topic, config.odom_topic):
                    frame_id = str(message.header.frame_id).strip()
                    if frame_id != config.input_frame_id:
                        raise PlanningError(
                            "odom frame_id 为 %r，应为 %r"
                            % (frame_id, config.input_frame_id)
                        )
                    position = message.pose.pose.position
                    values = (
                        _message_stamp(message, bag_time),
                        float(position.x),
                        float(position.y),
                        float(position.z),
                    )
                    if all(math.isfinite(value) for value in values):
                        odom.append(
                            OdomSample(
                                values[0],
                                values[1],
                                values[2],
                                values[3],
                                frame_id,
                            )
                        )
    except PlanningError:
        raise
    except Exception as exc:
        raise PlanningError("读取 rosbag 失败：%s" % exc)

    if final_map_message is None or map_message_count == 0:
        raise PlanningError("bag 中没有话题 %s 的累计地图消息" % config.map_topic)
    if not scans:
        raise PlanningError("bag 中没有可用于射线清空的逐帧点云 %s" % config.scan_topic)
    if not odom:
        raise PlanningError("bag 中没有可用于射线原点的 odom %s" % config.odom_topic)

    map_frame_id = str(final_map_message.header.frame_id).strip()
    if map_frame_id != config.input_frame_id:
        raise PlanningError(
            "累计地图 frame_id 为 %r，应为 %r"
            % (map_frame_id, config.input_frame_id)
        )
    map_points, _ = _read_xyz(final_map_message)
    if not map_points:
        raise PlanningError("最后一帧累计地图没有可用的有限点")

    scans.sort(key=lambda scan: scan.stamp)
    odom.sort(key=lambda sample: sample.stamp)
    data = BagData(
        path=path,
        map_points=map_points,
        map_frame_id=map_frame_id,
        map_message_count=map_message_count,
        scans=scans,
        odom=odom,
    )
    try:
        save_bag_cache(data, path, config)
    except (OSError, ValueError, TypeError, pickle.PickleError) as exc:
        # Caching accelerates later runs but must not make a valid bag
        # impossible to plan (for example on a read-only cache directory).
        print("警告：bag 缓存写入失败：%s" % exc, file=sys.stderr)
    return data


class OdomTimeline:
    """Linear interpolation of sensor origins in the map frame."""

    def __init__(self, samples: Sequence[OdomSample]):
        if not samples:
            raise PlanningError("odom 时间线为空")
        self.samples = list(sorted(samples, key=lambda sample: sample.stamp))
        self.stamps = [sample.stamp for sample in self.samples]

    def position_at(self, stamp: float) -> Tuple[float, float, float]:
        index = bisect.bisect_left(self.stamps, stamp)
        if index <= 0:
            sample = self.samples[0]
            return sample.x, sample.y, sample.z
        if index >= len(self.samples):
            sample = self.samples[-1]
            return sample.x, sample.y, sample.z

        before = self.samples[index - 1]
        after = self.samples[index]
        span = after.stamp - before.stamp
        fraction = 0.0 if span <= 0.0 else (stamp - before.stamp) / span
        return (
            before.x + fraction * (after.x - before.x),
            before.y + fraction * (after.y - before.y),
            before.z + fraction * (after.z - before.z),
        )
