"""Persistent caches for decoded bag data and deterministic occupancy grids."""

from __future__ import annotations

import hashlib
import json
import os
import pickle
import tempfile
from pathlib import Path
from typing import Any, Dict, Optional

from .models import BagData, PlannerConfig


CACHE_VERSION = 1
GRID_CACHE_VERSION = 2


def _fingerprint(path: Path, config: PlannerConfig) -> Dict[str, Any]:
    stat = path.stat()
    return {
        "version": CACHE_VERSION,
        "path": str(path.resolve()),
        "size": int(stat.st_size),
        "mtime_ns": int(getattr(stat, "st_mtime_ns", stat.st_mtime * 1e9)),
        "map_topic": config.map_topic,
        "scan_topic": config.scan_topic,
        "odom_topic": config.odom_topic,
        "input_frame_id": config.input_frame_id,
        "scan_voxel_size": float(config.scan_voxel_size),
    }


def _cache_path(path: Path, config: PlannerConfig) -> Path:
    identity = hashlib.sha256(str(path.resolve()).encode("utf-8")).hexdigest()
    return Path(config.cache_dir).expanduser() / (identity[:32] + ".pkl")


def load_cached_bag(path: Path, config: PlannerConfig) -> Optional[BagData]:
    """Return a cache hit, or None when the cache is absent/stale/invalid."""
    target = _cache_path(path, config)
    if not target.is_file():
        return None

    try:
        with target.open("rb") as stream:
            payload = pickle.load(stream)
        if not isinstance(payload, dict):
            return None
        if payload.get("fingerprint") != _fingerprint(path, config):
            return None
        data = payload.get("bag_data")
        if not isinstance(data, BagData):
            return None
        data.path = path
        data.cache_hit = True
        data.cache_path = target
        return data
    except Exception:
        return None


def save_bag_cache(data: BagData, path: Path, config: PlannerConfig) -> Path:
    """Atomically write a newly decoded bag cache."""
    target = _cache_path(path, config)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "fingerprint": _fingerprint(path, config),
        "bag_data": data,
    }

    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            prefix=target.name + ".",
            suffix=".tmp",
            dir=str(target.parent),
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(target))
    finally:
        if temporary is not None and temporary.exists():
            try:
                temporary.unlink()
            except OSError:
                pass

    data.cache_hit = False
    data.cache_path = target
    return target


def _grid_fingerprint(path: Path, config: PlannerConfig) -> Dict[str, Any]:
    """Fingerprint all inputs that affect the three-state occupancy grid."""
    return {
        "version": GRID_CACHE_VERSION,
        "bag": _fingerprint(path, config),
        "z": float(config.z),
        "slice_half_height": float(config.slice_half_height),
        "aircraft_length": float(config.aircraft_length),
        "aircraft_width": float(config.aircraft_width),
        "aircraft_height": float(config.aircraft_height),
        "grid_resolution": float(config.grid_resolution),
        "map_voxel_size": float(config.map_voxel_size),
    }


def occupancy_cache_path(path: Path, config: PlannerConfig) -> Path:
    identity = json.dumps(
        _grid_fingerprint(path, config),
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    digest = hashlib.sha256(identity).hexdigest()
    return Path(config.cache_dir).expanduser() / ("grid-" + digest[:32] + ".pkl")


def load_cached_occupancy(path: Path, config: PlannerConfig) -> Optional[Any]:
    """Load a cached occupancy object without importing occupancy.py here."""
    target = occupancy_cache_path(path, config)
    if not target.is_file():
        return None
    try:
        with target.open("rb") as stream:
            payload = pickle.load(stream)
        if not isinstance(payload, dict):
            return None
        if payload.get("fingerprint") != _grid_fingerprint(path, config):
            return None
        return payload.get("occupancy")
    except Exception:
        return None


def save_occupancy_cache(occupancy: Any, path: Path, config: PlannerConfig) -> Path:
    """Atomically write the deterministic occupancy result for a bag/config."""
    target = occupancy_cache_path(path, config)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "fingerprint": _grid_fingerprint(path, config),
        "occupancy": occupancy,
    }

    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            prefix=target.name + ".",
            suffix=".tmp",
            dir=str(target.parent),
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(target))
    finally:
        if temporary is not None and temporary.exists():
            try:
                temporary.unlink()
            except OSError:
                pass
    return target
