from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


def _to_float_triplet(values: Any, *, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if not isinstance(values, (list, tuple)) or len(values) < 3:
        return default
    try:
        return float(values[0]), float(values[1]), float(values[2])
    except (TypeError, ValueError):
        return default


def load_fusion_objects(path: str | Path) -> list[dict[str, Any]]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    objects = payload.get("objects", [])
    if not isinstance(objects, list):
        raise ValueError(f"Expected an objects array in {path}")

    normalized: list[dict[str, Any]] = []
    for index, item in enumerate(objects):
        if not isinstance(item, dict):
            continue
        centroid = _to_float_triplet(item.get("centroid_map_xyz"), default=(0.0, 0.0, 0.0))
        bbox_min = _to_float_triplet(item.get("bbox_aabb_min_xyz"), default=centroid)
        bbox_max = _to_float_triplet(item.get("bbox_aabb_max_xyz"), default=centroid)
        color = _to_float_triplet(item.get("class_color_rgb"), default=(220.0, 220.0, 80.0))
        normalized.append(
            {
                "index": index,
                "object_name": str(item.get("object_name", f"object_{index + 1:02d}")),
                "class_name": str(item.get("class_name", "unknown")),
                "instance_number": int(item.get("instance_number", index + 1)),
                "confidence_score": float(item.get("confidence_score", 0.0)),
                "num_points": int(item.get("num_points", 0)),
                "centroid_map_xyz": np.asarray(centroid, dtype=np.float64),
                "bbox_aabb_min_xyz": np.asarray(bbox_min, dtype=np.float64),
                "bbox_aabb_max_xyz": np.asarray(bbox_max, dtype=np.float64),
                "class_color_rgb": color,
                "gps": item.get("gps", {}) if isinstance(item.get("gps"), dict) else {},
                "raw": item,
            }
        )
    return normalized


def load_pole_neighbor_links(path: str | Path, fusion_objects: list[dict[str, Any]]) -> list[dict[str, Any]]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    neighbors = payload.get("pole_neighbors", payload.get("distances", []))
    if not isinstance(neighbors, list):
        return []

    by_name = {item["object_name"]: item for item in fusion_objects}
    links: list[dict[str, Any]] = []
    for item in neighbors:
        if not isinstance(item, dict):
            continue
        source_name = str(item.get("source_pole") or item.get("from_object_name") or "")
        target_name = str(item.get("target_pole") or item.get("to_object_name") or "")
        source = by_name.get(source_name)
        target = by_name.get(target_name)
        if source is None:
            source_centroid = np.asarray(
                _to_float_triplet(item.get("from_centroid_map_xyz"), default=(0.0, 0.0, 0.0)),
                dtype=np.float64,
            )
        else:
            source_centroid = np.asarray(source["centroid_map_xyz"], dtype=np.float64)
        if target is None:
            target_centroid = np.asarray(
                _to_float_triplet(item.get("to_centroid_map_xyz"), default=(0.0, 0.0, 0.0)),
                dtype=np.float64,
            )
        else:
            target_centroid = np.asarray(target["centroid_map_xyz"], dtype=np.float64)
        if not source_name or not target_name:
            continue
        links.append(
            {
                "source_name": source_name,
                "target_name": target_name,
                "source_centroid": source_centroid,
                "target_centroid": target_centroid,
                "horizontal_distance_m": float(item.get("horizontal_distance_m", 0.0)),
                "distance_3d_m": float(item.get("distance_3d_m", 0.0)),
                "delta_z_m": float(item.get("delta_z_m", 0.0)),
                "raw": item,
            }
        )
    return links


def format_fusion_info(item: dict[str, Any]) -> str:
    gps = item.get("gps", {}) or {}
    centroid = np.asarray(item.get("centroid_map_xyz", (0.0, 0.0, 0.0)), dtype=np.float64)
    bbox_min = np.asarray(item.get("bbox_aabb_min_xyz", centroid), dtype=np.float64)
    bbox_max = np.asarray(item.get("bbox_aabb_max_xyz", centroid), dtype=np.float64)

    text = [
        f"=== {item.get('object_name', 'object')} ===",
        f"Class: {item.get('class_name', 'unknown')}",
        f"Confidence: {float(item.get('confidence_score', 0.0)):.3f}",
        f"Points: {int(item.get('num_points', 0))}",
        f"Centroid XYZ: [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}]",
        f"BBox min XYZ: [{bbox_min[0]:.3f}, {bbox_min[1]:.3f}, {bbox_min[2]:.3f}]",
        f"BBox max XYZ: [{bbox_max[0]:.3f}, {bbox_max[1]:.3f}, {bbox_max[2]:.3f}]",
    ]
    if gps.get("lat") is not None and gps.get("lon") is not None:
        text.append(
            f"GPS: lat={float(gps['lat']):.8f}, lon={float(gps['lon']):.8f}, alt={float(gps.get('alt', 0.0)):.3f}"
        )
    return "\n".join(text)
