from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any

from project_paths import PROJECT_ROOT


DEFAULT_STATUS = {
    "slam": "pending",
    "filtering": "pending",
    "flai": "pending",
    "wire_extraction": "pending",
    "inference": "pending",
    "fusion": "pending",
}

DEFAULT_FILES = {
    "rosbag": "",
    "las": "processed/slam/cloud.las",
    "pcd": "processed/slam/scans.pcd",
    "filtered": "processed/filtered/cloud_filtered.las",
    "segmented": "processed/flai/segmented.las",
    "wires_points": "processed/wires/wires_points.npz",
    "wire_info": "processed/wires/wire_info.json",
    "ground_points": "processed/wires/ground_points.npz",
    "pose_recovery_root": "processed/pose_recovery",
    "latest_pose_recovery_run": "",
    "images_dir": "",
    "image_timestamps_csv": "",
    "tf_camera_csv": "",
    "tf_gps_csv": "",
    "yolo_output_dir": "processed/fusion/yolo_inference",
    "pred_images_dir": "",
    "masks_dir": "",
    "meta_dir": "",
    "fused_objects": "processed/fusion/fused_objects.json",
    "fused_map": "processed/fusion/fused_semantic_map.ply",
    "fused_labels": "processed/fusion/fused_semantic_labels.npz",
    "pole_neighbor_distances": "processed/fusion/pole_neighbor_distances.json",
    "powerline_overlay": "processed/wires/eng498_powerlines_overlay.json",
    "gps_output_dir": "processed/fusion/gps",
    "georeferenced_objects": "processed/fusion/gps/fused_objects_georeferenced.json",
    "gps_alignment": "processed/fusion/gps/gps_alignment.json",
    "tf_gps_georeferenced_csv": "processed/fusion/gps/tf_gps_georeferenced.csv",
    "georeferenced_powerline_overlay": "processed/fusion/gps/eng498_powerlines_overlay_georeferenced.json",
}

DEFAULT_CONFIG = {
    "inference": {
        "runtime": "auto",
        "weights": "",
        "local_device": "0",
        "preferred_colab_gpu": "A100",
    },
    "fusion": {
        "intrinsics_json": "",
        "extrinsics_json": "",
        "time_column": "t_query_sec",
        "time_offset_sec": 0.0,
        "image_filename_column": "filename",
        "image_time_column": "",
    },
    "gps": {
        "offset_body_xyz_m": "0,0,0",
    },
}

DEFAULT_WIRE_PARAMS = {
    "R": 0.5,
    "angleThr": 10,
    "linearity": 0.98,
}


def _deep_merge_dict(base: dict[str, Any], incoming: dict[str, Any]) -> dict[str, Any]:
    merged = deepcopy(base)
    for key, value in incoming.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge_dict(merged[key], value)
        else:
            merged[key] = value
    return merged


def resolve_scan_dir(scan_ref: str | Path) -> Path:
    path = Path(scan_ref).resolve()
    if path.is_file():
        path = path.parent

    current = path
    for candidate in [current, *current.parents]:
        if (candidate / "metadata.json").is_file():
            return candidate
    raise FileNotFoundError(f"Could not resolve scan directory from {scan_ref}")


def relativize_for_scan(scan_dir: Path, target: str | Path) -> str:
    path = Path(target).resolve()
    try:
        return path.relative_to(scan_dir.resolve()).as_posix()
    except ValueError:
        try:
            return path.relative_to(PROJECT_ROOT.resolve()).as_posix()
        except ValueError:
            return str(path)


def resolve_scan_path(scan_dir: Path, stored_path: str | Path | None) -> Path | None:
    if not stored_path:
        return None
    path = Path(stored_path)
    if path.is_absolute():
        return path
    scan_candidate = (scan_dir / path).resolve()
    if scan_candidate.exists():
        return scan_candidate
    project_candidate = (PROJECT_ROOT / path).resolve()
    if project_candidate.exists():
        return project_candidate
    return scan_candidate


def ensure_scan_structure(scan_dir: Path) -> None:
    for rel in (
        "raw",
        "raw/images",
        "processed/slam",
        "processed/filtered",
        "processed/flai",
        "processed/wires",
        "processed/fusion",
        "processed/fusion/yolo_inference",
        "processed/fusion/gps",
        "processed/pose_recovery",
    ):
        (scan_dir / rel).mkdir(parents=True, exist_ok=True)


def _normalize_metadata(scan_dir: Path, metadata: dict[str, Any]) -> dict[str, Any]:
    normalized = deepcopy(metadata)

    if "filepaths" in normalized and "files" not in normalized:
        normalized["files"] = normalized.pop("filepaths")

    files_value = normalized.get("files", {})
    if isinstance(files_value, str):
        files_dict: dict[str, Any] = {"las": files_value}
    elif isinstance(files_value, dict):
        files_dict = dict(files_value)
    else:
        files_dict = {}
    normalized["files"] = _deep_merge_dict(DEFAULT_FILES, files_dict)

    status_value = normalized.get("status", {})
    normalized["status"] = _deep_merge_dict(DEFAULT_STATUS, status_value if isinstance(status_value, dict) else {})

    config_value = normalized.get("config", {})
    normalized["config"] = _deep_merge_dict(DEFAULT_CONFIG, config_value if isinstance(config_value, dict) else {})

    wire_params = normalized.get("wire_params", {})
    normalized["wire_params"] = _deep_merge_dict(
        DEFAULT_WIRE_PARAMS,
        wire_params if isinstance(wire_params, dict) else {},
    )

    normalized.setdefault("name", scan_dir.name)
    normalized.setdefault("created_at", "")
    normalized.setdefault("notes", "")
    return normalized


def load_scan_metadata(scan_ref: str | Path) -> tuple[Path, dict[str, Any]]:
    scan_dir = resolve_scan_dir(scan_ref)
    ensure_scan_structure(scan_dir)
    metadata_path = scan_dir / "metadata.json"
    if metadata_path.is_file():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    else:
        metadata = {"name": scan_dir.name}
    normalized = _normalize_metadata(scan_dir, metadata)
    return scan_dir, normalized


def save_scan_metadata(scan_dir: Path, metadata: dict[str, Any]) -> None:
    scan_dir = resolve_scan_dir(scan_dir)
    ensure_scan_structure(scan_dir)
    normalized = _normalize_metadata(scan_dir, metadata)
    (scan_dir / "metadata.json").write_text(json.dumps(normalized, indent=2), encoding="utf-8")


def resolve_artifact_paths(scan_dir: Path, metadata: dict[str, Any]) -> dict[str, Path]:
    files = metadata.get("files", {})
    return {
        key: resolved
        for key, resolved in (
            (key, resolve_scan_path(scan_dir, value))
            for key, value in files.items()
            if isinstance(value, str)
        )
        if resolved is not None
    }


def update_file_entry(metadata: dict[str, Any], scan_dir: Path, key: str, target: str | Path | None) -> None:
    metadata.setdefault("files", {})
    metadata["files"][key] = "" if not target else relativize_for_scan(scan_dir, target)


def update_status(metadata: dict[str, Any], step_key: str, status: str) -> None:
    metadata.setdefault("status", {})
    metadata["status"][step_key] = status


def newest_directory(root: Path) -> Path | None:
    if not root.is_dir():
        return None
    dirs = [item for item in root.iterdir() if item.is_dir()]
    if not dirs:
        return None
    return max(dirs, key=lambda path: path.stat().st_mtime)
