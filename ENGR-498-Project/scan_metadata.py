from __future__ import annotations

import json
import shutil
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
    "raw_las": "processed/point_clouds/raw/cloud.las",
    "raw_pcd": "processed/point_clouds/raw/scans.pcd",
    "filtered_las": "processed/point_clouds/filtered/cloud.las",
    "filtered_pcd": "processed/point_clouds/filtered/scans.pcd",
    "las": "processed/point_clouds/raw/cloud.las",
    "pcd": "processed/point_clouds/raw/scans.pcd",
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
    "calibration_run_dir": "",
    "calibration_calib_json": "",
    "resolved_intrinsics_json": "",
    "resolved_extrinsics_json": "",
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
    "pose_recovery": {
        "enable_rviz": False,
    },
    "inference": {
        "runtime": "local",
        "weights": "",
        "local_device": "0",
        "preferred_colab_gpu": "A100",
    },
    "fusion": {
        "calibration_run_dir": "",
        "time_offset_sec": 0.0,
    },
    "gps": {
        "offset_body_xyz_m": "0,0,0",
    },
    "timing_overrides": {},
}

DEFAULT_WIRE_PARAMS = {
    "R": 0.5,
    "angleThr": 10,
    "linearity": 0.98,
    "sag_method": "legacy",
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
    if current.exists() and current.is_dir():
        return current
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
        "processed/point_clouds",
        "processed/point_clouds/raw",
        "processed/point_clouds/filtered",
        "processed/flai",
        "processed/wires",
        "processed/fusion",
        "processed/fusion/yolo_inference",
        "processed/fusion/gps",
        "processed/pose_recovery",
    ):
        (scan_dir / rel).mkdir(parents=True, exist_ok=True)


def migrate_legacy_layout(scan_dir: Path, metadata: dict[str, Any]) -> bool:
    """Migrate per-scan directories from processed/slam + processed/filtered
    to the new processed/point_clouds layout. Idempotent.

    Returns True if anything was changed (so the caller can persist metadata).
    """
    changed = False
    processed_dir = scan_dir / "processed"
    legacy_slam = processed_dir / "slam"
    legacy_filtered = processed_dir / "filtered"
    point_clouds_dir = processed_dir / "point_clouds"

    if legacy_slam.is_dir():
        raw_dir = point_clouds_dir / "raw"
        raw_dir.mkdir(parents=True, exist_ok=True)
        for name in ("scans.pcd", "cloud.las"):
            src = legacy_slam / name
            if src.is_file():
                dst = raw_dir / name
                if not dst.exists():
                    shutil.move(str(src), str(dst))
                    changed = True
        try:
            if not any(legacy_slam.iterdir()):
                legacy_slam.rmdir()
                changed = True
        except OSError:
            pass

    legacy_filtered_las = legacy_filtered / "cloud_filtered.las"
    if legacy_filtered_las.is_file():
        filtered_dir = point_clouds_dir / "filtered"
        filtered_dir.mkdir(parents=True, exist_ok=True)
        target_las = filtered_dir / "cloud.las"
        if (
            not target_las.exists()
            or target_las.stat().st_mtime < legacy_filtered_las.stat().st_mtime
        ):
            shutil.copy2(str(legacy_filtered_las), str(target_las))
            changed = True

    files_value = metadata.get("files")
    if isinstance(files_value, dict):
        for key in ("las", "pcd", "raw_las", "raw_pcd"):
            raw = files_value.get(key)
            if isinstance(raw, str) and raw.startswith("processed/slam/"):
                files_value[key] = raw.replace(
                    "processed/slam/", "processed/point_clouds/raw/", 1
                )
                changed = True

        filtered_las = files_value.get("filtered_las")
        if not isinstance(filtered_las, str) or not filtered_las:
            filtered_value = files_value.get("filtered")
            if isinstance(filtered_value, str) and filtered_value:
                files_value["filtered_las"] = filtered_value.replace(
                    "processed/filtered/",
                    "processed/point_clouds/filtered/",
                    1,
                )
                changed = True

        if "filtered" in files_value:
            files_value.pop("filtered")
            changed = True

    return changed


def _normalize_file_path_text(value: str) -> str:
    cleaned = value
    repeated = "processed/point_clouds/processed/point_clouds/"
    while repeated in cleaned:
        cleaned = cleaned.replace(repeated, "processed/point_clouds/", 1)
    return cleaned


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

    files_dict = normalized["files"]
    for key, value in list(files_dict.items()):
        if isinstance(value, str):
            files_dict[key] = _normalize_file_path_text(value)
    if not files_dict.get("raw_las") and files_dict.get("las"):
        files_dict["raw_las"] = files_dict["las"]
    if not files_dict.get("raw_pcd") and files_dict.get("pcd"):
        files_dict["raw_pcd"] = files_dict["pcd"]
    if not files_dict.get("las") and files_dict.get("raw_las"):
        files_dict["las"] = files_dict["raw_las"]
    if not files_dict.get("pcd") and files_dict.get("raw_pcd"):
        files_dict["pcd"] = files_dict["raw_pcd"]

    status_value = normalized.get("status", {})
    normalized["status"] = _deep_merge_dict(
        DEFAULT_STATUS,
        status_value if isinstance(status_value, dict) else {},
    )

    config_value = normalized.get("config", {})
    normalized["config"] = _deep_merge_dict(
        DEFAULT_CONFIG,
        config_value if isinstance(config_value, dict) else {},
    )

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
    migrated = migrate_legacy_layout(scan_dir, metadata)
    normalized = _normalize_metadata(scan_dir, metadata)
    if migrated:
        metadata_path.write_text(json.dumps(normalized, indent=2), encoding="utf-8")
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


def first_existing_artifact(
    scan_dir: Path,
    metadata: dict[str, Any],
    keys: tuple[str, ...] | list[str],
) -> tuple[str | None, Path | None]:
    artifacts = resolve_artifact_paths(scan_dir, metadata)
    for key in keys:
        candidate = artifacts.get(key)
        if candidate is not None and candidate.exists():
            return key, candidate
    return None, None


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
