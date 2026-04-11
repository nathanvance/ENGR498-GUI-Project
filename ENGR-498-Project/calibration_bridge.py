from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from PIL import Image

from project_paths import ROSBAG_PREPROCESSING_DIR


CALIBRATION_OUTPUT_ROOT = ROSBAG_PREPROCESSING_DIR / "outputs" / "calibration"


def newest_calibration_run(root: Path | None = None) -> Path | None:
    search_root = (root or CALIBRATION_OUTPUT_ROOT).resolve()
    if not search_root.is_dir():
        return None

    runs = [
        path
        for path in search_root.iterdir()
        if path.is_dir() and path.name != ".gitkeep" and not path.name.endswith("_raw_input")
    ]
    if not runs:
        return None
    return max(runs, key=lambda path: path.stat().st_mtime)


def resolve_calibration_run(stored_path: str | Path | None, *, fallback_to_latest: bool = True) -> Path | None:
    if stored_path:
        candidate = Path(stored_path)
        if candidate.exists():
            return candidate.resolve()

    if fallback_to_latest:
        return newest_calibration_run()
    return None


def find_calib_json(calibration_run_dir: Path) -> Path:
    candidate = calibration_run_dir / "calib.json"
    if not candidate.is_file():
        raise FileNotFoundError(f"Calibration run does not contain calib.json: {calibration_run_dir}")
    return candidate


def _load_calib_payload(calib_json_path: Path) -> dict:
    return json.loads(calib_json_path.read_text(encoding="utf-8"))


def _extract_image_size(calibration_run_dir: Path) -> tuple[int, int]:
    image_candidates = sorted(calibration_run_dir.glob("*.png")) + sorted(calibration_run_dir.glob("*.jpg"))
    if not image_candidates:
        raise FileNotFoundError(
            f"Could not infer calibration image size because no PNG/JPG images were found in {calibration_run_dir}"
        )
    with Image.open(image_candidates[0]) as image:
        width, height = image.size
    return int(width), int(height)


def _tum_pose_to_matrix(values: list[float]) -> np.ndarray:
    if len(values) != 7:
        raise ValueError(f"Expected 7-element TUM pose [tx,ty,tz,qx,qy,qz,qw], got {len(values)} values")

    tx, ty, tz, qx, qy, qz, qw = [float(v) for v in values]
    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("Quaternion norm is zero in calibration pose")
    qx, qy, qz, qw = (qx / norm, qy / norm, qz / norm, qw / norm)

    rotation = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
    return matrix


def _invert_rigid_transform(matrix: np.ndarray) -> np.ndarray:
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def export_fusion_calibration_artifacts(calibration_run_dir: Path, output_dir: Path) -> tuple[Path, Path, Path]:
    calibration_run_dir = calibration_run_dir.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    calib_json_path = find_calib_json(calibration_run_dir)
    payload = _load_calib_payload(calib_json_path)

    camera = payload.get("camera", {})
    intrinsics = camera.get("intrinsics")
    distortion = camera.get("distortion_coeffs", [])
    if not isinstance(intrinsics, list) or len(intrinsics) < 4:
        raise ValueError(f"Calibration file does not contain usable camera intrinsics: {calib_json_path}")

    width, height = _extract_image_size(calibration_run_dir)
    intrinsics_json = output_dir / "intrinsics_from_calibration.json"
    intrinsics_payload = {
        "fx": float(intrinsics[0]),
        "fy": float(intrinsics[1]),
        "cx": float(intrinsics[2]),
        "cy": float(intrinsics[3]),
        "width": width,
        "height": height,
        "distortion": [float(v) for v in distortion],
        "source_calib_json": str(calib_json_path),
    }
    intrinsics_json.write_text(json.dumps(intrinsics_payload, indent=2), encoding="utf-8")

    results = payload.get("results", {})
    pose_values = results.get("T_lidar_camera") or results.get("init_T_lidar_camera")
    if not isinstance(pose_values, list):
        raise ValueError(f"Calibration file does not contain a usable LiDAR-camera transform: {calib_json_path}")

    # direct_visual_lidar_calibration writes T_lidar_camera as the inverse of the
    # optimization variable T_camera_lidar. Fusion expects T_lidar_cam to map
    # LiDAR-frame points into the camera frame before projection, so invert here.
    camera_to_lidar = _tum_pose_to_matrix(pose_values)
    lidar_to_camera = _invert_rigid_transform(camera_to_lidar)

    extrinsics_json = output_dir / "extrinsics_from_calibration.json"
    extrinsics_payload = {
        "T_lidar_cam": lidar_to_camera.tolist(),
        "source_calib_json": str(calib_json_path),
    }
    extrinsics_json.write_text(json.dumps(extrinsics_payload, indent=2), encoding="utf-8")

    return calib_json_path, intrinsics_json, extrinsics_json
