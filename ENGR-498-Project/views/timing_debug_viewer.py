from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import laspy
import numpy as np
import open3d as o3d
import pyvista as pv
from PIL import Image
from pyvistaqt import QtInteractor
from PySide6.QtCore import QSignalBlocker, Qt
from PySide6.QtGui import QCloseEvent, QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDialog,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QSlider,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from scipy.spatial.transform import Rotation

from calibration_bridge import resolve_fusion_calibration_artifacts
from fusion.native.pointcloud_accel import ensure_built, project_points_to_image
from scan_metadata import first_existing_artifact, load_scan_metadata, resolve_scan_path, save_scan_metadata
from theme import THEME, button_style, subtle_button_style
from timing_settings import (
    load_global_timing_settings,
    resolve_effective_timing,
    resolve_global_gps_settings,
    save_global_timing_settings,
)


MAX_RENDER_POINTS = 250_000
PREVIEW_SLIDER_TICKS = 10_000
COARSE_OFFSET_STEP_SEC = 0.01
FINE_OFFSET_STEP_SEC = 0.001
DEFAULT_OVERLAY_OPACITY = 0.6


class DotRadioButton(QRadioButton):
    def __init__(self, text: str, parent=None):
        super().__init__(text, parent)
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(28)

    def paintEvent(self, event):  # noqa: N802 - Qt override
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        rect = self.rect()
        indicator_diameter = 16.0
        indicator_x = 2.0
        indicator_y = (rect.height() - indicator_diameter) / 2.0

        outer_rect = rect.adjusted(int(indicator_x), int(indicator_y), 0, 0)
        outer_rect.setWidth(int(indicator_diameter))
        outer_rect.setHeight(int(indicator_diameter))

        border_color = QColor("#e2e8f0") if self.isEnabled() else QColor("#64748b")
        dot_color = QColor("#60a5fa") if self.isEnabled() else QColor("#94a3b8")
        text_color = QColor(THEME["text"]) if self.isEnabled() else QColor(THEME["muted"])

        painter.setPen(QPen(border_color, 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(outer_rect)

        if self.isChecked():
            inner_margin = 4.5
            inner_rect = outer_rect.adjusted(int(inner_margin), int(inner_margin), int(-inner_margin), int(-inner_margin))
            painter.setPen(Qt.NoPen)
            painter.setBrush(dot_color)
            painter.drawEllipse(inner_rect)

        painter.setPen(text_color)
        text_rect = rect.adjusted(28, 0, 0, 0)
        painter.drawText(text_rect, Qt.AlignVCenter | Qt.AlignLeft, self.text())
        painter.end()


class TimingQtInteractor(QtInteractor):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.minimum_dolly_step = 0.01
        self.minimum_focus_distance = 0.02

    def wheelEvent(self, event):  # noqa: N802 - Qt override
        try:
            delta = event.angleDelta().y()
            if delta == 0:
                event.accept()
                return

            camera = self.camera
            position = np.asarray(camera.position, dtype=np.float64)
            focal_point = np.asarray(camera.focal_point, dtype=np.float64)
            view_vector = focal_point - position
            distance = float(np.linalg.norm(view_vector))
            if distance <= 1e-12:
                event.accept()
                return

            direction = view_vector / distance
            wheel_steps = abs(float(delta)) / 120.0
            dolly_step = max(distance * 0.35 * wheel_steps, float(self.minimum_dolly_step) * wheel_steps)
            min_camera_distance = 1e-7

            if delta > 0:
                target_distance = distance - dolly_step
                if target_distance < min_camera_distance:
                    target_distance = min_camera_distance
                position = focal_point - direction * target_distance
            else:
                target_distance = distance + dolly_step
                position = focal_point - direction * target_distance

            camera.position = tuple(position.tolist())
            camera.focal_point = tuple(focal_point.tolist())
            self.parent()._set_permissive_clipping_range()
            self.render()
            event.accept()
            return
        except Exception:
            pass

        super().wheelEvent(event)


@dataclass(frozen=True)
class KeyframeSet:
    label: str
    positions: np.ndarray
    directions: np.ndarray
    color: str


@dataclass(frozen=True)
class PoseRecord:
    timestamp: float
    translation: np.ndarray
    quaternion_xyzw: np.ndarray


@dataclass(frozen=True)
class GpsPoseRecord:
    t_query_sec: float
    translation_xyz: np.ndarray
    quaternion_xyzw: np.ndarray


@dataclass(frozen=True)
class DenseTrajectory:
    timestamps: np.ndarray
    translations: np.ndarray
    quaternions: np.ndarray


@dataclass(frozen=True)
class DenseTranslationTrajectory:
    timestamps: np.ndarray
    translations: np.ndarray


@dataclass(frozen=True)
class PreviewIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion: np.ndarray


@dataclass(frozen=True)
class CameraPreviewFrame:
    image_name: str
    image_path: Path
    timestamp: float
    elapsed_sec: float | None
    original_pose: PoseRecord


@dataclass(frozen=True)
class PreviewCalibration:
    calibration_run: Path
    calibration_source: str
    intrinsics_json: Path
    extrinsics_json: Path
    intrinsics: PreviewIntrinsics
    lidar_to_camera: np.ndarray
    camera_to_lidar: np.ndarray


@dataclass
class CameraPreviewState:
    selected_frame_index: int = 0
    selected_frame_name: str = ""
    selected_frame_timestamp: float = 0.0
    preview_offset_sec: float = 0.0
    overlay_opacity: float = DEFAULT_OVERLAY_OPACITY
    adjustment_mode: str = "coarse"


@dataclass(frozen=True)
class PreviewProjectionDiagnostics:
    front_of_camera_points: int = 0
    in_image_points: int = 0
    kept_points: int = 0
    unique_hit_pixels: int = 0
    multi_hit_pixels: int = 0
    avg_points_per_hit_pixel: float = 0.0
    max_points_per_hit_pixel: int = 0
    border_hits_before_filter: int = 0
    border_hits_after_filter: int = 0


def _gps_optional_enabled(metadata: dict[str, Any]) -> tuple[bool, str]:
    gps_cfg = metadata.get("config", {}).get("gps", {})
    per_scan_enabled = bool(gps_cfg.get("allow_missing", False))
    global_enabled = bool(resolve_global_gps_settings().get("allow_missing", False))
    if per_scan_enabled:
        return True, "per-scan"
    if global_enabled:
        return True, "global"
    return False, "disabled"


def _empty_gps_keyframe_sets() -> dict[str, KeyframeSet]:
    return {
        "original": KeyframeSet(
            label="Original GPS keyframes",
            positions=np.empty((0, 3), dtype=np.float64),
            directions=np.empty((0, 3), dtype=np.float64),
            color="#3b82f6",
        ),
        "shifted": KeyframeSet(
            label="Shifted GPS keyframes",
            positions=np.empty((0, 3), dtype=np.float64),
            directions=np.empty((0, 3), dtype=np.float64),
            color="#f59e0b",
        ),
    }


def _safe_normalize(values: np.ndarray) -> np.ndarray:
    denom = np.linalg.norm(values, axis=1, keepdims=True)
    denom[denom <= 1e-12] = 1.0
    return values / denom


def _normalize_quaternion_xyzw(quaternion: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quaternion / norm


def _slerp_quaternion_xyzw(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0n = _normalize_quaternion_xyzw(q0.astype(np.float64, copy=False))
    q1n = _normalize_quaternion_xyzw(q1.astype(np.float64, copy=False))
    dot = float(np.dot(q0n, q1n))
    if dot < 0.0:
        q1n = -q1n
        dot = -dot
    dot = max(min(dot, 1.0), -1.0)
    if dot > 0.9995:
        return _normalize_quaternion_xyzw(q0n + alpha * (q1n - q0n))
    theta_0 = float(np.arccos(dot))
    sin_theta_0 = float(np.sin(theta_0))
    if abs(sin_theta_0) < 1e-10:
        return _normalize_quaternion_xyzw(q0n)
    theta = theta_0 * alpha
    s0 = float(np.sin(theta_0 - theta) / sin_theta_0)
    s1 = float(np.sin(theta) / sin_theta_0)
    return _normalize_quaternion_xyzw((s0 * q0n) + (s1 * q1n))


def load_pose_records(path: Path, time_column: str) -> list[PoseRecord]:
    poses: list[PoseRecord] = []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            status = str(row.get("status", "OK")).strip().upper()
            if status and status != "OK":
                continue
            try:
                poses.append(
                    PoseRecord(
                        timestamp=float(row[time_column]),
                        translation=np.asarray([float(row["x"]), float(row["y"]), float(row["z"])], dtype=np.float64),
                        quaternion_xyzw=np.asarray(
                            [float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])],
                            dtype=np.float64,
                        ),
                    )
                )
            except (KeyError, ValueError):
                continue
    if not poses:
        raise ValueError(f"No valid poses were loaded from {path}")
    poses.sort(key=lambda record: record.timestamp)
    return poses


def interpolate_pose_record(
    poses: list[PoseRecord],
    pose_times: np.ndarray,
    target_time: float,
) -> PoseRecord | None:
    if pose_times.size == 0:
        return None
    if target_time < float(pose_times[0]) or target_time > float(pose_times[-1]):
        return None
    right = int(np.searchsorted(pose_times, target_time, side="left"))
    if right <= 0:
        return poses[0]
    if right >= pose_times.size:
        return poses[-1]
    left = right - 1
    t0 = float(pose_times[left])
    t1 = float(pose_times[right])
    if t1 <= t0:
        return None
    alpha = max(0.0, min(1.0, float((target_time - t0) / (t1 - t0))))
    p0 = poses[left]
    p1 = poses[right]
    return PoseRecord(
        timestamp=float(target_time),
        translation=((1.0 - alpha) * p0.translation + alpha * p1.translation).astype(np.float64),
        quaternion_xyzw=_slerp_quaternion_xyzw(p0.quaternion_xyzw, p1.quaternion_xyzw, alpha).astype(np.float64),
    )


def load_dense_trajectory(path: Path) -> DenseTrajectory:
    timestamps = []
    translations = []
    quaternions = []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            status = str(row.get("status", "")).strip().upper()
            if status != "OK":
                continue
            try:
                timestamps.append(float(row["timestamp_sec"]))
                translations.append([float(row["x"]), float(row["y"]), float(row["z"])])
                quaternions.append([float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])])
            except (KeyError, ValueError):
                continue
    if not timestamps:
        raise ValueError(f"No valid dense trajectory rows were loaded from {path}")
    order = np.argsort(np.asarray(timestamps, dtype=np.float64), kind="stable")
    ts = np.asarray([timestamps[int(i)] for i in order], dtype=np.float64)
    tr = np.asarray([translations[int(i)] for i in order], dtype=np.float64)
    qt = np.asarray([quaternions[int(i)] for i in order], dtype=np.float64)
    qt = np.asarray([_normalize_quaternion_xyzw(q) for q in qt], dtype=np.float64)
    for i in range(1, qt.shape[0]):
        if float(np.dot(qt[i - 1], qt[i])) < 0.0:
            qt[i] = -qt[i]
    return DenseTrajectory(timestamps=ts, translations=tr, quaternions=qt)


def query_dense_trajectory(traj: DenseTrajectory, target_time: float) -> PoseRecord | None:
    if traj.timestamps.size == 0:
        return None
    if target_time < float(traj.timestamps[0]) or target_time > float(traj.timestamps[-1]):
        return None
    right = int(np.searchsorted(traj.timestamps, target_time, side="left"))
    if right <= 0:
        return PoseRecord(float(traj.timestamps[0]), traj.translations[0].copy(), traj.quaternions[0].copy())
    if right >= traj.timestamps.size:
        return PoseRecord(float(traj.timestamps[-1]), traj.translations[-1].copy(), traj.quaternions[-1].copy())
    left = right - 1
    t0 = float(traj.timestamps[left])
    t1 = float(traj.timestamps[right])
    if t1 <= t0:
        return None
    alpha = max(0.0, min(1.0, float((target_time - t0) / (t1 - t0))))
    return PoseRecord(
        timestamp=float(target_time),
        translation=((1.0 - alpha) * traj.translations[left] + alpha * traj.translations[right]).astype(np.float64),
        quaternion_xyzw=_slerp_quaternion_xyzw(traj.quaternions[left], traj.quaternions[right], alpha).astype(np.float64),
    )


def load_tf_gps_records(path: Path) -> list[GpsPoseRecord]:
    records: list[GpsPoseRecord] = []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            status = str(row.get("status", "OK")).strip().upper()
            if status and status != "OK":
                continue
            latitude = str(row.get("latitude_deg", "")).strip()
            longitude = str(row.get("longitude_deg", "")).strip()
            altitude = str(row.get("altitude_m", "")).strip()
            if not latitude or not longitude or not altitude:
                continue
            try:
                records.append(
                    GpsPoseRecord(
                        t_query_sec=float(row["t_query_sec"]),
                        translation_xyz=np.asarray([float(row["x"]), float(row["y"]), float(row["z"])], dtype=np.float64),
                        quaternion_xyzw=np.asarray(
                            [float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])],
                            dtype=np.float64,
                        ),
                    )
                )
            except (KeyError, ValueError):
                continue
    if not records:
        raise ValueError(f"No valid GPS/TF rows were loaded from {path}")
    return records


def load_dense_translation_trajectory(path: Path) -> DenseTranslationTrajectory:
    dense = load_dense_trajectory(path)
    return DenseTranslationTrajectory(timestamps=dense.timestamps, translations=dense.translations)


def interpolate_dense_translation(traj: DenseTranslationTrajectory, target_time: float) -> np.ndarray | None:
    if traj.timestamps.size == 0:
        return None
    if target_time < float(traj.timestamps[0]) or target_time > float(traj.timestamps[-1]):
        return None
    right = int(np.searchsorted(traj.timestamps, target_time, side="left"))
    if right <= 0:
        return traj.translations[0].copy()
    if right >= traj.timestamps.size:
        return traj.translations[-1].copy()
    left = right - 1
    t0 = float(traj.timestamps[left])
    t1 = float(traj.timestamps[right])
    if t1 <= t0:
        return None
    alpha = max(0.0, min(1.0, float((target_time - t0) / (t1 - t0))))
    return ((1.0 - alpha) * traj.translations[left] + alpha * traj.translations[right]).astype(np.float64)


def _sample_points(points: np.ndarray, colors: np.ndarray, max_points: int = MAX_RENDER_POINTS) -> tuple[np.ndarray, np.ndarray]:
    if points.shape[0] <= max_points:
        return points, colors
    step = max(1, points.shape[0] // max_points)
    return points[::step], colors[::step]


def _neutral_gray(points: np.ndarray, shade: float = 0.65) -> np.ndarray:
    return np.full((points.shape[0], 3), shade, dtype=np.float32)


def _intensity_to_gray(intensity: np.ndarray) -> np.ndarray:
    values = np.asarray(intensity, dtype=np.float64).reshape(-1)
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    lo = float(np.percentile(finite, 2))
    hi = float(np.percentile(finite, 98))
    if hi <= lo:
        scaled = np.full(values.shape[0], 0.65, dtype=np.float32)
    else:
        scaled = np.clip((values - lo) / (hi - lo), 0.0, 1.0).astype(np.float32)
    scaled = 0.2 + 0.8 * scaled
    return np.repeat(scaled[:, None], 3, axis=1)


def load_renderable_point_cloud(path: Path) -> tuple[np.ndarray, np.ndarray, str]:
    suffix = path.suffix.lower()
    if suffix in {".las", ".laz"}:
        las = laspy.read(path)
        points = np.column_stack((las.x, las.y, las.z)).astype(np.float32)
        if points.size == 0:
            raise ValueError(f"Point cloud is empty: {path}")
        if hasattr(las, "intensity"):
            colors = _intensity_to_gray(np.asarray(las.intensity))
            if colors.shape[0] == points.shape[0]:
                sampled_points, sampled_colors = _sample_points(points, colors.astype(np.float32))
                return sampled_points, sampled_colors, "intensity"
        sampled_points, sampled_colors = _sample_points(points, _neutral_gray(points))
        return sampled_points, sampled_colors, "grayscale"

    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points, dtype=np.float32)
    if points.size == 0:
        raise ValueError(f"Point cloud is empty: {path}")
    sampled_points, sampled_colors = _sample_points(points, _neutral_gray(points))
    return sampled_points, sampled_colors, "grayscale"


def _camera_direction_from_quaternions(quaternions_xyzw: np.ndarray) -> np.ndarray:
    return Rotation.from_quat(quaternions_xyzw).apply(np.tile(np.array([1.0, 0.0, 0.0]), (quaternions_xyzw.shape[0], 1)))


def _gps_heading_from_quaternions(quaternions_xyzw: np.ndarray) -> np.ndarray:
    direction = Rotation.from_quat(quaternions_xyzw).apply(np.tile(np.array([1.0, 0.0, 0.0]), (quaternions_xyzw.shape[0], 1)))
    direction[:, 2] = 0.0
    return _safe_normalize(direction)


def _timestamp_key(timestamp: float) -> int:
    return int(round(float(timestamp) * 1_000_000_000.0))


def _load_preview_intrinsics(path: Path) -> PreviewIntrinsics:
    payload = json.loads(path.read_text(encoding="utf-8"))
    distortion = np.asarray(payload.get("distortion", []), dtype=np.float64).reshape(-1)
    return PreviewIntrinsics(
        fx=float(payload["fx"]),
        fy=float(payload["fy"]),
        cx=float(payload["cx"]),
        cy=float(payload["cy"]),
        width=int(payload["width"]),
        height=int(payload["height"]),
        distortion=distortion[:8].astype(np.float64, copy=False),
    )


def _load_preview_extrinsics(path: Path) -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    matrix = np.asarray(payload["T_lidar_cam"], dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"Expected a 4x4 T_lidar_cam matrix in {path}")
    return matrix


def _transform_from_pose(record: PoseRecord) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_quat(record.quaternion_xyzw).as_matrix()
    transform[:3, 3] = record.translation
    return transform


def _invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def _transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    return points @ transform[:3, :3].T + transform[:3, 3]


def _border_pixel_mask(
    u: np.ndarray,
    v: np.ndarray,
    width: int,
    height: int,
    *,
    margin_fraction: float = 0.05,
) -> np.ndarray:
    if u.size == 0:
        return np.zeros(0, dtype=bool)
    margin_x = max(1, int(round(float(width) * margin_fraction)))
    margin_y = max(1, int(round(float(height) * margin_fraction)))
    return (
        (u < margin_x)
        | (u >= width - margin_x)
        | (v < margin_y)
        | (v >= height - margin_y)
    )


def _load_image_timestamp_rows(path: Path) -> list[tuple[str, float, float | None]]:
    rows: list[tuple[str, float, float | None]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        if "filename" not in fieldnames:
            raise ValueError(f"image_timestamps.csv is missing a filename column: {path}")
        time_column = "t_query_sec" if "t_query_sec" in fieldnames else None
        if time_column is None:
            for candidate in ("timestamp", "time_sec", "t_in_sec"):
                if candidate in fieldnames:
                    time_column = candidate
                    break
        if time_column is None:
            raise ValueError(f"Could not find an image timestamp column in {path}")
        elapsed_column = "t_in_sec" if "t_in_sec" in fieldnames else None
        for row in reader:
            image_name = Path(str(row.get("filename", "")).strip()).name
            if not image_name:
                continue
            try:
                timestamp = float(row[time_column])
            except (TypeError, ValueError):
                continue
            elapsed_sec: float | None = None
            if elapsed_column:
                raw_elapsed = str(row.get(elapsed_column, "")).strip()
                if raw_elapsed:
                    try:
                        elapsed_sec = float(raw_elapsed)
                    except ValueError:
                        elapsed_sec = None
            rows.append((image_name, timestamp, elapsed_sec))
    if not rows:
        raise ValueError(f"No image timestamps were loaded from {path}")
    return rows


def _build_camera_preview_frames(scan_dir: Path, metadata: dict, camera_poses: list[PoseRecord]) -> list[CameraPreviewFrame]:
    files = metadata.get("files", {})
    images_dir = resolve_scan_path(scan_dir, files.get("images_dir"))
    timestamps_csv = resolve_scan_path(scan_dir, files.get("image_timestamps_csv"))
    if images_dir is None or not images_dir.is_dir():
        raise FileNotFoundError("Pose recovery images folder not found for this scan.")
    if timestamps_csv is None or not timestamps_csv.is_file():
        raise FileNotFoundError("image_timestamps.csv not found for this scan.")

    pose_by_time = {_timestamp_key(record.timestamp): record for record in camera_poses}
    frames: list[CameraPreviewFrame] = []
    for image_name, timestamp, elapsed_sec in _load_image_timestamp_rows(timestamps_csv):
        pose = pose_by_time.get(_timestamp_key(timestamp))
        if pose is None:
            continue
        image_path = images_dir / image_name
        if not image_path.is_file():
            continue
        frames.append(
            CameraPreviewFrame(
                image_name=image_name,
                image_path=image_path,
                timestamp=float(timestamp),
                elapsed_sec=elapsed_sec,
                original_pose=pose,
            )
        )
    if not frames:
        raise ValueError("No pose recovery camera keyframes with matching images/timestamps were found.")
    return frames


def _load_preview_calibration(scan_dir: Path, metadata: dict) -> PreviewCalibration:
    calibration_run, intrinsics_json, extrinsics_json, calibration_source = resolve_fusion_calibration_artifacts(
        scan_dir,
        metadata,
        persist_metadata=True,
    )
    intrinsics = _load_preview_intrinsics(intrinsics_json)
    lidar_to_camera = _load_preview_extrinsics(extrinsics_json)
    camera_to_lidar = _invert_transform(lidar_to_camera)
    return PreviewCalibration(
        calibration_run=calibration_run,
        calibration_source=calibration_source,
        intrinsics_json=intrinsics_json,
        extrinsics_json=extrinsics_json,
        intrinsics=intrinsics,
        lidar_to_camera=lidar_to_camera,
        camera_to_lidar=camera_to_lidar,
    )


def _build_camera_keyframes(scan_dir: Path, metadata: dict, effective_timing: dict) -> dict[str, KeyframeSet]:
    files = metadata.get("files", {})
    tf_camera_csv = resolve_scan_path(scan_dir, files.get("tf_camera_csv"))
    if tf_camera_csv is None or not tf_camera_csv.is_file():
        raise FileNotFoundError("Camera TF CSV not found for this scan.")

    poses = load_pose_records(tf_camera_csv, "t_query_sec")
    pose_times = np.asarray([record.timestamp for record in poses], dtype=np.float64)
    original_positions = np.asarray([record.translation for record in poses], dtype=np.float64)
    original_quats = np.asarray([record.quaternion_xyzw for record in poses], dtype=np.float64)

    shifted_positions = original_positions.copy()
    shifted_quats = original_quats.copy()

    offset_sec = float(effective_timing.get("fusion_time_offset_sec", 0.0))
    dense_path = resolve_scan_path(scan_dir, files.get("tf_dense_traj_csv"))
    dense_traj = None
    if dense_path is not None and dense_path.is_file():
        try:
            dense_traj = load_dense_trajectory(dense_path)
        except Exception:
            dense_traj = None

    if abs(offset_sec) > 1e-12:
        shifted_records = []
        for record in poses:
            t_effective = float(record.timestamp) + offset_sec
            shifted_record = None
            if dense_traj is not None:
                shifted_record = query_dense_trajectory(dense_traj, t_effective)
            if shifted_record is None:
                shifted_record = interpolate_pose_record(poses, pose_times, t_effective)
            if shifted_record is None:
                continue
            shifted_records.append(shifted_record)
        if shifted_records:
            shifted_positions = np.asarray([record.translation for record in shifted_records], dtype=np.float64)
            shifted_quats = np.asarray([record.quaternion_xyzw for record in shifted_records], dtype=np.float64)

    return {
        "original": KeyframeSet(
            label="Original camera keyframes",
            positions=original_positions,
            directions=_safe_normalize(_camera_direction_from_quaternions(original_quats)),
            color="#ef4444",
        ),
        "shifted": KeyframeSet(
            label="Shifted camera keyframes",
            positions=shifted_positions,
            directions=_safe_normalize(_camera_direction_from_quaternions(shifted_quats)),
            color="#22c55e",
        ),
    }


def _build_gps_keyframes(scan_dir: Path, metadata: dict, effective_timing: dict) -> dict[str, KeyframeSet]:
    files = metadata.get("files", {})
    tf_gps_csv = resolve_scan_path(scan_dir, files.get("tf_gps_csv"))
    if tf_gps_csv is None or not tf_gps_csv.is_file():
        raise FileNotFoundError("GPS TF CSV not found for this scan.")

    records = load_tf_gps_records(tf_gps_csv)
    original_positions = np.asarray([record.translation_xyz for record in records], dtype=np.float64)
    original_quats = np.asarray([record.quaternion_xyzw for record in records], dtype=np.float64)
    original_directions = _safe_normalize(_gps_heading_from_quaternions(original_quats))
    shifted_positions = original_positions.copy()
    shifted_directions = original_directions.copy()

    offset_sec = float(effective_timing.get("gps_time_offset_sec", 0.0))
    dense_path = resolve_scan_path(scan_dir, files.get("tf_dense_traj_csv"))
    dense_traj = None
    if dense_path is not None and dense_path.is_file():
        try:
            dense_traj = load_dense_translation_trajectory(dense_path)
        except Exception:
            dense_traj = None

    if abs(offset_sec) > 1e-12 and dense_traj is not None:
        shifted_positions_list = []
        kept_quats = []
        for record in records:
            translation = interpolate_dense_translation(dense_traj, float(record.t_query_sec) + offset_sec)
            if translation is None:
                continue
            shifted_positions_list.append(translation)
            kept_quats.append(record.quaternion_xyzw)
        if shifted_positions_list:
            shifted_positions = np.asarray(shifted_positions_list, dtype=np.float64)
            shifted_directions = _safe_normalize(_gps_heading_from_quaternions(np.asarray(kept_quats, dtype=np.float64)))
    return {
        "original": KeyframeSet(
            label="Original GPS keyframes",
            positions=original_positions,
            directions=original_directions,
            color="#3b82f6",
        ),
        "shifted": KeyframeSet(
            label="Shifted GPS keyframes",
            positions=shifted_positions,
            directions=shifted_directions,
            color="#f59e0b",
        ),
    }


class TimingDebugViewerDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Timing / Offset Viewer")
        self.resize(1680, 820)
        self.setAttribute(Qt.WA_DeleteOnClose, False)

        self.scan_dir: Path | None = None
        self.metadata: dict = {}
        self.effective_timing: dict = {}
        self.point_cloud_points = np.empty((0, 3), dtype=np.float32)
        self.point_cloud_points64 = np.empty((0, 3), dtype=np.float64)
        self.point_cloud_colors = np.empty((0, 3), dtype=np.float32)
        self.point_cloud_rgb = np.empty((0, 3), dtype=np.uint8)
        self.point_cloud_mode = "grayscale"
        self.point_cloud_mesh: pv.PolyData | None = None
        self.dense_trajectory: DenseTrajectory | None = None
        self.trajectory_points = np.empty((0, 3), dtype=np.float64)
        self.keyframes_by_tab: dict[str, dict[str, KeyframeSet]] = {}
        self.current_mode = {"camera": "original", "gps": "original"}
        self.camera_preview_frames: list[CameraPreviewFrame] = []
        self.camera_preview_state = CameraPreviewState()
        self.camera_preview_calibration: PreviewCalibration | None = None
        self.camera_preview_available = False
        self.camera_preview_error = ""
        self.camera_preview_visible_indices = np.empty(0, dtype=np.int64)
        self.camera_preview_visible_rgb = np.empty((0, 3), dtype=np.float32)
        self.camera_preview_effective_time: float | None = None
        self.camera_preview_selected_pose: PoseRecord | None = None
        self.camera_preview_selected_camera_position: np.ndarray | None = None
        self.camera_preview_selected_camera_direction: np.ndarray | None = None
        self.camera_preview_visible_count = 0
        self.camera_preview_last_drop_reason = ""
        self.camera_preview_apply_distortion = False
        self.camera_preview_frontmost_only = True
        self.camera_preview_diagnostics = PreviewProjectionDiagnostics()
        self.camera_preview_cached_image_name = ""
        self.camera_preview_cached_image_rgb: np.ndarray | None = None
        self.camera_preview_display_pixmap: QPixmap | None = None
        self.gps_optional_enabled = False
        self.gps_mode_source = "disabled"
        self.gps_available = True
        self._suspend_preview_updates = False
        self._has_initialized_camera = False

        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout()
        self.setLayout(layout)

        header = QLabel("Timing / Offset Viewer")
        header.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {THEME['text']};")
        layout.addWidget(header)

        self.summary_label = QLabel("Select a scan to inspect timing offsets.")
        self.summary_label.setWordWrap(True)
        self.summary_label.setStyleSheet(f"color: {THEME['muted']};")
        layout.addWidget(self.summary_label)

        content = QHBoxLayout()
        layout.addLayout(content, stretch=1)

        self.plotter = TimingQtInteractor(self)
        self.plotter.set_background(THEME["pane"])
        self.plotter.enable_anti_aliasing("msaa")
        try:
            self.plotter.enable_custom_trackball_style(left="pan", right="rotate", middle="pan")
        except Exception:
            pass
        content.addWidget(self.plotter, stretch=1)

        side = QFrame()
        side.setStyleSheet(
            f"QFrame {{ background-color: {THEME['pane']}; border: 1px solid {THEME['border']}; border-radius: {THEME['radius_md']}; }}"
        )
        side_layout = QVBoxLayout()
        side.setLayout(side_layout)
        content.addWidget(side, stretch=0)

        self.tab_widget = QTabWidget()
        self.tab_widget.currentChanged.connect(self._render_active_tab)
        side_layout.addWidget(self.tab_widget, stretch=1)

        self.camera_tab = self._build_tab("camera", "Camera Keyframes")
        self.gps_tab = self._build_tab("gps", "GPS Keyframes")
        self.tab_widget.addTab(self.camera_tab, "Camera Keyframes")
        self.tab_widget.addTab(self.gps_tab, "GPS Keyframes")

        self.status_label = QLabel("")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet(f"color: {THEME['muted']};")
        side_layout.addWidget(self.status_label)

        image_side = QFrame()
        image_side.setStyleSheet(
            f"QFrame {{ background-color: {THEME['pane']}; border: 1px solid {THEME['border']}; border-radius: {THEME['radius_md']}; }}"
        )
        image_side_layout = QVBoxLayout()
        image_side.setLayout(image_side_layout)
        content.addWidget(image_side, stretch=0)

        image_title = QLabel("Selected Camera Image")
        image_title.setStyleSheet(f"font-size: 15px; font-weight: bold; color: {THEME['text']};")
        image_side_layout.addWidget(image_title)

        self.image_preview_label = QLabel("Selected image preview unavailable.")
        self.image_preview_label.setAlignment(Qt.AlignCenter)
        self.image_preview_label.setMinimumSize(320, 320)
        self.image_preview_label.setStyleSheet(
            f"background-color: {THEME['input']}; color: {THEME['muted']}; border: 1px solid {THEME['border']}; border-radius: {THEME['radius_sm']}; padding: 8px;"
        )
        image_side_layout.addWidget(self.image_preview_label, stretch=1)

        self.image_caption_label = QLabel("")
        self.image_caption_label.setWordWrap(True)
        self.image_caption_label.setStyleSheet(f"color: {THEME['muted']};")
        image_side_layout.addWidget(self.image_caption_label)

        button_row = QHBoxLayout()
        close_btn = QPushButton("Close")
        close_btn.setStyleSheet(subtle_button_style())
        close_btn.clicked.connect(self.hide)
        reset_btn = QPushButton("Reset View")
        reset_btn.setStyleSheet(button_style(THEME["accent"], THEME["accent_hover"]))
        reset_btn.clicked.connect(self._reset_camera)
        button_row.addWidget(reset_btn)
        button_row.addWidget(close_btn)
        layout.addLayout(button_row)

    def _build_tab(self, tab_key: str, title: str) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout()
        widget.setLayout(layout)

        group_style = (
            f"""
            QGroupBox {{
                color: {THEME['text']};
                border: 1px solid {THEME['border']};
                border-radius: {THEME['radius_md']};
                margin-top: 12px;
                padding: 10px 8px 8px 8px;
                font-weight: 600;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
            }}
            QRadioButton {{
                color: {THEME['text']};
                spacing: 10px;
                padding: 4px 0;
            }}
            QRadioButton::indicator {{
                width: 16px;
                height: 16px;
                border-radius: 8px;
                border: 2px solid #cbd5e1;
                background-color: #0f172a;
            }}
            QRadioButton::indicator:checked {{
                border: 2px solid #60a5fa;
                background-color: #0f172a;
            }}
            QRadioButton::indicator:checked::after {{
                background-color: #60a5fa;
            }}
            """
        )
        input_style = (
            f"QComboBox {{ background: {THEME['input']}; color: {THEME['text']}; border: 1px solid {THEME['border']}; "
            f"border-radius: {THEME['radius_sm']}; padding: 6px 8px; }}"
        )

        title_label = QLabel(title)
        title_label.setStyleSheet(f"font-size: 15px; font-weight: bold; color: {THEME['text']};")
        layout.addWidget(title_label)

        mode_group = QGroupBox("Keyframe Display")
        mode_layout = QVBoxLayout()
        mode_group.setLayout(mode_layout)
        mode_group.setStyleSheet(group_style)

        original_btn = DotRadioButton("Original keyframes")
        shifted_btn = DotRadioButton("Shifted keyframes")
        original_btn.setChecked(True)
        original_btn.toggled.connect(lambda checked, key=tab_key: self._set_mode(key, "original", checked))
        shifted_btn.toggled.connect(lambda checked, key=tab_key: self._set_mode(key, "shifted", checked))
        radio_group = QButtonGroup(widget)
        radio_group.addButton(original_btn)
        radio_group.addButton(shifted_btn)
        mode_layout.addWidget(original_btn)
        mode_layout.addWidget(shifted_btn)
        layout.addWidget(mode_group)

        if tab_key == "camera":
            preview_group = QGroupBox("Interactive Alignment")
            preview_group.setStyleSheet(group_style)
            preview_layout = QVBoxLayout()
            preview_group.setLayout(preview_layout)

            frame_label = QLabel("Camera keyframe")
            frame_label.setStyleSheet(f"color: {THEME['text']};")
            preview_layout.addWidget(frame_label)

            frame_row = QHBoxLayout()

            prev_frame_btn = QPushButton("<")
            prev_frame_btn.setStyleSheet(subtle_button_style())
            prev_frame_btn.setEnabled(False)
            prev_frame_btn.clicked.connect(lambda: self._step_preview_frame(-1))
            frame_row.addWidget(prev_frame_btn)

            frame_combo = QComboBox()
            frame_combo.setStyleSheet(input_style)
            frame_combo.setEnabled(False)
            frame_combo.setMaxVisibleItems(20)
            frame_combo.currentIndexChanged.connect(self._on_preview_frame_changed)
            frame_row.addWidget(frame_combo, stretch=1)

            next_frame_btn = QPushButton(">")
            next_frame_btn.setStyleSheet(subtle_button_style())
            next_frame_btn.setEnabled(False)
            next_frame_btn.clicked.connect(lambda: self._step_preview_frame(1))
            frame_row.addWidget(next_frame_btn)
            preview_layout.addLayout(frame_row)

            offset_header = QLabel("Time offset")
            offset_header.setStyleSheet(f"color: {THEME['text']};")
            preview_layout.addWidget(offset_header)

            offset_slider = QSlider(Qt.Horizontal)
            offset_slider.setRange(-PREVIEW_SLIDER_TICKS, PREVIEW_SLIDER_TICKS)
            offset_slider.setTracking(True)
            offset_slider.setEnabled(False)
            offset_slider.valueChanged.connect(self._on_preview_offset_changed)
            preview_layout.addWidget(offset_slider)

            offset_value = QLabel("0.000000 s")
            offset_value.setStyleSheet(f"color: {THEME['muted']}; font-weight: 600;")
            preview_layout.addWidget(offset_value)

            mode_label = QLabel("Adjustment mode")
            mode_label.setStyleSheet(f"color: {THEME['text']};")
            preview_layout.addWidget(mode_label)

            mode_row = QHBoxLayout()
            coarse_btn = DotRadioButton("Coarse")
            fine_btn = DotRadioButton("Fine")
            coarse_btn.setChecked(True)
            coarse_btn.setEnabled(False)
            fine_btn.setEnabled(False)
            coarse_btn.toggled.connect(lambda checked: self._on_preview_mode_changed("coarse", checked))
            fine_btn.toggled.connect(lambda checked: self._on_preview_mode_changed("fine", checked))
            preview_mode_group = QButtonGroup(widget)
            preview_mode_group.addButton(coarse_btn)
            preview_mode_group.addButton(fine_btn)
            mode_row.addWidget(coarse_btn)
            mode_row.addWidget(fine_btn)
            preview_layout.addLayout(mode_row)

            opacity_header = QLabel("Overlay opacity")
            opacity_header.setStyleSheet(f"color: {THEME['text']};")
            preview_layout.addWidget(opacity_header)

            opacity_slider = QSlider(Qt.Horizontal)
            opacity_slider.setRange(0, 100)
            opacity_slider.setValue(int(round(DEFAULT_OVERLAY_OPACITY * 100.0)))
            opacity_slider.setEnabled(False)
            opacity_slider.valueChanged.connect(self._on_overlay_opacity_changed)
            preview_layout.addWidget(opacity_slider)

            opacity_value = QLabel(f"{int(round(DEFAULT_OVERLAY_OPACITY * 100.0))}%")
            opacity_value.setStyleSheet(f"color: {THEME['muted']}; font-weight: 600;")
            preview_layout.addWidget(opacity_value)

            projection_header = QLabel("Projection debug")
            projection_header.setStyleSheet(f"color: {THEME['text']};")
            preview_layout.addWidget(projection_header)

            distortion_checkbox = QCheckBox("Apply calibration distortion")
            distortion_checkbox.setStyleSheet(f"color: {THEME['text']};")
            distortion_checkbox.setChecked(False)
            distortion_checkbox.setEnabled(False)
            distortion_checkbox.toggled.connect(self._on_preview_projection_options_changed)
            preview_layout.addWidget(distortion_checkbox)

            frontmost_checkbox = QCheckBox("Keep nearest point per pixel only")
            frontmost_checkbox.setStyleSheet(f"color: {THEME['text']};")
            frontmost_checkbox.setChecked(True)
            frontmost_checkbox.setEnabled(False)
            frontmost_checkbox.toggled.connect(self._on_preview_projection_options_changed)
            preview_layout.addWidget(frontmost_checkbox)

            diagnostics_label = QLabel("Projection diagnostics unavailable.")
            diagnostics_label.setWordWrap(True)
            diagnostics_label.setStyleSheet(f"color: {THEME['muted']};")
            preview_layout.addWidget(diagnostics_label)

            save_btn = QPushButton("Save Time Offset")
            save_btn.setStyleSheet(button_style(THEME["accent"], THEME["accent_hover"]))
            save_btn.setEnabled(False)
            save_btn.clicked.connect(self._save_time_offset)
            preview_layout.addWidget(save_btn)

            widget.frame_combo = frame_combo  # type: ignore[attr-defined]
            widget.prev_frame_btn = prev_frame_btn  # type: ignore[attr-defined]
            widget.next_frame_btn = next_frame_btn  # type: ignore[attr-defined]
            widget.offset_slider = offset_slider  # type: ignore[attr-defined]
            widget.offset_value_label = offset_value  # type: ignore[attr-defined]
            widget.coarse_btn = coarse_btn  # type: ignore[attr-defined]
            widget.fine_btn = fine_btn  # type: ignore[attr-defined]
            widget.opacity_slider = opacity_slider  # type: ignore[attr-defined]
            widget.opacity_value_label = opacity_value  # type: ignore[attr-defined]
            widget.distortion_checkbox = distortion_checkbox  # type: ignore[attr-defined]
            widget.frontmost_checkbox = frontmost_checkbox  # type: ignore[attr-defined]
            widget.diagnostics_label = diagnostics_label  # type: ignore[attr-defined]
            widget.preview_mode_group = preview_mode_group  # type: ignore[attr-defined]
            widget.save_btn = save_btn  # type: ignore[attr-defined]
            layout.addWidget(preview_group)

        info = QLabel()
        info.setWordWrap(True)
        info.setStyleSheet(f"color: {THEME['muted']};")
        layout.addWidget(info)
        layout.addStretch()

        widget.original_btn = original_btn  # type: ignore[attr-defined]
        widget.shifted_btn = shifted_btn  # type: ignore[attr-defined]
        widget.info_label = info  # type: ignore[attr-defined]
        widget.radio_group = radio_group  # type: ignore[attr-defined]
        return widget

    def load_scan(self, scan_ref: str | Path, *, restore_preview: dict[str, Any] | None = None) -> None:
        restore_preview = dict(restore_preview or {})
        scan_dir, metadata = load_scan_metadata(scan_ref)
        effective_timing = resolve_effective_timing(metadata)

        _, point_cloud_path = first_existing_artifact(scan_dir, metadata, ("filtered_las", "filtered_pcd", "raw_las", "raw_pcd", "las", "pcd"))
        if point_cloud_path is None:
            raise FileNotFoundError("No filtered or raw point cloud is available for this scan.")

        dense_path = resolve_scan_path(scan_dir, metadata.get("files", {}).get("tf_dense_traj_csv"))
        if dense_path is None or not dense_path.is_file():
            raise FileNotFoundError("Dense trajectory CSV not found for this scan.")

        points, colors, mode = load_renderable_point_cloud(point_cloud_path)
        trajectory = load_dense_trajectory(dense_path)

        self.scan_dir = scan_dir
        self.metadata = metadata
        self.effective_timing = effective_timing
        self.gps_optional_enabled, self.gps_mode_source = _gps_optional_enabled(metadata)
        self.gps_available = True
        self.point_cloud_points = points
        self.point_cloud_points64 = points.astype(np.float64)
        self.point_cloud_colors = colors
        self.point_cloud_rgb = (255.0 * np.clip(colors, 0.0, 1.0)).astype(np.uint8)
        self.point_cloud_mode = mode
        self.point_cloud_mesh = None
        self.dense_trajectory = trajectory
        self.trajectory_points = trajectory.translations.astype(np.float64)
        self.keyframes_by_tab = {"camera": _build_camera_keyframes(scan_dir, metadata, effective_timing)}
        try:
            self.keyframes_by_tab["gps"] = _build_gps_keyframes(scan_dir, metadata, effective_timing)
        except FileNotFoundError:
            if not self.gps_optional_enabled:
                raise
            self.gps_available = False
            self.keyframes_by_tab["gps"] = _empty_gps_keyframe_sets()
        except ValueError as exc:
            if not self.gps_optional_enabled or "No valid GPS/TF rows were loaded from" not in str(exc):
                raise
            self.gps_available = False
            self.keyframes_by_tab["gps"] = _empty_gps_keyframe_sets()
        self._initialize_camera_preview(scan_dir, metadata, effective_timing, restore_preview=restore_preview)
        self._configure_camera_preview_controls()
        self._has_initialized_camera = False

        calibration_note = ""
        if self.camera_preview_available and self.camera_preview_calibration is not None:
            calibration_note = (
                f" Preview calibration = {self.camera_preview_calibration.calibration_source} "
                f"({self.camera_preview_calibration.calibration_run.name})."
            )
        elif self.camera_preview_error:
            calibration_note = " Interactive preview unavailable."
        self.summary_label.setText(
            f"{scan_dir.name}: rendering {point_cloud_path.name} using {mode} shading. "
            f"Camera offset = {effective_timing.get('fusion_time_offset_sec', 0.0):.6f} s, "
            f"GPS offset = {effective_timing.get('gps_time_offset_sec', 0.0):.6f} s."
            f"{calibration_note}"
        )
        self._update_tab_info()
        self._render_active_tab()

    def _initialize_camera_preview(
        self,
        scan_dir: Path,
        metadata: dict,
        effective_timing: dict,
        *,
        restore_preview: dict[str, Any] | None = None,
    ) -> None:
        restore_preview = dict(restore_preview or {})
        self.camera_preview_frames = []
        self.camera_preview_calibration = None
        self.camera_preview_available = False
        self.camera_preview_error = ""
        self.camera_preview_visible_indices = np.empty(0, dtype=np.int64)
        self.camera_preview_visible_rgb = np.empty((0, 3), dtype=np.float32)
        self.camera_preview_effective_time = None
        self.camera_preview_selected_pose = None
        self.camera_preview_selected_camera_position = None
        self.camera_preview_selected_camera_direction = None
        self.camera_preview_visible_count = 0
        self.camera_preview_last_drop_reason = ""
        self.camera_preview_apply_distortion = bool(restore_preview.get("apply_distortion", False))
        self.camera_preview_frontmost_only = bool(restore_preview.get("frontmost_only", True))
        self.camera_preview_diagnostics = PreviewProjectionDiagnostics()
        self.camera_preview_cached_image_name = ""
        self.camera_preview_cached_image_rgb = None
        self.camera_preview_display_pixmap = None
        self.camera_preview_state = CameraPreviewState(
            preview_offset_sec=float(effective_timing.get("fusion_time_offset_sec", 0.0)),
            overlay_opacity=float(restore_preview.get("overlay_opacity", DEFAULT_OVERLAY_OPACITY)),
            adjustment_mode=str(restore_preview.get("adjustment_mode", "coarse")),
        )
        if self.camera_preview_state.adjustment_mode not in {"coarse", "fine"}:
            self.camera_preview_state.adjustment_mode = "coarse"

        try:
            files = metadata.get("files", {})
            tf_camera_csv = resolve_scan_path(scan_dir, files.get("tf_camera_csv"))
            if tf_camera_csv is None or not tf_camera_csv.is_file():
                raise FileNotFoundError("Camera TF CSV not found for this scan.")
            camera_poses = load_pose_records(tf_camera_csv, "t_query_sec")
            self.camera_preview_frames = _build_camera_preview_frames(scan_dir, metadata, camera_poses)
            self.camera_preview_calibration = _load_preview_calibration(scan_dir, metadata)
            ensure_built()
            preferred_frame = str(restore_preview.get("selected_frame_name", "")).strip()
            if preferred_frame:
                for index, frame in enumerate(self.camera_preview_frames):
                    if frame.image_name == preferred_frame:
                        self.camera_preview_state.selected_frame_index = index
                        break
            self._set_preview_frame_index(self.camera_preview_state.selected_frame_index)
            self.camera_preview_available = True
        except Exception as exc:
            self.camera_preview_error = str(exc)
            self.camera_preview_state.selected_frame_index = 0
            self.camera_preview_state.selected_frame_name = ""
            self.camera_preview_state.selected_frame_timestamp = 0.0

    def _configure_camera_preview_controls(self) -> None:
        frame_combo = self.camera_tab.frame_combo  # type: ignore[attr-defined]
        prev_frame_btn = self.camera_tab.prev_frame_btn  # type: ignore[attr-defined]
        next_frame_btn = self.camera_tab.next_frame_btn  # type: ignore[attr-defined]
        offset_slider = self.camera_tab.offset_slider  # type: ignore[attr-defined]
        coarse_btn = self.camera_tab.coarse_btn  # type: ignore[attr-defined]
        fine_btn = self.camera_tab.fine_btn  # type: ignore[attr-defined]
        opacity_slider = self.camera_tab.opacity_slider  # type: ignore[attr-defined]
        distortion_checkbox = self.camera_tab.distortion_checkbox  # type: ignore[attr-defined]
        frontmost_checkbox = self.camera_tab.frontmost_checkbox  # type: ignore[attr-defined]
        save_btn = self.camera_tab.save_btn  # type: ignore[attr-defined]

        frame_combo_blocker = QSignalBlocker(frame_combo)
        try:
            frame_combo.clear()
            for index, frame in enumerate(self.camera_preview_frames):
                if frame.elapsed_sec is None:
                    label = f"{index + 1:03d} | {frame.image_name}"
                else:
                    label = f"{index + 1:03d} | {frame.image_name} | {frame.elapsed_sec:.3f} s"
                frame_combo.addItem(label)
            if self.camera_preview_frames:
                frame_combo.setCurrentIndex(self.camera_preview_state.selected_frame_index)
        finally:
            del frame_combo_blocker

        available = self.camera_preview_available and bool(self.camera_preview_frames)
        frame_combo.setEnabled(available)
        prev_frame_btn.setEnabled(available and self.camera_preview_state.selected_frame_index > 0)
        next_frame_btn.setEnabled(available and self.camera_preview_state.selected_frame_index < len(self.camera_preview_frames) - 1)
        offset_slider.setEnabled(available)
        coarse_btn.setEnabled(available)
        fine_btn.setEnabled(available)
        opacity_slider.setEnabled(available)
        distortion_checkbox.setEnabled(available)
        frontmost_checkbox.setEnabled(available)
        save_btn.setEnabled(available)
        offset_slider.setSingleStep(1)
        offset_slider.setPageStep(25 if self.camera_preview_state.adjustment_mode == "fine" else 250)

        self._suspend_preview_updates = True
        try:
            coarse_blocker = QSignalBlocker(coarse_btn)
            fine_blocker = QSignalBlocker(fine_btn)
            try:
                if self.camera_preview_state.adjustment_mode == "fine":
                    fine_btn.setChecked(True)
                else:
                    coarse_btn.setChecked(True)
                    self.camera_preview_state.adjustment_mode = "coarse"
            finally:
                del coarse_blocker
                del fine_blocker

            offset_blocker = QSignalBlocker(offset_slider)
            try:
                offset_slider.setValue(self._offset_slider_value(self.camera_preview_state.preview_offset_sec))
            finally:
                del offset_blocker

            opacity_blocker = QSignalBlocker(opacity_slider)
            distortion_blocker = QSignalBlocker(distortion_checkbox)
            frontmost_blocker = QSignalBlocker(frontmost_checkbox)
            try:
                opacity_slider.setValue(int(round(self.camera_preview_state.overlay_opacity * 100.0)))
                distortion_checkbox.setChecked(self.camera_preview_apply_distortion)
                frontmost_checkbox.setChecked(self.camera_preview_frontmost_only)
            finally:
                del opacity_blocker
                del distortion_blocker
                del frontmost_blocker
        finally:
            self._suspend_preview_updates = False

        self._update_offset_readout()
        self._update_opacity_readout()
        self._update_projection_diagnostics_label()
        self._update_selected_image_display()

    def _capture_preview_restore_state(self) -> dict[str, Any]:
        return {
            "selected_frame_name": self.camera_preview_state.selected_frame_name,
            "adjustment_mode": self.camera_preview_state.adjustment_mode,
            "overlay_opacity": self.camera_preview_state.overlay_opacity,
            "apply_distortion": self.camera_preview_apply_distortion,
            "frontmost_only": self.camera_preview_frontmost_only,
        }

    def _preview_step_sec(self) -> float:
        return FINE_OFFSET_STEP_SEC if self.camera_preview_state.adjustment_mode == "fine" else COARSE_OFFSET_STEP_SEC

    def _offset_slider_value(self, offset_sec: float) -> int:
        value = int(round(float(offset_sec) / self._preview_step_sec()))
        return max(-PREVIEW_SLIDER_TICKS, min(PREVIEW_SLIDER_TICKS, value))

    def _set_preview_frame_index(self, index: int) -> None:
        if not self.camera_preview_frames:
            self.camera_preview_state.selected_frame_index = 0
            self.camera_preview_state.selected_frame_name = ""
            self.camera_preview_state.selected_frame_timestamp = 0.0
            return
        clamped = max(0, min(int(index), len(self.camera_preview_frames) - 1))
        frame = self.camera_preview_frames[clamped]
        self.camera_preview_state.selected_frame_index = clamped
        self.camera_preview_state.selected_frame_name = frame.image_name
        self.camera_preview_state.selected_frame_timestamp = float(frame.timestamp)

    def _set_preview_frame_selection(self, index: int) -> None:
        if not self.camera_preview_frames:
            return
        clamped = max(0, min(int(index), len(self.camera_preview_frames) - 1))
        frame_combo = self.camera_tab.frame_combo  # type: ignore[attr-defined]
        blocker = QSignalBlocker(frame_combo)
        try:
            frame_combo.setCurrentIndex(clamped)
        finally:
            del blocker
        self._set_preview_frame_index(clamped)
        self._update_preview_navigation_buttons()
        self._refresh_camera_preview()

    def _update_preview_navigation_buttons(self) -> None:
        prev_frame_btn = self.camera_tab.prev_frame_btn  # type: ignore[attr-defined]
        next_frame_btn = self.camera_tab.next_frame_btn  # type: ignore[attr-defined]
        available = self.camera_preview_available and bool(self.camera_preview_frames)
        prev_frame_btn.setEnabled(available and self.camera_preview_state.selected_frame_index > 0)
        next_frame_btn.setEnabled(available and self.camera_preview_state.selected_frame_index < len(self.camera_preview_frames) - 1)

    def _current_preview_frame(self) -> CameraPreviewFrame | None:
        if not self.camera_preview_frames:
            return None
        index = self.camera_preview_state.selected_frame_index
        if not 0 <= index < len(self.camera_preview_frames):
            return None
        return self.camera_preview_frames[index]

    def _update_offset_readout(self) -> None:
        step_ms = int(round(self._preview_step_sec() * 1000.0))
        self.camera_tab.offset_value_label.setText(  # type: ignore[attr-defined]
            f"{self.camera_preview_state.preview_offset_sec:+.6f} s ({self.camera_preview_state.adjustment_mode.title()} {step_ms} ms)"
        )

    def _update_opacity_readout(self) -> None:
        opacity_percent = int(round(self.camera_preview_state.overlay_opacity * 100.0))
        self.camera_tab.opacity_value_label.setText(f"{opacity_percent}%")  # type: ignore[attr-defined]

    def _update_projection_diagnostics_label(self) -> None:
        label = self.camera_tab.diagnostics_label  # type: ignore[attr-defined]
        if not self.camera_preview_available:
            label.setText("Projection diagnostics unavailable.")
            return

        mode_text = (
            f"Mode: {'distortion on' if self.camera_preview_apply_distortion else 'rectified / no distortion'}, "
            f"{'front-most only' if self.camera_preview_frontmost_only else 'all in-image points'}."
        )
        stats = self.camera_preview_diagnostics
        if stats.in_image_points <= 0:
            label.setText(f"{mode_text} No in-image hits for the current frame.")
            return

        label.setText(
            f"{mode_text}\n"
            f"Front of camera: {stats.front_of_camera_points:,} | In-image: {stats.in_image_points:,} | "
            f"Kept: {stats.kept_points:,}\n"
            f"Unique pixels: {stats.unique_hit_pixels:,} | Multi-hit pixels: {stats.multi_hit_pixels:,} | "
            f"Avg hits/pixel: {stats.avg_points_per_hit_pixel:.3f} | Max hits/pixel: {stats.max_points_per_hit_pixel}\n"
            f"Border hits: {stats.border_hits_before_filter:,} -> {stats.border_hits_after_filter:,}"
        )

    def _on_preview_frame_changed(self, index: int) -> None:
        if self._suspend_preview_updates:
            return
        self._set_preview_frame_index(index)
        self._update_preview_navigation_buttons()
        self._refresh_camera_preview()

    def _step_preview_frame(self, delta: int) -> None:
        if not self.camera_preview_frames:
            return
        self._set_preview_frame_selection(self.camera_preview_state.selected_frame_index + int(delta))

    def _on_preview_offset_changed(self, value: int) -> None:
        if self._suspend_preview_updates:
            return
        self.camera_preview_state.preview_offset_sec = float(value) * self._preview_step_sec()
        self._update_offset_readout()
        self._refresh_camera_preview()

    def _on_preview_mode_changed(self, mode: str, checked: bool) -> None:
        if not checked or self._suspend_preview_updates:
            return
        previous_offset = float(self.camera_preview_state.preview_offset_sec)
        self.camera_preview_state.adjustment_mode = mode

        slider = self.camera_tab.offset_slider  # type: ignore[attr-defined]
        slider.setSingleStep(1)
        slider.setPageStep(25 if mode == "fine" else 250)

        self._suspend_preview_updates = True
        try:
            slider_blocker = QSignalBlocker(slider)
            try:
                slider.setValue(self._offset_slider_value(previous_offset))
            finally:
                del slider_blocker
            self.camera_preview_state.preview_offset_sec = float(slider.value()) * self._preview_step_sec()
        finally:
            self._suspend_preview_updates = False

        self._update_offset_readout()
        self._refresh_camera_preview()

    def _on_overlay_opacity_changed(self, value: int) -> None:
        if self._suspend_preview_updates:
            return
        self.camera_preview_state.overlay_opacity = max(0.0, min(1.0, float(value) / 100.0))
        self._update_opacity_readout()
        self._apply_preview_colors(render=True)
        self._update_status_label()

    def _on_preview_projection_options_changed(self) -> None:
        if self._suspend_preview_updates:
            return
        self.camera_preview_apply_distortion = bool(self.camera_tab.distortion_checkbox.isChecked())  # type: ignore[attr-defined]
        self.camera_preview_frontmost_only = bool(self.camera_tab.frontmost_checkbox.isChecked())  # type: ignore[attr-defined]
        self._refresh_camera_preview()

    def _load_selected_preview_image(self) -> np.ndarray:
        frame = self._current_preview_frame()
        if frame is None:
            raise ValueError("No camera preview frame is selected.")
        if self.camera_preview_cached_image_name == frame.image_name and self.camera_preview_cached_image_rgb is not None:
            return self.camera_preview_cached_image_rgb

        with Image.open(frame.image_path) as image:
            rgb = np.asarray(image.convert("RGB"), dtype=np.uint8)

        calibration = self.camera_preview_calibration
        if calibration is not None:
            image_height, image_width = rgb.shape[:2]
            if image_width != calibration.intrinsics.width or image_height != calibration.intrinsics.height:
                raise ValueError(
                    f"{frame.image_name} size {image_width}x{image_height} does not match calibration "
                    f"{calibration.intrinsics.width}x{calibration.intrinsics.height}."
                )

        self.camera_preview_cached_image_name = frame.image_name
        self.camera_preview_cached_image_rgb = rgb
        return rgb

    def _update_selected_image_display(self) -> None:
        preview_label = self.image_preview_label
        frame = self._current_preview_frame()
        if frame is None or not self.camera_preview_available:
            self.camera_preview_display_pixmap = None
            preview_label.setPixmap(QPixmap())
            preview_label.setText("Selected image preview unavailable.")
            self.image_caption_label.setText("")
            return

        try:
            rgb = self._load_selected_preview_image()
        except Exception as exc:
            self.camera_preview_display_pixmap = None
            preview_label.setPixmap(QPixmap())
            preview_label.setText(str(exc))
            self.image_caption_label.setText("")
            return

        rgb_contiguous = np.ascontiguousarray(rgb)
        height, width = rgb_contiguous.shape[:2]
        bytes_per_line = int(rgb_contiguous.strides[0])
        image = QImage(rgb_contiguous.data, width, height, bytes_per_line, QImage.Format_RGB888).copy()
        self.camera_preview_display_pixmap = QPixmap.fromImage(image)
        scaled = self.camera_preview_display_pixmap.scaled(
            preview_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        preview_label.setText("")
        preview_label.setPixmap(scaled)
        elapsed_text = ""
        if frame.elapsed_sec is not None:
            elapsed_text = f" | {frame.elapsed_sec:.3f} s"
        self.image_caption_label.setText(
            f"{frame.image_name}{elapsed_text}\nSelected keyframe {self.camera_preview_state.selected_frame_index + 1} of {len(self.camera_preview_frames)}"
        )

    def _refresh_camera_preview(self) -> None:
        frame = self._current_preview_frame()
        self.camera_preview_visible_indices = np.empty(0, dtype=np.int64)
        self.camera_preview_visible_rgb = np.empty((0, 3), dtype=np.float32)
        self.camera_preview_diagnostics = PreviewProjectionDiagnostics()
        self.camera_preview_effective_time = None
        self.camera_preview_selected_pose = None
        self.camera_preview_selected_camera_position = None
        self.camera_preview_selected_camera_direction = None
        self.camera_preview_visible_count = 0
        self.camera_preview_last_drop_reason = ""

        if not self.camera_preview_available or frame is None or self.dense_trajectory is None or self.camera_preview_calibration is None:
            self._update_projection_diagnostics_label()
            self._apply_preview_colors(render=False)
            self._update_camera_preview_markers()
            self._update_status_label()
            if self.tab_widget.currentIndex() == 0:
                self.plotter.render()
            return

        self.camera_preview_effective_time = float(frame.timestamp) + float(self.camera_preview_state.preview_offset_sec)
        image_rgb: np.ndarray | None = None

        try:
            image_rgb = self._load_selected_preview_image()
            pose = query_dense_trajectory(self.dense_trajectory, self.camera_preview_effective_time)
            if pose is None:
                self.camera_preview_last_drop_reason = "effective_time_out_of_range"
            else:
                self.camera_preview_selected_pose = pose
                lidar_to_map = _transform_from_pose(pose)
                map_to_lidar = _invert_transform(lidar_to_map)
                camera_to_map = lidar_to_map @ self.camera_preview_calibration.camera_to_lidar
                preview_direction = camera_to_map[:3, :3] @ np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
                preview_direction = _safe_normalize(preview_direction.reshape(1, 3))[0]
                self.camera_preview_selected_camera_position = camera_to_map[:3, 3].astype(np.float64)
                self.camera_preview_selected_camera_direction = preview_direction.astype(np.float64)

                intrinsics = self.camera_preview_calibration.intrinsics
                distortion = intrinsics.distortion if self.camera_preview_apply_distortion else np.empty(0, dtype=np.float64)
                u, v, visible = project_points_to_image(
                    points_map=self.point_cloud_points64,
                    map_to_lidar=map_to_lidar,
                    lidar_to_cam=self.camera_preview_calibration.lidar_to_camera,
                    fx=intrinsics.fx,
                    fy=intrinsics.fy,
                    cx=intrinsics.cx,
                    cy=intrinsics.cy,
                    width=intrinsics.width,
                    height=intrinsics.height,
                    distortion=distortion,
                )
                points_lidar = _transform_points(self.point_cloud_points64, map_to_lidar)
                points_camera = _transform_points(points_lidar, self.camera_preview_calibration.lidar_to_camera)
                visible_indices = np.flatnonzero(visible)
                if visible_indices.size == 0:
                    self.camera_preview_diagnostics = PreviewProjectionDiagnostics(
                        front_of_camera_points=int(np.count_nonzero(points_camera[:, 2] > 1e-9))
                    )
                    self.camera_preview_last_drop_reason = "no_projected_points"
                else:
                    in_image_count = int(visible_indices.size)
                    projected_u = u[visible_indices].astype(np.int32, copy=False)
                    projected_v = v[visible_indices].astype(np.int32, copy=False)
                    pixel_ids = projected_v.astype(np.int64) * intrinsics.width + projected_u.astype(np.int64)
                    unique_pixels, inverse, counts = np.unique(pixel_ids, return_inverse=True, return_counts=True)
                    border_hits = _border_pixel_mask(projected_u, projected_v, intrinsics.width, intrinsics.height)
                    border_hits_before_filter = int(np.count_nonzero(border_hits))
                    keep_mask = np.ones(visible_indices.shape[0], dtype=bool)
                    if self.camera_preview_frontmost_only:
                        depths = points_camera[visible_indices, 2]
                        frontmost_depth = np.full(unique_pixels.shape[0], np.inf, dtype=np.float64)
                        np.minimum.at(frontmost_depth, inverse, depths)
                        keep_mask = depths <= (frontmost_depth[inverse] + 0.02)
                        visible_indices = visible_indices[keep_mask]
                        projected_u = projected_u[keep_mask]
                        projected_v = projected_v[keep_mask]
                        border_hits = border_hits[keep_mask]

                    self.camera_preview_diagnostics = PreviewProjectionDiagnostics(
                        front_of_camera_points=int(np.count_nonzero(points_camera[:, 2] > 1e-9)),
                        in_image_points=in_image_count,
                        kept_points=int(visible_indices.size),
                        unique_hit_pixels=int(unique_pixels.size),
                        multi_hit_pixels=int(np.count_nonzero(counts > 1)),
                        avg_points_per_hit_pixel=float(in_image_count / unique_pixels.size),
                        max_points_per_hit_pixel=int(np.max(counts)),
                        border_hits_before_filter=border_hits_before_filter,
                        border_hits_after_filter=int(np.count_nonzero(border_hits)),
                    )

                    self.camera_preview_visible_indices = visible_indices.astype(np.int64, copy=False)
                    self.camera_preview_visible_rgb = image_rgb[projected_v, projected_u].astype(np.float32) / 255.0
                    self.camera_preview_visible_count = int(visible_indices.size)
        except Exception as exc:
            self.camera_preview_last_drop_reason = str(exc)

        self._update_selected_image_display()
        self._update_projection_diagnostics_label()
        self._apply_preview_colors(render=False)
        self._update_camera_preview_markers()
        self._update_status_label()
        if self.tab_widget.currentIndex() == 0:
            self.plotter.render()

    def _apply_preview_colors(self, *, render: bool) -> None:
        if self.point_cloud_mesh is None:
            return
        blended = self.point_cloud_colors.copy()
        if self.camera_preview_visible_indices.size and self.camera_preview_state.overlay_opacity > 0.0:
            alpha = float(self.camera_preview_state.overlay_opacity)
            indices = self.camera_preview_visible_indices
            blended[indices] = ((1.0 - alpha) * blended[indices] + alpha * self.camera_preview_visible_rgb).astype(np.float32)
        self.point_cloud_mesh["rgb"] = (255.0 * np.clip(blended, 0.0, 1.0)).astype(np.uint8)
        self._mark_mesh_modified(self.point_cloud_mesh)
        if render and self.tab_widget.currentIndex() == 0:
            self.plotter.render()

    @staticmethod
    def _mark_mesh_modified(mesh: pv.PolyData) -> None:
        modified = getattr(mesh, "modified", None)
        if callable(modified):
            modified()
            return
        vtk_modified = getattr(mesh, "Modified", None)
        if callable(vtk_modified):
            vtk_modified()

    def _safe_remove_actor(self, actor_name: str) -> None:
        try:
            self.plotter.remove_actor(actor_name, reset_camera=False, render=False)
        except Exception:
            pass

    def _update_camera_preview_markers(self) -> None:
        for actor_name in (
            "selected_original_point",
            "selected_original_arrow",
            "selected_preview_point",
            "selected_preview_arrow",
            "selected_preview_link",
        ):
            self._safe_remove_actor(actor_name)

        if self.tab_widget.currentIndex() != 0:
            return

        frame = self._current_preview_frame()
        if frame is None:
            return

        arrow_length = self._arrow_length(np.asarray([frame.original_pose.translation], dtype=np.float64))

        original_cloud = pv.PolyData(np.asarray([frame.original_pose.translation], dtype=np.float64))
        original_direction = Rotation.from_quat(frame.original_pose.quaternion_xyzw).apply(np.asarray([[1.0, 0.0, 0.0]], dtype=np.float64))
        original_cloud["glyph_vec"] = _safe_normalize(original_direction)
        original_arrow = original_cloud.glyph(
            orient="glyph_vec",
            factor=arrow_length,
            geom=pv.Arrow(tip_length=0.28, tip_radius=0.08, shaft_radius=0.025),
        )
        self.plotter.add_mesh(
            original_cloud,
            color="#f8fafc",
            point_size=18,
            render_points_as_spheres=True,
            name="selected_original_point",
        )
        self.plotter.add_mesh(original_arrow, color="#f8fafc", opacity=0.95, name="selected_original_arrow")

        if self.camera_preview_selected_camera_position is None or self.camera_preview_selected_camera_direction is None:
            return

        preview_cloud = pv.PolyData(self.camera_preview_selected_camera_position.reshape(1, 3))
        preview_cloud["glyph_vec"] = self.camera_preview_selected_camera_direction.reshape(1, 3)
        preview_arrow = preview_cloud.glyph(
            orient="glyph_vec",
            factor=arrow_length,
            geom=pv.Arrow(tip_length=0.28, tip_radius=0.08, shaft_radius=0.025),
        )
        self.plotter.add_mesh(
            preview_cloud,
            color="#fde047",
            point_size=18,
            render_points_as_spheres=True,
            name="selected_preview_point",
        )
        self.plotter.add_mesh(preview_arrow, color="#fde047", opacity=0.95, name="selected_preview_arrow")

        if np.linalg.norm(self.camera_preview_selected_camera_position - frame.original_pose.translation) > 1e-6:
            link = pv.Line(frame.original_pose.translation, self.camera_preview_selected_camera_position)
            self.plotter.add_mesh(link, color="#38bdf8", line_width=4, opacity=0.95, name="selected_preview_link")

    def _camera_preview_status_text(self) -> str:
        if not self.camera_preview_available:
            return f"Interactive preview unavailable: {self.camera_preview_error}"

        frame = self._current_preview_frame()
        if frame is None:
            return "Interactive preview ready."

        calibration = self.camera_preview_calibration
        status = (
            f"Selected frame {frame.image_name} at offset {self.camera_preview_state.preview_offset_sec:+.6f} s. "
            f"Opacity {self.camera_preview_state.overlay_opacity:.0%}. "
            f"{'Distortion on' if self.camera_preview_apply_distortion else 'Rectified / no distortion'}. "
            f"{'Front-most only' if self.camera_preview_frontmost_only else 'All in-image points'}."
        )
        if calibration is not None:
            status += (
                f" Calibration source: {calibration.calibration_source} "
                f"({calibration.calibration_run.name})."
            )
        if self.camera_preview_selected_pose is not None:
            status += (
                f" Visible projected points: {self.camera_preview_visible_count}. "
                f"Unique hit pixels: {self.camera_preview_diagnostics.unique_hit_pixels}."
            )
        elif self.camera_preview_last_drop_reason:
            status += f" Preview status: {self.camera_preview_last_drop_reason}."
        return status

    def _update_status_label(self) -> None:
        if not self.keyframes_by_tab:
            return
        tab_key = "camera" if self.tab_widget.currentIndex() == 0 else "gps"
        mode = self.current_mode[tab_key]
        keyframe_set = self.keyframes_by_tab[tab_key][mode]
        status = (
            f"{keyframe_set.label}: {keyframe_set.positions.shape[0]} poses. "
            f"Dense trajectory samples: {self.trajectory_points.shape[0]}."
        )
        if tab_key == "camera":
            status += " " + self._camera_preview_status_text()
        elif not self.gps_available and self.gps_optional_enabled:
            status += f" GPS data is unavailable for this scan; opening the viewer is allowed by {self.gps_mode_source} developer mode."
        self.status_label.setText(status)

    def _save_time_offset(self) -> None:
        if self.scan_dir is None:
            return
        restore_preview = self._capture_preview_restore_state()
        camera_state = self._capture_camera_state()
        saved_offset_sec = float(self.camera_preview_state.preview_offset_sec)
        scan_dir, metadata = load_scan_metadata(self.scan_dir)
        timing_overrides = metadata.setdefault("config", {}).setdefault("timing_overrides", {})
        timing_overrides["fusion_time_offset_sec"] = saved_offset_sec
        metadata.setdefault("config", {}).setdefault("fusion", {})["time_offset_sec"] = saved_offset_sec
        save_scan_metadata(scan_dir, metadata)

        global_settings = load_global_timing_settings()
        global_settings.setdefault("timing", {})["fusion_time_offset_sec"] = saved_offset_sec
        save_global_timing_settings(global_settings)

        self.load_scan(scan_dir, restore_preview=restore_preview)
        if camera_state is not None:
            self._apply_camera_state(camera_state)
        QMessageBox.information(self, "Timing / Offset Viewer", "time offset saved")

    def _set_mode(self, tab_key: str, mode: str, checked: bool) -> None:
        if not checked:
            return
        self.current_mode[tab_key] = mode
        active_key = "camera" if self.tab_widget.currentIndex() == 0 else "gps"
        if active_key == tab_key:
            self._render_active_tab()

    def _update_tab_info(self) -> None:
        if not self.scan_dir:
            return
        camera_text = (
            "Camera tab uses the effective camera/Fusion offset and interpolated pose lookup when needed. "
            "Orientation glyphs point along the camera forward axis. "
            "Interactive preview projects the selected rectified camera image into the map using the same calibration artifacts, "
            "LiDAR-to-camera extrinsics, and dense-trajectory pose lookup as Fusion. "
            "The preview now defaults to rectified/no-distortion sampling plus a nearest-point-per-pixel filter, "
            "and the debug toggles can reproduce the legacy behavior for comparison."
        )
        if self.camera_preview_available and self.camera_preview_calibration is not None:
            camera_text += (
                f" Calibration source: {self.camera_preview_calibration.calibration_source} "
                f"({self.camera_preview_calibration.calibration_run.name})."
            )
        elif self.camera_preview_error:
            camera_text += f" Interactive preview unavailable: {self.camera_preview_error}"
        gps_text = (
            "GPS tab uses the independent effective GPS georeference offset. "
            "Shifted GPS positions come from dense-trajectory translation compensation while headings stay lightweight."
        )
        if not self.gps_available and self.gps_optional_enabled:
            gps_text += f" GPS data is missing for this scan, but viewer access is allowed by {self.gps_mode_source} developer mode."
        self.camera_tab.info_label.setText(camera_text)  # type: ignore[attr-defined]
        self.gps_tab.info_label.setText(gps_text)  # type: ignore[attr-defined]

    def _render_active_tab(self) -> None:
        if not self.keyframes_by_tab:
            return
        tab_key = "camera" if self.tab_widget.currentIndex() == 0 else "gps"
        mode = self.current_mode[tab_key]
        keyframe_set = self.keyframes_by_tab[tab_key][mode]
        previous_camera = self._capture_camera_state()

        self.plotter.clear()

        cloud = pv.PolyData(self.point_cloud_points)
        cloud["rgb"] = self.point_cloud_rgb.copy()
        self.point_cloud_mesh = cloud
        self.plotter.add_mesh(
            cloud,
            scalars="rgb",
            rgb=True,
            point_size=2,
            render_points_as_spheres=True,
            opacity=0.95,
            name="point_cloud",
        )

        if self.trajectory_points.shape[0] >= 2:
            trajectory_line = pv.lines_from_points(self.trajectory_points, close=False)
            self.plotter.add_mesh(trajectory_line, color="#f8fafc", line_width=2, opacity=0.85, name="trajectory")

        if keyframe_set.positions.shape[0] > 0:
            keyframe_cloud = pv.PolyData(keyframe_set.positions)
            keyframe_cloud["glyph_vec"] = keyframe_set.directions
            arrow_length = self._arrow_length(keyframe_set.positions)
            arrows = keyframe_cloud.glyph(orient="glyph_vec", factor=arrow_length, geom=pv.Arrow(tip_length=0.28, tip_radius=0.08, shaft_radius=0.025))
            self.plotter.add_mesh(keyframe_cloud, color=keyframe_set.color, point_size=9, render_points_as_spheres=True, name="keyframe_points")
            self.plotter.add_mesh(arrows, color=keyframe_set.color, opacity=0.95, name="keyframe_arrows")

        self.plotter.show_axes()
        if tab_key == "camera":
            self._refresh_camera_preview()
        else:
            self._update_status_label()
        if self._has_initialized_camera and previous_camera is not None:
            self._apply_camera_state(previous_camera)
        else:
            self._reset_camera()
            self._has_initialized_camera = True

        scene_scale = self._scene_diagonal()
        self.plotter.minimum_dolly_step = max(0.02, scene_scale * 0.0008)
        self.plotter.minimum_focus_distance = 1e-6

    def _arrow_length(self, positions: np.ndarray) -> float:
        if self.trajectory_points.shape[0] >= 2:
            span = np.linalg.norm(np.max(self.trajectory_points, axis=0) - np.min(self.trajectory_points, axis=0))
        elif positions.shape[0] >= 2:
            span = np.linalg.norm(np.max(positions, axis=0) - np.min(positions, axis=0))
        else:
            span = 1.0
        return max(0.2, float(span) * 0.015)

    def _reset_camera(self) -> None:
        try:
            self.plotter.reset_camera()
            self._set_permissive_clipping_range()
            self.plotter.render()
        except Exception:
            pass

    def _capture_camera_state(self):
        try:
            camera = self.plotter.camera
            return {
                "position": tuple(camera.position),
                "focal_point": tuple(camera.focal_point),
                "view_up": tuple(camera.up),
                "parallel_scale": float(getattr(camera, "parallel_scale", 1.0)),
                "parallel_projection": bool(getattr(camera, "parallel_projection", False)),
            }
        except Exception:
            return None

    def _apply_camera_state(self, state) -> None:
        try:
            camera = self.plotter.camera
            camera.position = state["position"]
            camera.focal_point = state["focal_point"]
            camera.up = state["view_up"]
            if state.get("parallel_projection", False):
                camera.enable_parallel_projection()
                camera.parallel_scale = state["parallel_scale"]
            else:
                camera.disable_parallel_projection()
            self._set_permissive_clipping_range()
            self.plotter.render()
        except Exception:
            self._reset_camera()

    def _set_permissive_clipping_range(self) -> None:
        try:
            diagonal = self._scene_diagonal()
            camera = self.plotter.camera
            position = np.asarray(camera.position, dtype=np.float64)
            focal_point = np.asarray(camera.focal_point, dtype=np.float64)
            distance = float(np.linalg.norm(focal_point - position))
            near = max(1e-9, min(diagonal * 1e-8, max(distance * 1e-5, 1e-9)))
            far = max(100.0, diagonal * 8.0, distance * 500.0)
            self.plotter.camera.clipping_range = (near, far)
        except Exception:
            pass

    def _scene_diagonal(self) -> float:
        all_points = []
        if self.point_cloud_points.size:
            all_points.append(np.asarray(self.point_cloud_points, dtype=np.float64))
        if self.trajectory_points.size:
            all_points.append(np.asarray(self.trajectory_points, dtype=np.float64))
        for tab_sets in self.keyframes_by_tab.values():
            for keyframe_set in tab_sets.values():
                if keyframe_set.positions.size:
                    all_points.append(np.asarray(keyframe_set.positions, dtype=np.float64))
        if not all_points:
            return 1.0
        stacked = np.vstack(all_points)
        return max(1.0, float(np.linalg.norm(np.max(stacked, axis=0) - np.min(stacked, axis=0))))

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 - Qt override
        event.ignore()
        self.hide()

    def resizeEvent(self, event):  # noqa: N802 - Qt override
        super().resizeEvent(event)
        if self.camera_preview_display_pixmap is None:
            return
        preview_label = self.image_preview_label
        scaled = self.camera_preview_display_pixmap.scaled(
            preview_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        preview_label.setPixmap(scaled)


def open_timing_debug_viewer(
    parent: QWidget,
    scan_ref: str | Path,
    existing_dialog: TimingDebugViewerDialog | None = None,
) -> TimingDebugViewerDialog | None:
    dialog = existing_dialog if isinstance(existing_dialog, TimingDebugViewerDialog) else TimingDebugViewerDialog(parent=parent)
    try:
        dialog.load_scan(scan_ref)
    except Exception as exc:
        QMessageBox.warning(parent, "Timing Viewer", str(exc))
        if existing_dialog is None:
            dialog.deleteLater()
        return None
    dialog.showNormal()
    dialog.raise_()
    dialog.activateWindow()
    return dialog
