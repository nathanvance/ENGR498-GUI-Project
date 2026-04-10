from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import laspy
import numpy as np
import open3d as o3d
import pyvista as pv
from pyvistaqt import QtInteractor
from PySide6.QtCore import Qt
from PySide6.QtGui import QCloseEvent, QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QButtonGroup,
    QDialog,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from scipy.spatial.transform import Rotation

from scan_metadata import first_existing_artifact, load_scan_metadata, resolve_scan_path
from theme import THEME, button_style, subtle_button_style
from timing_settings import resolve_effective_timing


MAX_RENDER_POINTS = 250_000


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
        self.resize(1480, 920)
        self.setAttribute(Qt.WA_DeleteOnClose, False)

        self.scan_dir: Path | None = None
        self.metadata: dict = {}
        self.effective_timing: dict = {}
        self.point_cloud_points = np.empty((0, 3), dtype=np.float32)
        self.point_cloud_colors = np.empty((0, 3), dtype=np.float32)
        self.point_cloud_mode = "grayscale"
        self.trajectory_points = np.empty((0, 3), dtype=np.float64)
        self.keyframes_by_tab: dict[str, dict[str, KeyframeSet]] = {}
        self.current_mode = {"camera": "original", "gps": "original"}
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

        title_label = QLabel(title)
        title_label.setStyleSheet(f"font-size: 15px; font-weight: bold; color: {THEME['text']};")
        layout.addWidget(title_label)

        mode_group = QGroupBox("Keyframe Display")
        mode_layout = QVBoxLayout()
        mode_group.setLayout(mode_layout)
        mode_group.setStyleSheet(
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

    def load_scan(self, scan_ref: str | Path) -> None:
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
        self.point_cloud_points = points
        self.point_cloud_colors = colors
        self.point_cloud_mode = mode
        self.trajectory_points = trajectory.translations.astype(np.float64)
        self.keyframes_by_tab = {
            "camera": _build_camera_keyframes(scan_dir, metadata, effective_timing),
            "gps": _build_gps_keyframes(scan_dir, metadata, effective_timing),
        }
        self._has_initialized_camera = False

        self.summary_label.setText(
            f"{scan_dir.name}: rendering {point_cloud_path.name} using {mode} shading. "
            f"Camera offset = {effective_timing.get('fusion_time_offset_sec', 0.0):.6f} s, "
            f"GPS offset = {effective_timing.get('gps_time_offset_sec', 0.0):.6f} s."
        )
        self._update_tab_info()
        self._render_active_tab()

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
            "Orientation glyphs point along the camera forward axis."
        )
        gps_text = (
            "GPS tab uses the independent effective GPS georeference offset. "
            "Shifted GPS positions come from dense-trajectory translation compensation while headings stay lightweight."
        )
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
        cloud["rgb"] = (255.0 * np.clip(self.point_cloud_colors, 0.0, 1.0)).astype(np.uint8)
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
        self.status_label.setText(
            f"{keyframe_set.label}: {keyframe_set.positions.shape[0]} poses. "
            f"Dense trajectory samples: {self.trajectory_points.shape[0]}."
        )
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
