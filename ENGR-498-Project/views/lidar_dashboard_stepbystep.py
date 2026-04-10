"""
LiDAR Processing Dashboard (Step-by-Step Mode)
Advanced control center with detailed step control, checkboxes, and parameter modification.
"""

from pathlib import Path
from datetime import datetime

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QTableWidget, QTableWidgetItem, QHeaderView, QCheckBox,
    QFrame, QScrollArea, QFileDialog, QMessageBox,
    QToolButton, QDialog, QDoubleSpinBox, QSpinBox, QFormLayout,
    QDialogButtonBox, QTextEdit, QLineEdit, QComboBox
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QFont

from scan_metadata import (
    first_existing_artifact,
    load_scan_metadata,
    relativize_for_scan,
    resolve_scan_path,
    save_scan_metadata,
)
from timing_settings import load_global_timing_settings, save_global_timing_settings, resolve_effective_timing
from theme import THEME, button_style, notification_item_style, notification_panel_style, subtle_button_style


def compact_step_button_style(background: str, hover: str, *, text: str = "#ffffff") -> str:
    return f"""
        QPushButton {{
            background-color: {background};
            color: {text};
            border: 1px solid transparent;
            padding: 6px 8px;
            border-radius: {THEME['radius_sm']};
            font-weight: 700;
            font-size: 11px;
        }}
        QPushButton:hover:enabled {{
            background-color: {hover};
        }}
        QPushButton:disabled {{
            background-color: #475569;
            color: {THEME['muted']};
            border-color: {THEME['border']};
        }}
    """


class WireParametersDialog(QDialog):
    """Dialog for modifying wire extraction parameters"""
    
    def __init__(self, current_params=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Wire Extraction Parameters")
        self.setMinimumWidth(500)
        
        if current_params is None:
            current_params = {"R": 0.5, "angleThr": 10, "linearity": 0.98, "sag_method": "legacy"}
        
        self._setup_ui(current_params)
        
    def _setup_ui(self, current_params):
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title
        title = QLabel("Configure Wire Extraction Parameters")
        title.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {THEME['text']}; margin-bottom: 10px;")
        layout.addWidget(title)
        
        # Form layout
        form = QFormLayout()
        
        # R - Search Radius
        r_widget = QWidget()
        r_layout = QVBoxLayout()
        r_layout.setContentsMargins(0, 0, 0, 0)
        r_widget.setLayout(r_layout)
        
        self.r_spin = QDoubleSpinBox()
        self.r_spin.setRange(0.01, 10.0)
        self.r_spin.setSingleStep(0.1)
        self.r_spin.setValue(current_params.get("R", 0.5))
        self.r_spin.setSuffix(" m")
        r_layout.addWidget(self.r_spin)
        
        r_desc = QLabel(
            "<b>Search Radius (R)</b><br>"
            "How far to search for neighbors when computing normals.<br>"
            "• <b>Too small:</b> Misses neighbors, bad normal estimation<br>"
            "• <b>Too large:</b> Includes unrelated points, blurs features<br>"
            "• Should be <b>smaller than minimum distance between adjacent power lines</b>"
        )
        r_desc.setWordWrap(True)
        r_desc.setStyleSheet(f"color: {THEME['muted']}; font-size: 11px; background-color: {THEME['pane_raised']}; padding: 8px; border-radius: 4px;")
        r_layout.addWidget(r_desc)
        
        form.addRow("Search Radius (R):", r_widget)
        
        # angleThr - Angle Threshold
        angle_widget = QWidget()
        angle_layout = QVBoxLayout()
        angle_layout.setContentsMargins(0, 0, 0, 0)
        angle_widget.setLayout(angle_layout)
        
        self.angle_spin = QSpinBox()
        self.angle_spin.setRange(1, 45)
        self.angle_spin.setValue(current_params.get("angleThr", 10))
        self.angle_spin.setSuffix(" °")
        angle_layout.addWidget(self.angle_spin)
        
        angle_desc = QLabel(
            "<b>Angle Threshold</b><br>"
            "Angle threshold in degrees for detecting horizontal features.<br>"
            "• Powerlines are ~horizontal, so normals should be ~vertical (90° from vertical = horizontal)<br>"
            "• <b>Smaller value:</b> More strict (only very horizontal features)<br>"
            "• <b>Larger value:</b> More permissive"
        )
        angle_desc.setWordWrap(True)
        angle_desc.setStyleSheet(f"color: {THEME['muted']}; font-size: 11px; background-color: {THEME['pane_raised']}; padding: 8px; border-radius: 4px;")
        angle_layout.addWidget(angle_desc)
        
        form.addRow("Angle Threshold:", angle_widget)
        
        # linearity - Linearity Threshold
        linearity_widget = QWidget()
        linearity_layout = QVBoxLayout()
        linearity_layout.setContentsMargins(0, 0, 0, 0)
        linearity_widget.setLayout(linearity_layout)
        
        self.linearity_spin = QDoubleSpinBox()
        self.linearity_spin.setRange(0.5, 1.0)
        self.linearity_spin.setSingleStep(0.01)
        self.linearity_spin.setDecimals(3)
        self.linearity_spin.setValue(current_params.get("linearity", 0.98))
        linearity_layout.addWidget(self.linearity_spin)
        
        linearity_desc = QLabel(
            "<b>Linearity Threshold</b><br>"
            "How straight the features must be to be considered wires.<br>"
            "• <b>1.0:</b> Perfect line<br>"
            "• <b>0.98:</b> Very linear (recommended)<br>"
            "• <b>Lower values:</b> Accept more curved features"
        )
        linearity_desc.setWordWrap(True)
        linearity_desc.setStyleSheet(f"color: {THEME['muted']}; font-size: 11px; background-color: {THEME['pane_raised']}; padding: 8px; border-radius: 4px;")
        linearity_layout.addWidget(linearity_desc)
        
        form.addRow("Linearity Threshold:", linearity_widget)

        sag_method_widget = QWidget()
        sag_method_layout = QVBoxLayout()
        sag_method_layout.setContentsMargins(0, 0, 0, 0)
        sag_method_widget.setLayout(sag_method_layout)

        self.sag_method_combo = QComboBox()
        self.sag_method_combo.addItem("Legacy chord-based sag", "legacy")
        self.sag_method_combo.addItem("Fusion span catenary sag", "fusion_span")
        current_sag_method = str(current_params.get("sag_method", "legacy"))
        current_index = self.sag_method_combo.findData(current_sag_method)
        self.sag_method_combo.setCurrentIndex(current_index if current_index >= 0 else 0)
        sag_method_layout.addWidget(self.sag_method_combo)

        sag_method_desc = QLabel(
            "<b>Sag Measurement Method</b><br>"
            "Choose how sag is computed in the semantic viewer.<br>"
            "<b>Legacy chord-based sag</b> uses only the fitted wire curve.<br>"
            "<b>Fusion span catenary sag</b> matches the wire to Fusion pole-pair spans and only works after the Fusion step completes."
        )
        sag_method_desc.setWordWrap(True)
        sag_method_desc.setStyleSheet(f"color: {THEME['muted']}; font-size: 11px; background-color: {THEME['pane_raised']}; padding: 8px; border-radius: 4px;")
        sag_method_layout.addWidget(sag_method_desc)

        form.addRow("Sag Method:", sag_method_widget)
        
        layout.addLayout(form)
        
        # Dialog buttons
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
    
    def get_parameters(self):
        """Get the configured parameters"""
        return {
            "R": self.r_spin.value(),
            "angleThr": self.angle_spin.value(),
            "linearity": self.linearity_spin.value(),
            "sag_method": self.sag_method_combo.currentData(),
        }


class StepInfoDialog(QDialog):
    """Read-only explanation dialog for a pipeline stage."""

    def __init__(self, title: str, body_markdown: str, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(700, 420)

        layout = QVBoxLayout()
        self.setLayout(layout)

        title_label = QLabel(title)
        title_label.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {THEME['text']}; margin-bottom: 8px;")
        layout.addWidget(title_label)

        body = QTextEdit()
        body.setReadOnly(True)
        body.setMarkdown(body_markdown)
        body.setProperty("log", True)
        layout.addWidget(body, stretch=1)

        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(self.reject)
        buttons.accepted.connect(self.accept)
        layout.addWidget(buttons)


class InferenceParametersDialog(QDialog):
    """Configure inference runtime and optional weights override."""

    def __init__(self, current_config=None, scan_dir: Path | None = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Image Inference Runtime")
        self.setMinimumWidth(620)
        self.scan_dir = scan_dir
        current_config = current_config or {}
        self._setup_ui(current_config)

    def _browse_weights(self):
        chosen, _ = QFileDialog.getOpenFileName(
            self,
            "Select YOLO weights",
            str(self.scan_dir) if self.scan_dir is not None else "",
            "PyTorch Weights (*.pt);;All Files (*)",
        )
        if chosen:
            self.weights_edit.setText(chosen)

    def _setup_ui(self, current_config):
        layout = QVBoxLayout()
        self.setLayout(layout)

        title = QLabel("Configure Image Inference Runtime")
        title.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {THEME['text']}; margin-bottom: 10px;")
        layout.addWidget(title)

        help_text = QLabel(
            "This stage runs YOLO segmentation on the JPGs exported during rosbag preprocessing. "
            "Default behavior is to use the local NVIDIA GPU and the repo's bundled YOLO weights when available. "
            "The GUI always targets the first local CUDA device (`cuda:0`) for local inference; the numeric device "
            "selector is kept internal so the user does not need to manage GPU ordinals."
        )
        help_text.setWordWrap(True)
        help_text.setStyleSheet(f"color: {THEME['muted']}; background-color: {THEME['pane_raised']}; padding: 8px; border-radius: 4px;")
        layout.addWidget(help_text)

        form = QFormLayout()
        self.runtime_combo = QComboBox()
        self.runtime_combo.addItems(["local", "auto", "colab"])
        self.runtime_combo.setCurrentText(str(current_config.get("runtime", "local")))
        form.addRow("Runtime Policy:", self.runtime_combo)

        self.preferred_gpu_edit = QLineEdit(str(current_config.get("preferred_colab_gpu", "A100")))
        form.addRow("Preferred Colab GPU (fallback only):", self.preferred_gpu_edit)

        weights_row = QWidget()
        weights_layout = QHBoxLayout()
        weights_layout.setContentsMargins(0, 0, 0, 0)
        weights_row.setLayout(weights_layout)
        self.weights_edit = QLineEdit(str(current_config.get("weights", "")))
        weights_layout.addWidget(self.weights_edit, stretch=1)
        browse_btn = QPushButton("Browse")
        browse_btn.clicked.connect(self._browse_weights)
        weights_layout.addWidget(browse_btn)
        form.addRow("Weights Override (.pt, optional):", weights_row)

        layout.addLayout(form)

        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    def get_config(self):
        weights_value = self.weights_edit.text().strip()
        if weights_value and self.scan_dir is not None:
            weights_value = relativize_for_scan(self.scan_dir, weights_value)
        return {
            "runtime": self.runtime_combo.currentText().strip() or "local",
            "local_device": "0",
            "preferred_colab_gpu": self.preferred_gpu_edit.text().strip() or "A100",
            "weights": weights_value,
        }


class FusionParametersDialog(QDialog):
    """Link a completed calibration run and GPS settings."""

    def __init__(
        self,
        fusion_config=None,
        gps_config=None,
        timing_overrides=None,
        effective_timing=None,
        scan_dir: Path | None = None,
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Calibration Link and GPS Mapping")
        self.setMinimumWidth(700)
        self.scan_dir = scan_dir
        self._setup_ui(
            fusion_config or {},
            gps_config or {},
            timing_overrides or {},
            effective_timing or {},
        )

    def _browse_directory(self, target_edit: QLineEdit, title: str):
        chosen = QFileDialog.getExistingDirectory(
            self,
            title,
            str(self.scan_dir) if self.scan_dir is not None else "",
        )
        if chosen:
            target_edit.setText(chosen)

    def _setup_ui(self, fusion_config, gps_config, timing_overrides, effective_timing):
        layout = QVBoxLayout()
        self.setLayout(layout)

        title = QLabel("Link Calibration Output and GPS Mapping")
        title.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {THEME['text']}; margin-bottom: 10px;")
        layout.addWidget(title)

        help_text = QLabel(
            "Fusion consumes outputs from two earlier modes: calibration mode provides camera intrinsics and the LiDAR-camera extrinsic transform, "
            "and rosbag preprocessing provides the SLAM cloud, JPGs, and TF CSVs. This dialog links the completed calibration run, GPS lever-arm settings, "
            "and the developer-mode GPS override for bags that do not contain NavSatFix data."
        )
        help_text.setWordWrap(True)
        help_text.setStyleSheet(f"color: {THEME['muted']}; background-color: {THEME['pane_raised']}; padding: 8px; border-radius: 4px;")
        layout.addWidget(help_text)

        form = QFormLayout()

        calibration_row = QWidget()
        calibration_layout = QHBoxLayout()
        calibration_layout.setContentsMargins(0, 0, 0, 0)
        calibration_row.setLayout(calibration_layout)
        self.calibration_run_edit = QLineEdit(str(fusion_config.get("calibration_run_dir", "")))
        calibration_layout.addWidget(self.calibration_run_edit, stretch=1)
        calibration_browse = QPushButton("Browse")
        calibration_browse.clicked.connect(lambda: self._browse_directory(self.calibration_run_edit, "Select completed calibration run folder"))
        calibration_layout.addWidget(calibration_browse)
        form.addRow("Calibration Run Folder:", calibration_row)

        self.gps_offset_edit = QLineEdit(str(gps_config.get("offset_body_xyz_m", "0,0,0")))
        form.addRow("GPS->LiDAR Offset (x,y,z m):", self.gps_offset_edit)

        self.allow_missing_gps_cb = QCheckBox("Developer mode: allow missing GPS and skip georeferencing")
        self.allow_missing_gps_cb.setChecked(bool(gps_config.get("allow_missing", False)))
        layout.addWidget(self.allow_missing_gps_cb)

        timing_title = QLabel("Timing Calibration Overrides (per scan)")
        timing_title.setStyleSheet(f"font-weight: bold; color: {THEME['text']}; margin-top: 6px;")
        layout.addWidget(timing_title)

        self.override_fusion_cb = QCheckBox("Override Fusion time offset")
        self.override_fusion_cb.setChecked("fusion_time_offset_sec" in timing_overrides)
        layout.addWidget(self.override_fusion_cb)

        self.fusion_offset_ms = QDoubleSpinBox()
        self.fusion_offset_ms.setRange(-5000.0, 5000.0)
        self.fusion_offset_ms.setDecimals(3)
        self.fusion_offset_ms.setSingleStep(0.5)
        self.fusion_offset_ms.setSuffix(" ms")
        self.fusion_offset_ms.setValue(
            float(timing_overrides.get("fusion_time_offset_sec", effective_timing.get("fusion_time_offset_sec", 0.0))) * 1000.0
        )
        form.addRow("Fusion Time Offset:", self.fusion_offset_ms)

        effective_label = QLabel(
            "A zero fusion offset uses the existing nearest-pose camera CSV matching. "
            "A non-zero offset shifts frame timestamps and uses interpolated pose lookup. "
            "GPS georeferencing remains strict by default; enable developer mode above only for bags that truly have no GPS."
        )
        effective_label.setWordWrap(True)
        effective_label.setStyleSheet(f"color: {THEME['muted']}; background-color: {THEME['pane_raised']}; padding: 8px; border-radius: 4px;")
        layout.addWidget(effective_label)

        layout.addLayout(form)

        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    def get_configs(self):
        calibration_run_value = self.calibration_run_edit.text().strip()
        if calibration_run_value and self.scan_dir is not None:
            calibration_run_value = relativize_for_scan(self.scan_dir, calibration_run_value)

        fusion_config = {
            "calibration_run_dir": calibration_run_value,
        }
        gps_config = {
            "offset_body_xyz_m": self.gps_offset_edit.text().strip() or "0,0,0",
            "allow_missing": bool(self.allow_missing_gps_cb.isChecked()),
        }
        timing_overrides = {}
        if self.override_fusion_cb.isChecked():
            timing_overrides["fusion_time_offset_sec"] = float(self.fusion_offset_ms.value()) / 1000.0
        return fusion_config, gps_config, timing_overrides


class GlobalTimingSettingsDialog(QDialog):
    def __init__(self, current_settings=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Global Settings")
        self.setMinimumWidth(560)
        self._setup_ui(current_settings or {})

    def _setup_ui(self, current_settings):
        layout = QVBoxLayout()
        self.setLayout(layout)

        title = QLabel("Global Timing, GPS, Fusion, And Pose-Recovery Defaults")
        title.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {THEME['text']}; margin-bottom: 10px;")
        layout.addWidget(title)

        description = QLabel(
            "These defaults apply to every scan unless that scan sets explicit timing overrides. "
            "Timing values are shown in milliseconds but stored in seconds. "
            "The GPS developer-mode toggle lets you process bags without NavSatFix data from the GUI, "
            "and the blur-filter toggle controls whether rosbag preprocessing removes blurry exported camera frames."
        )
        description.setWordWrap(True)
        description.setStyleSheet(f"color: {THEME['muted']}; background-color: {THEME['pane_raised']}; padding: 8px; border-radius: 4px;")
        layout.addWidget(description)

        form = QFormLayout()
        current_timing = current_settings.get("timing", {})
        current_fusion = current_settings.get("fusion", {})
        current_gps = current_settings.get("gps", {})
        current_pose_recovery = current_settings.get("pose_recovery", {})

        self.fusion_offset_ms = QDoubleSpinBox()
        self.fusion_offset_ms.setRange(-5000.0, 5000.0)
        self.fusion_offset_ms.setDecimals(3)
        self.fusion_offset_ms.setSingleStep(0.5)
        self.fusion_offset_ms.setSuffix(" ms")
        self.fusion_offset_ms.setValue(float(current_timing.get("fusion_time_offset_sec", 0.0)) * 1000.0)
        form.addRow("Default Fusion Time Offset:", self.fusion_offset_ms)

        self.fusion_visualize_cb = QCheckBox("Enable live Open3D visualization during Fusion")
        self.fusion_visualize_cb.setChecked(bool(current_fusion.get("visualize", False)))
        form.addRow("Fusion Visualization:", self.fusion_visualize_cb)

        self.global_allow_missing_gps_cb = QCheckBox(
            "Developer mode: allow missing GPS and skip georeferencing for all scans"
        )
        self.global_allow_missing_gps_cb.setChecked(bool(current_gps.get("allow_missing", False)))
        form.addRow("GPS Developer Mode:", self.global_allow_missing_gps_cb)

        self.global_blur_filter_cb = QCheckBox(
            "Enable blur filtering during rosbag preprocessing"
        )
        self.global_blur_filter_cb.setChecked(bool(current_pose_recovery.get("blur_filter_enabled", True)))
        form.addRow("Pose-Recovery Blur Filter:", self.global_blur_filter_cb)

        layout.addLayout(form)
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    def get_settings(self):
        return {
            "timing": {
                "fusion_time_offset_sec": float(self.fusion_offset_ms.value()) / 1000.0,
            },
            "fusion": {
                "visualize": bool(self.fusion_visualize_cb.isChecked()),
            },
            "gps": {
                "allow_missing": bool(self.global_allow_missing_gps_cb.isChecked()),
            },
            "pose_recovery": {
                "blur_filter_enabled": bool(self.global_blur_filter_cb.isChecked()),
            },
        }


class NotificationPanel(QWidget):
    """Dropdown notification panel"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Popup | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground, False)
        self.setFixedSize(350, 400)
        self._setup_ui()
        self.notifications = []
        
    def _setup_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        self.setLayout(layout)
        
        self.setStyleSheet(notification_panel_style())
        
        header = QLabel("Notifications")
        header.setStyleSheet(f"font-size: 16px; font-weight: bold; border: none; color: {THEME['text']};")
        layout.addWidget(header)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("border: none;")
        self.notification_container = QWidget()
        self.notification_layout = QVBoxLayout()
        self.notification_layout.setContentsMargins(0, 0, 0, 0)
        self.notification_container.setLayout(self.notification_layout)
        scroll.setWidget(self.notification_container)
        layout.addWidget(scroll)
        
        btn_clear = QPushButton("Clear All")
        btn_clear.clicked.connect(self.clear_notifications)
        btn_clear.setStyleSheet(subtle_button_style())
        layout.addWidget(btn_clear)
    
    def add_notification(self, message, status="info"):
        """Add a notification"""
        notif_widget = QFrame()
        notif_widget.setStyleSheet(notification_item_style())
        
        notif_layout = QHBoxLayout()
        notif_layout.setContentsMargins(4, 4, 4, 4)
        notif_widget.setLayout(notif_layout)
        
        status_colors = {
            "done": "✔ 🟢",
            "running": "⏳ 🟡",
            "error": "❌ 🔴",
            "info": "ℹ️"
        }
        icon = QLabel(status_colors.get(status, "ℹ️"))
        notif_layout.addWidget(icon)
        
        msg_label = QLabel(message)
        msg_label.setWordWrap(True)
        msg_label.setStyleSheet("border: none; background: transparent;")
        notif_layout.addWidget(msg_label, stretch=1)
        
        time_label = QLabel(datetime.now().strftime("%H:%M"))
        time_label.setStyleSheet(f"color: {THEME['muted']}; border: none; background: transparent;")
        notif_layout.addWidget(time_label)
        
        self.notification_layout.insertWidget(0, notif_widget)
        self.notifications.append(notif_widget)
        
        if len(self.notifications) > 50:
            old = self.notifications.pop()
            old.deleteLater()
    
    def clear_notifications(self):
        """Clear all notifications"""
        for notif in self.notifications:
            notif.deleteLater()
        self.notifications.clear()


class StepByStepDashboard(QWidget):
    """Step-by-Step Dashboard with detailed control"""
    
    # Signals
    openViewerRequested = Signal(str)  # scan_path
    openMapRequested = Signal(str)  # scan_path
    runPipelineRequested = Signal(str)  # scan_path
    runStepRequested = Signal(str, str)  # scan_path, step_key
    createScanRequested = Signal()
    switchToAutoModeRequested = Signal()  # Switch back to auto mode
    openFilterViewerRequested = Signal(str)  # Open the point cloud filter viewer
    openSlamPointCloudRequested = Signal(str)  # point_cloud_path
    openImagesFolderRequested = Signal(str)  # images_dir_path
    
    STEP_COLUMNS = {
        "Rosbag Preprocessing": "slam",
        "Filtering": "filtering",
        "FLAI": "flai",
        "Wires": "wire_extraction",
        "Image Inference": "inference",
        "Fusion + GPS": "fusion",
    }

    STEP_DETAILS = {
        "slam": {
            "title": "Rosbag Preprocessing",
            "summary": "Replays the rosbag in Docker, runs FAST-LIO SLAM, copies the raw map into processed/point_clouds as PCD plus LAS, exports JPG frames from /image/compressed, and samples tf_camera_out.csv plus the dense trajectory. tf_gps_out.csv is still produced, but it may be empty only in developer mode for no-GPS bags.",
            "run_tooltip": "Run the full rosbag preprocessing stage: FAST-LIO SLAM, JPG export, image timestamp CSV export, dense /tf sampling, and GPS TF sampling when available.",
            "view_tooltip": "Open the latest SLAM point cloud from rosbag preprocessing in the Open3D viewer.",
            "modify_label": "Details",
            "modify_tooltip": "Show exactly what rosbag preprocessing does and which files it produces.",
        },
        "filtering": {
            "title": "Filtering",
            "summary": "Optional point-cloud cleanup and LAS inspection stage. This does not run automatically in the experimental backend.",
            "run_tooltip": "Filtering is currently manual. Use the filter viewer when a LAS or point cloud file is available.",
            "view_tooltip": "Open the point cloud filter viewer for the best available filtered/raw point cloud.",
            "modify_label": "Open",
            "modify_tooltip": "Open the filter viewer so you can inspect or adjust point-cloud filtering manually.",
        },
        "flai": {
            "title": "FLAI",
            "summary": "External/manual segmentation stage. If you use FLAI, import its segmented LAS output into the scan folder before opening the semantic viewer.",
            "run_tooltip": "FLAI is not automated in this experimental branch.",
            "view_tooltip": "Open the semantic viewer to inspect any current scan results.",
            "modify_label": "Guide",
            "modify_tooltip": "Show guidance for how FLAI fits into the current backend and where its outputs belong.",
        },
        "wire_extraction": {
            "title": "Wires",
            "summary": "Runs MATLAB-based wire extraction on a LAS/point cloud, producing wires_points.npz, wire_info.json, ground_points.npz, and a Leaflet-ready overlay export.",
            "run_tooltip": "Run the ENGR498 wire extraction stage for this scan.",
            "view_tooltip": "Open the semantic viewer, which overlays wire extraction outputs with fusion objects.",
            "modify_label": "Params",
            "modify_tooltip": "Adjust the wire extraction parameters stored in scan metadata.",
        },
        "inference": {
            "title": "Image Inference",
            "summary": "Runs YOLO segmentation on the JPG images exported during rosbag preprocessing and produces masks_npz/, meta_json/, pred_images/, and optionally a Colab bundle.",
            "run_tooltip": "Run YOLO inference using the preprocessing JPG frames for this scan.",
            "view_tooltip": "Open the semantic viewer for the scan. Inference artifacts are consumed by Fusion rather than viewed directly here.",
            "modify_label": "Runtime",
            "modify_tooltip": "Configure local-vs-Colab runtime policy, optional weight override, and fallback Colab GPU preference.",
        },
        "fusion": {
            "title": "Fusion + GPS",
            "summary": "Projects segmentation masks into the SLAM point cloud, applies dense-cluster cleanup and instance clustering, measures pole spacing, and georeferences the outputs when usable GPS samples exist. Developer mode can skip georeferencing for no-GPS bags.",
            "run_tooltip": "Run camera-LiDAR fusion and the GPS georeferencing/export stage for this scan.",
            "view_tooltip": "Open the semantic viewer with fused objects and wire overlays.",
            "modify_label": "Calibration",
            "modify_tooltip": "Link a completed calibration run and set the GPS lever-arm used by Fusion.",
        },
    }

    @staticmethod
    def _is_checked_state(state) -> bool:
        if isinstance(state, bool):
            return state
        try:
            return int(state) == int(Qt.CheckState.Checked.value)
        except Exception:
            return False
    
    def __init__(self, assets_path="ENGR-498-Project/assets", parent=None):
        super().__init__(parent)
        self.assets_path = Path(assets_path)
        self.selected_scan = None
        self.scans_data = {}
        self._scan_snapshot = ()
        
        self._setup_ui()
        self._setup_refresh_timer()
        self.refresh_scans()
        
    def _setup_ui(self):
        """Setup the main UI layout"""
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(16, 16, 16, 16)
        main_layout.setSpacing(12)
        self.setLayout(main_layout)
        
        # Top bar
        top_bar = self._create_top_bar()
        main_layout.addWidget(top_bar)
        
        # Controls bar
        controls_bar = self._create_controls_bar()
        main_layout.addWidget(controls_bar)

        # Scan table
        self.table = self._create_scan_table()
        main_layout.addWidget(self.table, stretch=1)

        self.log_output = self._create_log_panel()
        main_layout.addWidget(self.log_output)
        
        # Status bar
        self.status_label = QLabel("Post-Processing Step-By-Step: Rosbag Preprocessing -> Wires -> Image Inference -> Fusion + GPS, with manual Filtering and FLAI hooks.")
        self.status_label.setStyleSheet(f"color: {THEME['muted']}; padding: 8px;")
        main_layout.addWidget(self.status_label)
        
        # Apply global styles
        self.setStyleSheet(
            f"""
            QWidget {{
                background-color: {THEME['bg']};
                color: {THEME['text']};
                font-family: 'Segoe UI', Arial, sans-serif;
            }}
            QLabel {{
                color: {THEME['text']};
                background: transparent;
            }}
            QCheckBox {{
                color: {THEME['text']};
                spacing: 6px;
                font-size: 12px;
            }}
            QTableWidget {{
                background-color: {THEME['input']};
                color: {THEME['text']};
                border: 1px solid {THEME['border']};
                border-radius: {THEME['radius_md']};
                gridline-color: {THEME['border']};
                alternate-background-color: {THEME['pane']};
            }}
            QTableWidget::item {{
                padding: 8px;
                color: {THEME['text']};
            }}
            QTableWidget::item:selected {{
                background-color: {THEME['selection']};
                color: #ffffff;
            }}
            QHeaderView::section {{
                background-color: {THEME['pane_raised']};
                color: {THEME['text']};
                padding: 10px;
                border: none;
                border-bottom: 1px solid {THEME['border_strong']};
                font-weight: bold;
            }}
        """
        )
    
    def _create_top_bar(self):
        """Create top bar"""
        bar = QWidget()
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        bar.setLayout(layout)
        
        title = QLabel("LiDAR Processing Dashboard")
        title_font = QFont()
        title_font.setPointSize(20)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setStyleSheet(f"color: {THEME['text']};")
        layout.addWidget(title)
        
        # Mode badge
        mode_badge = QLabel("STEP-BY-STEP MODE")
        mode_badge.setStyleSheet("""
            QLabel {
                background-color: #673AB7;
                color: white;
                padding: 6px 12px;
                border-radius: 4px;
                font-weight: bold;
                font-size: 12px;
            }
        """)
        layout.addWidget(mode_badge)
        
        layout.addStretch()
        
        # Notification bell
        self.notification_panel = NotificationPanel(self)
        self.btn_notifications = QToolButton()
        self.btn_notifications.setText("🔔")
        self.btn_notifications.setStyleSheet("""
            QToolButton {
                font-size: 24px;
                border: none;
                background: transparent;
                padding: 8px;
            }
            QToolButton:hover {
                background-color: #e0e0e0;
                border-radius: 4px;
            }
        """)
        self.btn_notifications.clicked.connect(self._show_notifications)
        layout.addWidget(self.btn_notifications)
        
        return bar
    
    def _create_controls_bar(self):
        """Create controls bar"""
        bar = QFrame()
        bar.setStyleSheet(
            f"QFrame {{ background-color: {THEME['pane']}; border: 1px solid {THEME['border']}; border-radius: {THEME['radius_md']}; padding: 12px; }}"
        )
        layout = QHBoxLayout()
        bar.setLayout(layout)
        
        # Create New Scan button
        btn_new_scan = QPushButton("➕ Upload New Scan")
        btn_new_scan.setStyleSheet(button_style(THEME["success"], "#4ade80"))
        btn_new_scan.clicked.connect(self.createScanRequested.emit)
        layout.addWidget(btn_new_scan)

        self.btn_run_pipeline = QPushButton("Run Full Pipeline")
        self.btn_run_pipeline.setEnabled(False)
        self.btn_run_pipeline.setStyleSheet(button_style("#7c3aed", "#8b5cf6"))
        self.btn_run_pipeline.setToolTip("Run the automated backend chain for the selected scan: Rosbag Preprocessing -> Wires -> Image Inference -> Fusion + GPS.")
        self.btn_run_pipeline.clicked.connect(self._run_full_pipeline)
        layout.addWidget(self.btn_run_pipeline)

        btn_timing = QPushButton("Global Settings")
        btn_timing.setToolTip(
            "Set global timing defaults, the GPS developer-mode override, and the Fusion visualization default."
        )
        btn_timing.setStyleSheet(button_style("#0f766e", "#14b8a6"))
        btn_timing.clicked.connect(self._open_global_timing_dialog)
        layout.addWidget(btn_timing)

        layout.addStretch()
        
        # Switch to Auto Mode button
        btn_auto_mode = QPushButton("⚡ Auto Mode")
        btn_auto_mode.setToolTip("Switch to automatic pipeline processing")
        btn_auto_mode.setStyleSheet(button_style("#7c3aed", "#8b5cf6"))
        btn_auto_mode.clicked.connect(self.switchToAutoModeRequested.emit)
        layout.addWidget(btn_auto_mode)
        
        # Refresh button
        btn_refresh = QPushButton("🔄")
        btn_refresh.setToolTip("Refresh scans")
        btn_refresh.setStyleSheet(subtle_button_style())
        btn_refresh.clicked.connect(lambda: self.refresh_scans(force=True))
        layout.addWidget(btn_refresh)
        
        return bar

    def _open_global_timing_dialog(self):
        settings = load_global_timing_settings()
        dialog = GlobalTimingSettingsDialog(current_settings=settings, parent=self)
        if dialog.exec() == QDialog.Accepted:
            settings.update(dialog.get_settings())
            save_global_timing_settings(settings)
            self.add_notification("Updated global timing, GPS, and Fusion defaults", "done")
            self.status_label.setText("Updated global timing, GPS, and Fusion defaults.")

    def _create_log_panel(self) -> QTextEdit:
        log_output = QTextEdit()
        log_output.setReadOnly(True)
        log_output.setMinimumHeight(170)
        log_output.setPlaceholderText("Backend console output will appear here while Rosbag Preprocessing, Wires, Image Inference, and Fusion + GPS run.")
        log_output.setProperty("log", True)
        return log_output

    def _create_pipeline_overview(self):
        """Create a short in-context explanation of what each backend stage does."""
        panel = QFrame()
        panel.setStyleSheet(
            f"QFrame {{ background-color: {THEME['pane']}; border: 1px solid {THEME['border']}; border-radius: {THEME['radius_md']}; padding: 10px; }}"
        )
        layout = QVBoxLayout()
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)
        panel.setLayout(layout)

        title = QLabel("Backend Stage Summary")
        title.setStyleSheet(f"font-size: 15px; font-weight: bold; color: {THEME['text']};")
        layout.addWidget(title)

        summary = QLabel(
            "<b>Rosbag Preprocessing</b> replays the rosbag, runs FAST-LIO SLAM, stages raw <code>scans.pcd</code> plus a LAS conversion in <code>processed/point_clouds/</code>, "
            "exports JPGs from <code>/image/compressed</code>, writes <code>image_timestamps.csv</code>, and samples "
            "both <code>tf_camera_out.csv</code> and <code>tf_gps_out.csv</code>. <b>Image Inference</b> runs YOLO "
            "on those JPGs and writes <code>masks_npz/</code> plus <code>meta_json/</code>. <b>Fusion + GPS</b> "
            "projects the masks into the SLAM cloud, keeps dense clusters, segments utility assets, measures pole "
            "spacing, and georeferences the results when GPS data is available. <b>Wires</b> remains the MATLAB wire "
            "extraction path, and calibration stays in its own separate GUI mode where camera intrinsics are extracted "
            "from <code>/camera/camera_info</code> and the LiDAR-camera extrinsic is solved."
        )
        summary.setWordWrap(True)
        summary.setStyleSheet(f"color: {THEME['muted']}; line-height: 1.35;")
        layout.addWidget(summary)

        legend = QLabel(
            "Use <b>Run</b> to execute a post-processing stage, <b>View</b> to inspect the current result layer, and the right-most "
            "button to open that stage's actual details, guide, parameters, or configuration. Calibration is intentionally separate from this table."
        )
        legend.setWordWrap(True)
        legend.setStyleSheet(f"color: {THEME['muted']};")
        layout.addWidget(legend)
        return panel
    
    def _create_scan_table(self):
        """Create the scan table with checkboxes"""
        table = QTableWidget()
        
        columns = ["Scan Name"] + list(self.STEP_COLUMNS.keys()) + ["Actions"]
        table.setColumnCount(len(columns))
        table.setHorizontalHeaderLabels(columns)

        for idx, header_text in enumerate(columns):
            item = table.horizontalHeaderItem(idx)
            if item is None:
                continue
            if header_text in self.STEP_COLUMNS:
                step_key = self.STEP_COLUMNS[header_text]
                item.setToolTip(self.STEP_DETAILS[step_key]["summary"])
            elif header_text == "Actions":
                item.setToolTip("Open the semantic/map views, run the full chain, open the latest SLAM point cloud, open exported images, or delete the scan.")
            else:
                item.setToolTip("The scan directory under assets/ containing raw bags, processed outputs, and metadata.")
        
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setSelectionMode(QTableWidget.SingleSelection)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)
        
        # Fix row height
        table.verticalHeader().setDefaultSectionSize(120)
        
        # Column sizing
        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)  # Scan Name
        for i in range(1, len(columns) - 1):  # Step columns
            header.setSectionResizeMode(i, QHeaderView.Fixed)
            table.setColumnWidth(i, 225)
        header.setSectionResizeMode(len(columns) - 1, QHeaderView.Fixed)  # Actions
        table.setColumnWidth(len(columns) - 1, 520)
        
        table.setMinimumHeight(300)
        table.itemSelectionChanged.connect(self._on_selection_changed)
        
        return table
    
    def _setup_refresh_timer(self):
        """Setup timer to auto-refresh table"""
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_scans)
        self.refresh_timer.start(3000)

    def _collect_scan_snapshot(self) -> tuple[tuple[str, int, int], ...]:
        snapshot = []
        if not self.assets_path.exists():
            return tuple()

        for scan_dir in sorted(self.assets_path.iterdir(), key=lambda path: path.name):
            metadata_path = scan_dir / "metadata.json"
            if not scan_dir.is_dir() or not metadata_path.is_file():
                continue
            stat = metadata_path.stat()
            snapshot.append((scan_dir.name, stat.st_mtime_ns, stat.st_size))
        return tuple(snapshot)

    def _persist_scan_metadata(self, scan_name: str, metadata: dict, *, status_message: str | None = None) -> None:
        scan_dir = self.assets_path / scan_name
        save_scan_metadata(scan_dir, metadata)
        _, reloaded = load_scan_metadata(scan_dir)
        self.scans_data[scan_name] = reloaded
        self._scan_snapshot = self._collect_scan_snapshot()
        if status_message:
            self.status_label.setText(status_message)

    def refresh_scans(self, force: bool = False):
        """Refresh scan data"""
        if not self.assets_path.exists():
            self.assets_path.mkdir(parents=True, exist_ok=True)
            print(f"Created assets directory at {self.assets_path.resolve()}")
            return

        snapshot = self._collect_scan_snapshot()
        if not force and snapshot == self._scan_snapshot:
            return

        self._scan_snapshot = snapshot
        selected_scan = self.selected_scan
        self.scans_data = {}
        
        for scan_dir in self.assets_path.iterdir():
            if not scan_dir.is_dir():
                continue
            if not (scan_dir / "metadata.json").is_file():
                continue

            try:
                _, metadata = load_scan_metadata(scan_dir)
                self.scans_data[scan_dir.name] = metadata
            except Exception as e:
                print(f"Error loading metadata for {scan_dir.name}: {e}")
        
        self._update_table()
        if selected_scan and selected_scan in self.scans_data:
            self.select_scan(selected_scan)

    def select_scan(self, scan_name: str) -> None:
        for row in range(self.table.rowCount()):
            item = self.table.item(row, 0)
            if item is None:
                continue
            if item.data(Qt.UserRole) == scan_name:
                self.table.setCurrentCell(row, 0)
                self.table.scrollToItem(item, QTableWidget.PositionAtCenter)
                return
    
    def _update_table(self):

        self.table.setRowCount(0)
        #the above just clears the table, this might be unncessary, but I'm leaving it here. 
        
        """Update table with checkboxes and buttons"""
        self.table.setRowCount(len(self.scans_data))
        
        for row, (scan_name, metadata) in enumerate(sorted(self.scans_data.items())):
            # Scan Name
            name_item = QTableWidgetItem(scan_name)
            name_item.setData(Qt.UserRole, scan_name)
            self.table.setItem(row, 0, name_item)
            
            # Step columns with checkboxes + view/modify buttons
            status_dict = metadata.get("status", {})
            files_dict = metadata.get("files", {})
            
            for col_idx, (col_name, step_key) in enumerate(self.STEP_COLUMNS.items(), start=1):
                step_widget = self._create_step_widget(scan_name, step_key, status_dict, files_dict)
                self.table.setCellWidget(row, col_idx, step_widget)
            
            # Actions column
            actions_widget = self._create_actions_widget(scan_name, metadata)
            self.table.setCellWidget(row, len(self.STEP_COLUMNS) + 1, actions_widget)
    
    def _create_step_widget(self, scan_name, step_key, status_dict, files_dict):
        """Create widget for each step with checkbox and buttons"""
        step_info = self.STEP_DETAILS[step_key]
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(5)
        widget.setLayout(layout)
        widget.setToolTip(step_info["summary"])
        widget.setMinimumHeight(112)

        # Checkbox for completion
        checkbox = QCheckBox("Complete")
        checkbox.setStyleSheet(
            f"QCheckBox {{ color: {THEME['text']}; spacing: 8px; }}"
            f"QCheckBox::indicator {{ width: 16px; height: 16px; border: 2px solid {THEME['border_strong']}; border-radius: 3px; background: {THEME['input']}; }}"
            "QCheckBox::indicator:checked { background: #673AB7; border: 2px solid #673AB7; }"
        )
        checkbox.setChecked(status_dict.get(step_key) == "done")
        checkbox.setToolTip(
            f"Mark {step_info['title']} complete in metadata. This saves immediately and does not run the stage by itself."
        )
        checkbox.stateChanged.connect(lambda state, sn=scan_name, sk=step_key: self._on_checkbox_changed(sn, sk, state))
        layout.addWidget(checkbox)

        if step_key == "slam":
            pose_cfg = self.scans_data.get(scan_name, {}).get("config", {}).get("pose_recovery", {})
            rviz_checkbox = QCheckBox("Show RViz")
            rviz_checkbox.setStyleSheet(
                f"QCheckBox {{ color: {THEME['text']}; spacing: 8px; }}"
                f"QCheckBox::indicator {{ width: 16px; height: 16px; border: 2px solid {THEME['border_strong']}; border-radius: 3px; background: {THEME['input']}; }}"
                f"QCheckBox::indicator:checked {{ background: #7c3aed; border: 2px solid #7c3aed; }}"
            )
            rviz_checkbox.setChecked(bool(pose_cfg.get("enable_rviz", False)))
            rviz_checkbox.setToolTip(
                "Launch RViz alongside FAST-LIO so you can watch the SLAM point cloud being built in real time."
            )
            rviz_checkbox.stateChanged.connect(
                lambda state, sn=scan_name: self._on_pose_recovery_rviz_changed(sn, state)
            )
            layout.addWidget(rviz_checkbox)

        # Buttons layout
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(4)
        btn_layout.setContentsMargins(0, 2, 0, 0)

        btn_run = QPushButton("Run")
        btn_run.setStyleSheet(compact_step_button_style("#7c3aed", "#8b5cf6"))
        btn_run.setToolTip(step_info["run_tooltip"])
        btn_run.setFixedHeight(30)
        btn_run.setMinimumWidth(56)
        btn_run.clicked.connect(
            lambda checked=False, sp=str(self.assets_path / scan_name), sk=step_key: self.runStepRequested.emit(sp, sk)
        )
        btn_layout.addWidget(btn_run, 1)
        
        # View button
        btn_view = QPushButton("View")
        btn_view.setStyleSheet(compact_step_button_style(THEME["accent"], THEME["accent_hover"]))
        btn_view.setToolTip(step_info["view_tooltip"])
        btn_view.setFixedHeight(30)
        btn_view.setMinimumWidth(60)
        btn_view.clicked.connect(lambda: self._on_view_step(scan_name, step_key))
        btn_layout.addWidget(btn_view, 1)

        # Context button
        btn_modify = QPushButton(step_info["modify_label"])
        btn_modify.setStyleSheet(compact_step_button_style(THEME["warning"], "#fbbf24", text="#111827"))
        btn_modify.setToolTip(step_info["modify_tooltip"])
        btn_modify.setFixedHeight(30)
        btn_modify.setMinimumWidth(70)
        btn_modify.clicked.connect(lambda: self._on_modify_step(scan_name, step_key))
        btn_layout.addWidget(btn_modify, 1)
        
        layout.addLayout(btn_layout)
        
        return widget
    
    def _create_actions_widget(self, scan_name, metadata):
        """Create actions column"""
        widget = QWidget()
        layout = QHBoxLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        widget.setLayout(layout)
        status_dict = metadata.get("status", {})
        files_dict = metadata.get("files", {})
        scan_dir = self.assets_path / scan_name
        pcd_path = resolve_scan_path(scan_dir, files_dict.get("pcd"))
        images_dir = resolve_scan_path(scan_dir, files_dict.get("images_dir"))
        slam_complete = status_dict.get("slam") == "done"
        pcd_ready = slam_complete and pcd_path is not None and pcd_path.is_file()
        images_ready = slam_complete and images_dir is not None and images_dir.is_dir()

        # View Result button
        btn_view = QPushButton("View Result")
        btn_view.setStyleSheet(button_style(THEME["success"], "#4ade80"))
        btn_view.clicked.connect(lambda: self.openViewerRequested.emit(str(self.assets_path / scan_name)))
        btn_view.setToolTip("Open the combined semantic viewer for this scan.")
        layout.addWidget(btn_view)

        btn_run_full = QPushButton("Run Full")
        btn_run_full.setStyleSheet(button_style("#7c3aed", "#8b5cf6"))
        btn_run_full.clicked.connect(lambda: self.runPipelineRequested.emit(str(self.assets_path / scan_name)))
        btn_run_full.setToolTip("Run the automated backend chain for this scan.")
        layout.addWidget(btn_run_full)

        btn_map = QPushButton("Map")
        btn_map.setStyleSheet(button_style(THEME["accent"], THEME["accent_hover"]))
        btn_map.clicked.connect(lambda: self.openMapRequested.emit(str(self.assets_path / scan_name)))
        btn_map.setToolTip("Open the Leaflet map for this scan if fusion objects and/or powerline overlays exist.")
        layout.addWidget(btn_map)

        btn_pcd = QPushButton("PCD")
        btn_pcd.setStyleSheet(button_style("#2563eb", "#1d4ed8"))
        btn_pcd.setEnabled(pcd_ready)
        btn_pcd.setToolTip(
            "Open the latest SLAM point cloud in the Open3D viewer."
            if pcd_ready
            else "Run Rosbag Preprocessing first to generate the raw SLAM point cloud."
        )
        if pcd_ready:
            btn_pcd.clicked.connect(lambda checked=False, fp=str(pcd_path): self.openSlamPointCloudRequested.emit(fp))
        layout.addWidget(btn_pcd)

        btn_images = QPushButton("Images")
        btn_images.setStyleSheet(button_style("#0f766e", "#115e59"))
        btn_images.setEnabled(images_ready)
        btn_images.setToolTip(
            "Open the folder containing the exported JPG frames."
            if images_ready
            else "Run Rosbag Preprocessing first to export the image frames."
        )
        if images_ready:
            btn_images.clicked.connect(lambda checked=False, fp=str(images_dir): self.openImagesFolderRequested.emit(fp))
        layout.addWidget(btn_images)
        
        # Delete button
        btn_delete = QPushButton("Delete")
        btn_delete.setToolTip("Delete scan")
        btn_delete.setStyleSheet(button_style(THEME["danger"], "#f87171"))
        btn_delete.clicked.connect(lambda: self._delete_scan(scan_name))
        layout.addWidget(btn_delete)
        
        return widget
    
    def _on_checkbox_changed(self, scan_name, step_key, state):
        """Handle checkbox state change"""
        is_checked = self._is_checked_state(state)
        new_status = "done" if is_checked else "pending"
        if scan_name not in self.scans_data:
            return

        try:
            scan_dir, metadata = load_scan_metadata(self.assets_path / scan_name)
            metadata.setdefault("status", {})[step_key] = new_status
            self._persist_scan_metadata(
                scan_name,
                metadata,
                status_message=f"Saved {self.STEP_DETAILS[step_key]['title']} status for {scan_name}: {new_status}.",
            )
            print(f"Saved {scan_name} -> {step_key}: {new_status}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save: {e}")
    
    def _on_view_step(self, scan_name, step_key):
        """Handle view button click"""
        title = self.STEP_DETAILS[step_key]["title"]
        print(f"VIEW clicked: {scan_name} -> {step_key}")
        self.add_notification(f"View requested: {scan_name} - {title}", "info")
        if step_key == "slam":
            scan_dir = self.assets_path / scan_name
            metadata = self.scans_data.get(scan_name, {})
            files_dict = metadata.get("files", {})
            pcd_path = resolve_scan_path(scan_dir, files_dict.get("pcd"))
            if pcd_path is not None and pcd_path.exists():
                self.openSlamPointCloudRequested.emit(str(pcd_path))
                return
            QMessageBox.information(
                self,
                "Rosbag Preprocessing",
                "No SLAM point cloud is available yet. Run Rosbag Preprocessing first, then use the Images action button if you want to open the exported JPG folder.",
            )
            return
        if step_key == "filtering":
            scan_dir = self.assets_path / scan_name
            metadata = self.scans_data.get(scan_name, {})
            _, filepath = first_existing_artifact(
                scan_dir,
                metadata,
                ("filtered_las", "raw_las", "filtered_pcd", "raw_pcd", "las", "pcd"),
            )
            if filepath is not None:
                self.openFilterViewerRequested.emit(str(filepath))
                return
            QMessageBox.information(
                self,
                "Filtering",
                "No filtered or raw point cloud is available yet. Run Rosbag Preprocessing first so the scan has a point cloud to inspect.",
            )
            return
        self.openViewerRequested.emit(str(self.assets_path / scan_name))

    def _run_full_pipeline(self):
        """Run the full backend pipeline for the selected scan."""
        if not self.selected_scan:
            return
        scan_path = str(self.assets_path / self.selected_scan)
        self.runPipelineRequested.emit(scan_path)
        self.add_notification(f"Started full pipeline for {self.selected_scan}", "running")
    

    #TODO I have to pass the parameters to wire extraction TODO TODO TODO TODO 
    def _on_modify_step(self, scan_name, step_key):
        """Handle modify button click"""
        scan_dir = self.assets_path / scan_name
        metadata = self.scans_data.get(scan_name, {})

        if step_key == "wire_extraction":
            # Show parameter dialog for wires
            current_params = metadata.get("wire_params", {})
            dialog = WireParametersDialog(current_params, self)
            
            if dialog.exec() == QDialog.Accepted:
                params = dialog.get_parameters()
                print(f"MODIFY Wire Parameters for {scan_name}:")
                print(f"  R (Search Radius): {params['R']} m")
                print(f"  angleThr (Angle Threshold): {params['angleThr']}°")
                print(f"  linearity (Linearity Threshold): {params['linearity']}")
                print(f"  sag_method: {params['sag_method']}")
                
                metadata["wire_params"] = params
                self._persist_scan_metadata(
                    scan_name,
                    metadata,
                    status_message=f"Saved wire extraction parameters for {scan_name}.",
                )
                self.add_notification(f"Saved wire parameters for {scan_name}", "done")
        elif step_key == "filtering":
            print(f"OPEN FILTER clicked: {scan_name} -> {step_key}")
            if scan_name in self.scans_data:
                key, resolved = first_existing_artifact(
                    scan_dir,
                    metadata,
                    ("filtered_las", "raw_las", "filtered_pcd", "raw_pcd", "las", "pcd"),
                )
                print(f"Opening filter viewer for {scan_name} with {key}: {resolved}")
                if resolved is not None:
                    self.openFilterViewerRequested.emit(str(resolved))
                    return
            QMessageBox.information(
                self,
                "Filtering",
                "Filtering is a manual step. Run Rosbag Preprocessing first, then use Open/View to inspect the available point cloud in the filter tool.",
            )
        elif step_key == "inference":
            current_config = metadata.get("config", {}).get("inference", {})
            dialog = InferenceParametersDialog(current_config=current_config, scan_dir=scan_dir, parent=self)
            if dialog.exec() == QDialog.Accepted:
                metadata.setdefault("config", {})["inference"] = dialog.get_config()
                self._persist_scan_metadata(
                    scan_name,
                    metadata,
                    status_message=f"Saved inference runtime settings for {scan_name}.",
                )
                self.add_notification(f"Saved inference runtime settings for {scan_name}", "done")
        elif step_key == "fusion":
            current_config = metadata.get("config", {})
            effective_timing = resolve_effective_timing(metadata)
            dialog = FusionParametersDialog(
                fusion_config=current_config.get("fusion", {}),
                gps_config=current_config.get("gps", {}),
                timing_overrides=current_config.get("timing_overrides", {}),
                effective_timing=effective_timing,
                scan_dir=scan_dir,
                parent=self,
            )
            if dialog.exec() == QDialog.Accepted:
                fusion_config, gps_config, timing_overrides = dialog.get_configs()
                metadata.setdefault("config", {})["fusion"] = fusion_config
                metadata.setdefault("config", {})["gps"] = gps_config
                metadata.setdefault("config", {})["timing_overrides"] = timing_overrides
                self._persist_scan_metadata(
                    scan_name,
                    metadata,
                    status_message=f"Saved Fusion calibration, GPS, and timing settings for {scan_name}.",
                )
                self.add_notification(f"Saved Fusion calibration/GPS settings for {scan_name}", "done")
        elif step_key == "slam":
            StepInfoDialog(
                "Rosbag Preprocessing Outputs",
                (
                    "### What this stage does\n"
                    "- Replays the rosbag inside the Docker preprocessing environment.\n"
                    "- Runs FAST-LIO SLAM to produce the local map point cloud.\n"
                    "- Saves every `/image/compressed` frame as a JPG under the pose-recovery run folder.\n"
                    "- Writes `image_timestamps.csv`, `tf_camera_out.csv`, `tf_dense_trajectory.csv`, and `tf_gps_out.csv`.\n"
                    "- In developer mode for no-GPS bags, `tf_gps_out.csv` may be header-only while the other outputs are still produced.\n\n"
                    "### What this stage does not do\n"
                    "- It does not compute camera intrinsics or LiDAR-camera extrinsics.\n"
                    "- Those come from the separate Calibration Mode, where intrinsics are extracted from `/camera/camera_info` and direct visual LiDAR calibration solves the extrinsic transform.\n\n"
                    "### Main outputs\n"
                    "- `processed/pose_recovery/<run>/pcd/scans.pcd`\n"
                    "- `processed/point_clouds/scans.pcd`\n"
                    "- `processed/point_clouds/cloud.las`\n"
                    "- `processed/pose_recovery/<run>/images/*.jpg`\n"
                    "- `processed/pose_recovery/<run>/image_timestamps.csv`\n"
                    "- `processed/pose_recovery/<run>/tf_camera_out.csv`\n"
                    "- `processed/pose_recovery/<run>/tf_dense_trajectory.csv`\n"
                    "- `processed/pose_recovery/<run>/tf_gps_out.csv`\n\n"
                    "### Optional live viewer\n"
                    "- Use the **Show RViz** checkbox directly in this Rosbag Preprocessing cell if you want RViz to open while FAST-LIO builds the map in real time.\n\n"
                    "### Why it matters\n"
                    "Image inference and Fusion both depend on the JPG images and camera timestamps created here."
                ),
                self,
            ).exec()
        elif step_key == "flai":
            StepInfoDialog(
                "FLAI Integration",
                (
                    "### Current state\n"
                    "FLAI is not automated in this experimental GUI branch.\n\n"
                    "### Expected role\n"
                    "- Run FLAI externally if you need segmented LAS input.\n"
                    "- Place or import the segmented LAS into the scan's processed folder.\n"
                    "- The semantic viewer can still load the scan and combine those results with wires and Fusion outputs."
                ),
                self,
            ).exec()
        else:
            print(f"MODIFY clicked: {scan_name} -> {step_key}")
            self.add_notification(f"Details requested: {scan_name} - {self.STEP_DETAILS[step_key]['title']}", "info")
    
    def _delete_scan(self, scan_name):
        """Delete a scan"""
        reply = QMessageBox.question(
            self,
            "Confirm Delete",
            f"Delete '{scan_name}'?",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            try:
                import shutil
                shutil.rmtree(self.assets_path / scan_name)
                del self.scans_data[scan_name]
                self.refresh_scans(force=True)
                self.add_notification(f"Deleted {scan_name}", "info")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to delete: {e}")
    
    def _on_selection_changed(self):
        """Handle selection change"""
        selected_items = self.table.selectedItems()
        if selected_items:
            self.selected_scan = selected_items[0].data(Qt.UserRole)
            self.status_label.setText(f"Selected: {self.selected_scan}")
            self.btn_run_pipeline.setEnabled(True)
        else:
            self.selected_scan = None
            self.status_label.setText("Step-by-Step Mode: run Rosbag Preprocessing, Wires, Image Inference, or Fusion + GPS individually. Calibration is a separate mode in the toolbar.")
            self.btn_run_pipeline.setEnabled(False)

    def _on_pose_recovery_rviz_changed(self, scan_name: str, state: int):
        scan_dir, metadata = load_scan_metadata(self.assets_path / scan_name)
        is_checked = self._is_checked_state(state)
        metadata.setdefault("config", {}).setdefault("pose_recovery", {})["enable_rviz"] = is_checked
        state_text = "enabled" if is_checked else "disabled"
        self._persist_scan_metadata(
            scan_name,
            metadata,
            status_message=f"Rosbag Preprocessing RViz preview {state_text} for {scan_name}",
        )
        self.add_notification(f"RViz preview {state_text} for {scan_name}", "info")
    
    def _show_notifications(self):
        """Show notification panel"""
        button_pos = self.btn_notifications.mapToGlobal(self.btn_notifications.rect().bottomLeft())
        panel_x = button_pos.x() - self.notification_panel.width() + self.btn_notifications.width()
        panel_y = button_pos.y() + 5
        self.notification_panel.move(panel_x, panel_y)
        self.notification_panel.show()
    
    def add_notification(self, message, status="info"):
        """Add notification"""
        self.notification_panel.add_notification(message, status)

    def append_log(self, message: str) -> None:
        if not message:
            return
        self.log_output.append(message)
        scrollbar = self.log_output.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def clear_log(self) -> None:
        self.log_output.clear()


if __name__ == "__main__":
    from PySide6.QtWidgets import QApplication
    import sys
    
    app = QApplication(sys.argv)
    
    # Create test data
    assets_path = Path("assets")
    assets_path.mkdir(exist_ok=True)
    
    scan1 = assets_path / "scan_001"
    scan1.mkdir(exist_ok=True)
    
    metadata = {
        "name": "scan_001",
        "created_at": "2026-03-17",
        "status": {
            "slam": "done",
            "filtering": "done",
            "flai": "pending",
            "wire_extraction": "pending",
            "fusion": "pending"
        },
        "files": {
            "raw_las": "processed/point_clouds/cloud.las",
            "raw_pcd": "processed/point_clouds/scans.pcd",
            "filtered_las": "processed/point_clouds/cloud_filtered.las",
            "filtered_pcd": "processed/point_clouds/scans_filtered.pcd"
        },
        "wire_params": {
            "R": 0.5,
            "angleThr": 10,
            "linearity": 0.98
        }
    }
    
    with open(scan1 / "metadata.json", 'w') as f:
        json.dump(metadata, f, indent=2)
    
    dashboard = StepByStepDashboard(assets_path="assets")
    dashboard.resize(1600, 800)
    dashboard.show()
    
    sys.exit(app.exec())
