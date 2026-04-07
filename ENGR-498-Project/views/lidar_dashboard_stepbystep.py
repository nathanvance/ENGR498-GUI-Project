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

from scan_metadata import load_scan_metadata, relativize_for_scan, resolve_scan_path, save_scan_metadata


class WireParametersDialog(QDialog):
    """Dialog for modifying wire extraction parameters"""
    
    def __init__(self, current_params=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Wire Extraction Parameters")
        self.setMinimumWidth(500)
        
        if current_params is None:
            current_params = {"R": 0.5, "angleThr": 10, "linearity": 0.98}
        
        self._setup_ui(current_params)
        
    def _setup_ui(self, current_params):
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title
        title = QLabel("Configure Wire Extraction Parameters")
        title.setStyleSheet("font-size: 16px; font-weight: bold; color: #1976D2; margin-bottom: 10px;")
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
        r_desc.setStyleSheet("color: #666; font-size: 11px; background-color: #f5f5f5; padding: 8px; border-radius: 4px;")
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
        angle_desc.setStyleSheet("color: #666; font-size: 11px; background-color: #f5f5f5; padding: 8px; border-radius: 4px;")
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
        linearity_desc.setStyleSheet("color: #666; font-size: 11px; background-color: #f5f5f5; padding: 8px; border-radius: 4px;")
        linearity_layout.addWidget(linearity_desc)
        
        form.addRow("Linearity Threshold:", linearity_widget)
        
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
            "linearity": self.linearity_spin.value()
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
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #1f3a5f; margin-bottom: 8px;")
        layout.addWidget(title_label)

        body = QTextEdit()
        body.setReadOnly(True)
        body.setMarkdown(body_markdown)
        body.setStyleSheet(
            "QTextEdit { background-color: #ffffff; color: #1f2937; border: 1px solid #d0d7de; "
            "border-radius: 6px; padding: 8px; }"
        )
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
        title.setStyleSheet("font-size: 16px; font-weight: bold; color: #1f3a5f; margin-bottom: 10px;")
        layout.addWidget(title)

        help_text = QLabel(
            "This stage runs YOLO segmentation on the JPGs exported during rosbag preprocessing. "
            "Default behavior is to use the local NVIDIA GPU and the repo's bundled YOLO weights when available. "
            "The GUI always targets the first local CUDA device (`cuda:0`) for local inference; the numeric device "
            "selector is kept internal so the user does not need to manage GPU ordinals."
        )
        help_text.setWordWrap(True)
        help_text.setStyleSheet("color: #4b5563; background-color: #eef4ff; padding: 8px; border-radius: 4px;")
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

    def __init__(self, fusion_config=None, gps_config=None, scan_dir: Path | None = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Calibration Link and GPS Mapping")
        self.setMinimumWidth(700)
        self.scan_dir = scan_dir
        self._setup_ui(fusion_config or {}, gps_config or {})

    def _browse_directory(self, target_edit: QLineEdit, title: str):
        chosen = QFileDialog.getExistingDirectory(
            self,
            title,
            str(self.scan_dir) if self.scan_dir is not None else "",
        )
        if chosen:
            target_edit.setText(chosen)

    def _setup_ui(self, fusion_config, gps_config):
        layout = QVBoxLayout()
        self.setLayout(layout)

        title = QLabel("Link Calibration Output and GPS Mapping")
        title.setStyleSheet("font-size: 16px; font-weight: bold; color: #1f3a5f; margin-bottom: 10px;")
        layout.addWidget(title)

        help_text = QLabel(
            "Fusion consumes outputs from two earlier modes: calibration mode provides camera intrinsics and the LiDAR-camera extrinsic transform, "
            "and rosbag preprocessing provides the SLAM cloud, JPGs, and TF CSVs. This dialog should only link the completed calibration run and GPS lever-arm settings."
        )
        help_text.setWordWrap(True)
        help_text.setStyleSheet("color: #4b5563; background-color: #eef8ef; padding: 8px; border-radius: 4px;")
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
        }
        return fusion_config, gps_config


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
        
        self.setStyleSheet("""
            QWidget {
                background-color: white;
                border: 1px solid #ddd;
                border-radius: 8px;
            }
        """)
        
        header = QLabel("Notifications")
        header.setStyleSheet("font-size: 16px; font-weight: bold; border: none;")
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
        btn_clear.setStyleSheet("""
            QPushButton {
                background-color: #f5f5f5;
                border: 1px solid #ddd;
                padding: 6px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
        """)
        layout.addWidget(btn_clear)
    
    def add_notification(self, message, status="info"):
        """Add a notification"""
        notif_widget = QFrame()
        notif_widget.setStyleSheet("""
            QFrame {
                background-color: #f9f9f9;
                border: 1px solid #e0e0e0;
                border-radius: 4px;
                padding: 8px;
                margin: 2px;
            }
        """)
        
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
        time_label.setStyleSheet("color: #999; border: none; background: transparent;")
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
            "summary": "Replays the rosbag in Docker, runs FAST-LIO SLAM, writes scans.pcd, exports JPG frames from /image/compressed, and samples both tf_camera_out.csv and tf_gps_out.csv using message timestamps.",
            "run_tooltip": "Run the full rosbag preprocessing stage: FAST-LIO SLAM, JPG export, image timestamp CSV export, and camera/GPS TF sampling.",
            "view_tooltip": "Open the semantic viewer for this scan using the latest outputs generated so far.",
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
            "summary": "Projects segmentation masks into the SLAM point cloud, applies dense-cluster cleanup and instance clustering, measures pole spacing, and georeferences the outputs when tf_gps_out.csv exists. This stage consumes calibration outputs produced by direct visual LiDAR calibration, including camera intrinsics derived from /camera/camera_info.",
            "run_tooltip": "Run camera-LiDAR fusion and the GPS georeferencing/export stage for this scan.",
            "view_tooltip": "Open the semantic viewer with fused objects and wire overlays.",
            "modify_label": "Calibration",
            "modify_tooltip": "Link a completed calibration run and set the GPS lever-arm used by Fusion.",
        },
    }
    
    def __init__(self, assets_path="ENGR-498-Project/assets", parent=None):
        super().__init__(parent)
        self.assets_path = Path(assets_path)
        self.selected_scan = None
        self.scans_data = {}
        self.modified_scans = set()  # Track which scans have unsaved changes
        
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

        # Pipeline overview
        overview = self._create_pipeline_overview()
        main_layout.addWidget(overview)
        
        # Scan table
        self.table = self._create_scan_table()
        main_layout.addWidget(self.table, stretch=1)
        
        # Status bar
        self.status_label = QLabel("Post-Processing Step-By-Step: Rosbag Preprocessing -> Wires -> Image Inference -> Fusion + GPS, with manual Filtering and FLAI hooks.")
        self.status_label.setStyleSheet("color: #666; padding: 8px;")
        main_layout.addWidget(self.status_label)
        
        # Apply global styles
        self.setStyleSheet("""
            QWidget {
                background-color: #f6f7fb;
                color: #1f2937;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QLabel {
                color: #1f2937;
                background: transparent;
            }
            QCheckBox {
                color: #1f2937;
                spacing: 6px;
            }
            QTableWidget {
                background-color: white;
                color: #1f2937;
                border: 1px solid #ddd;
                border-radius: 8px;
                gridline-color: #e0e0e0;
                alternate-background-color: #f9fbff;
            }
            QTableWidget::item {
                padding: 8px;
                color: #1f2937;
            }
            QTableWidget::item:selected {
                background-color: #dbeafe;
                color: #111827;
            }
            QHeaderView::section {
                background-color: #f5f5f5;
                color: #111827;
                padding: 10px;
                border: none;
                border-bottom: 2px solid #673AB7;
                font-weight: bold;
            }
        """)
    
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
        title.setStyleSheet("color: #673AB7;")
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
        bar.setStyleSheet("""
            QFrame {
                background-color: white;
                border: 1px solid #ddd;
                border-radius: 8px;
                padding: 12px;
            }
        """)
        layout = QHBoxLayout()
        bar.setLayout(layout)
        
        # Create New Scan button
        btn_new_scan = QPushButton("➕ Upload New Scan")
        btn_new_scan.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 10px 20px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        btn_new_scan.clicked.connect(self.createScanRequested.emit)
        layout.addWidget(btn_new_scan)

        self.btn_run_pipeline = QPushButton("Run Full Pipeline")
        self.btn_run_pipeline.setEnabled(False)
        self.btn_run_pipeline.setStyleSheet("""
            QPushButton {
                background-color: #673AB7;
                color: white;
                border: none;
                padding: 10px 18px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #5E35B1;
            }
            QPushButton:disabled {
                background-color: #D1C4E9;
                color: white;
            }
        """)
        self.btn_run_pipeline.setToolTip("Run the automated backend chain for the selected scan: Rosbag Preprocessing -> Wires -> Image Inference -> Fusion + GPS.")
        self.btn_run_pipeline.clicked.connect(self._run_full_pipeline)
        layout.addWidget(self.btn_run_pipeline)
        
        layout.addStretch()
        
        # Switch to Auto Mode button
        btn_auto_mode = QPushButton("⚡ Auto Mode")
        btn_auto_mode.setToolTip("Switch to automatic pipeline processing")
        btn_auto_mode.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 10px 20px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        btn_auto_mode.clicked.connect(self.switchToAutoModeRequested.emit)
        layout.addWidget(btn_auto_mode)
        
        # Refresh button
        btn_refresh = QPushButton("🔄")
        btn_refresh.setToolTip("Refresh scans")
        btn_refresh.setStyleSheet("""
            QPushButton {
                background-color: #f5f5f5;
                border: 1px solid #ddd;
                padding: 8px 12px;
                border-radius: 4px;
                font-size: 16px;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
        """)
        btn_refresh.clicked.connect(self.refresh_scans)
        layout.addWidget(btn_refresh)
        
        return bar

    def _create_pipeline_overview(self):
        """Create a short in-context explanation of what each backend stage does."""
        panel = QFrame()
        panel.setStyleSheet(
            "QFrame { background-color: #ffffff; border: 1px solid #d8dee9; border-radius: 8px; padding: 10px; }"
        )
        layout = QVBoxLayout()
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)
        panel.setLayout(layout)

        title = QLabel("Backend Stage Summary")
        title.setStyleSheet("font-size: 15px; font-weight: bold; color: #1f3a5f;")
        layout.addWidget(title)

        summary = QLabel(
            "<b>Rosbag Preprocessing</b> replays the rosbag, runs FAST-LIO SLAM, writes <code>scans.pcd</code>, "
            "exports JPGs from <code>/image/compressed</code>, writes <code>image_timestamps.csv</code>, and samples "
            "both <code>tf_camera_out.csv</code> and <code>tf_gps_out.csv</code>. <b>Image Inference</b> runs YOLO "
            "on those JPGs and writes <code>masks_npz/</code> plus <code>meta_json/</code>. <b>Fusion + GPS</b> "
            "projects the masks into the SLAM cloud, keeps dense clusters, segments utility assets, measures pole "
            "spacing, and georeferences the results when GPS data is available. <b>Wires</b> remains the MATLAB wire "
            "extraction path, and calibration stays in its own separate GUI mode where camera intrinsics are extracted "
            "from <code>/camera/camera_info</code> and the LiDAR-camera extrinsic is solved."
        )
        summary.setWordWrap(True)
        summary.setStyleSheet("color: #374151; line-height: 1.35;")
        layout.addWidget(summary)

        legend = QLabel(
            "Use <b>Run</b> to execute a post-processing stage, <b>View</b> to inspect the current result layer, and the right-most "
            "button to open that stage's actual details, guide, parameters, or configuration. Calibration is intentionally separate from this table."
        )
        legend.setWordWrap(True)
        legend.setStyleSheet("color: #4b5563;")
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
                item.setToolTip("Open the combined semantic viewer, run the full backend chain, open the Leaflet map, or delete the scan.")
            else:
                item.setToolTip("The scan directory under assets/ containing raw bags, processed outputs, and metadata.")
        
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setSelectionMode(QTableWidget.SingleSelection)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)
        
        # Fix row height
        table.verticalHeader().setDefaultSectionSize(70)
        
        # Column sizing
        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)  # Scan Name
        for i in range(1, len(columns) - 1):  # Step columns
            header.setSectionResizeMode(i, QHeaderView.Fixed)
            table.setColumnWidth(i, 205)
        header.setSectionResizeMode(len(columns) - 1, QHeaderView.Fixed)  # Actions
        table.setColumnWidth(len(columns) - 1, 300)
        
        table.setMinimumHeight(300)
        table.itemSelectionChanged.connect(self._on_selection_changed)
        
        return table
    
    def _setup_refresh_timer(self):
        """Setup timer to auto-refresh table"""
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_scans)
        self.refresh_timer.start(3000)
    
    def refresh_scans(self):
        """Refresh scan data"""
        if not self.assets_path.exists():
            self.assets_path.mkdir(parents=True, exist_ok=True)
            print(f"Created assets directory at {self.assets_path.resolve()}")
            return

        self.scans_data = {}
        
        for scan_dir in self.assets_path.iterdir():
            if not scan_dir.is_dir():
                continue

            try:
                _, metadata = load_scan_metadata(scan_dir)
                self.scans_data[scan_dir.name] = metadata
            except Exception as e:
                print(f"Error loading metadata for {scan_dir.name}: {e}")
        
        self._update_table()
    
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
        layout.setSpacing(4)
        widget.setLayout(layout)
        widget.setToolTip(step_info["summary"])

        # Checkbox for completion
        checkbox = QCheckBox("Complete")
        checkbox.setChecked(status_dict.get(step_key) == "done")
        checkbox.setToolTip(f"Mark {step_info['title']} complete in metadata. This does not run the stage by itself.")
        checkbox.stateChanged.connect(lambda state, sn=scan_name, sk=step_key: self._on_checkbox_changed(sn, sk, state))
        layout.addWidget(checkbox)

        # Buttons layout
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(4)

        btn_run = QPushButton("Run")
        btn_run.setStyleSheet("""
            QPushButton {
                background-color: #673AB7;
                color: white;
                border: none;
                padding: 4px 8px;
                border-radius: 3px;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #5E35B1;
            }
        """)
        btn_run.setToolTip(step_info["run_tooltip"])
        btn_run.clicked.connect(
            lambda checked=False, sp=str(self.assets_path / scan_name), sk=step_key: self.runStepRequested.emit(sp, sk)
        )
        btn_layout.addWidget(btn_run)
        
        # View button
        btn_view = QPushButton("View")
        btn_view.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                padding: 4px 8px;
                border-radius: 3px;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #0b7dda;
            }
        """)
        btn_view.setToolTip(step_info["view_tooltip"])
        btn_view.clicked.connect(lambda: self._on_view_step(scan_name, step_key))
        btn_layout.addWidget(btn_view)

        # Context button
        btn_modify = QPushButton(step_info["modify_label"])
        btn_modify.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                border: none;
                padding: 4px 8px;
                border-radius: 3px;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #F57C00;
            }
        """)
        btn_modify.setToolTip(step_info["modify_tooltip"])
        btn_modify.clicked.connect(lambda: self._on_modify_step(scan_name, step_key))
        btn_layout.addWidget(btn_modify)
        
        layout.addLayout(btn_layout)
        
        return widget
    
    def _create_actions_widget(self, scan_name, metadata):
        """Create actions column"""
        widget = QWidget()
        layout = QHBoxLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        widget.setLayout(layout)
        
        # View Result button
        btn_view = QPushButton("View Result")
        btn_view.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 6px 12px;
                border-radius: 4px;
                font-size: 11px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        btn_view.clicked.connect(lambda: self.openViewerRequested.emit(str(self.assets_path / scan_name)))
        btn_view.setToolTip("Open the combined semantic viewer for this scan.")
        layout.addWidget(btn_view)

        btn_run_full = QPushButton("Run Full")
        btn_run_full.setStyleSheet("""
            QPushButton {
                background-color: #673AB7;
                color: white;
                border: none;
                padding: 6px 10px;
                border-radius: 4px;
                font-size: 11px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #5E35B1;
            }
        """)
        btn_run_full.clicked.connect(lambda: self.runPipelineRequested.emit(str(self.assets_path / scan_name)))
        btn_run_full.setToolTip("Run the automated backend chain for this scan.")
        layout.addWidget(btn_run_full)

        btn_map = QPushButton("Map")
        btn_map.setStyleSheet("""
            QPushButton {
                background-color: #1976D2;
                color: white;
                border: none;
                padding: 6px 10px;
                border-radius: 4px;
                font-size: 11px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #125A9C;
            }
        """)
        btn_map.clicked.connect(lambda: self.openMapRequested.emit(str(self.assets_path / scan_name)))
        btn_map.setToolTip("Open the Leaflet map for this scan if fusion objects and/or powerline overlays exist.")
        layout.addWidget(btn_map)
        
        # Save button (only visible if modified)
        btn_save = QPushButton("💾 Save")
        btn_save.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                border: none;
                padding: 6px 12px;
                border-radius: 4px;
                font-size: 11px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #7B1FA2;
            }
        """)
        btn_save.clicked.connect(lambda: self._save_scan(scan_name))
        btn_save.setVisible(scan_name in self.modified_scans)
        btn_save.setObjectName(f"save_{scan_name}")  # For finding later
        layout.addWidget(btn_save)
        
        # Delete button
        btn_delete = QPushButton("🗑")
        btn_delete.setToolTip("Delete scan")
        btn_delete.setStyleSheet("""
            QPushButton {
                background-color: #f5f5f5;
                border: 1px solid #ddd;
                padding: 6px 10px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #ffebee;
                border-color: #F44336;
            }
        """)
        btn_delete.clicked.connect(lambda: self._delete_scan(scan_name))
        layout.addWidget(btn_delete)
        
        return widget
    
    def _on_checkbox_changed(self, scan_name, step_key, state):
        """Handle checkbox state change"""
        new_status = "done" if state == Qt.Checked else "pending"
        
        # Update in-memory data
        if scan_name in self.scans_data:
            if "status" not in self.scans_data[scan_name]:
                self.scans_data[scan_name]["status"] = {}
            self.scans_data[scan_name]["status"][step_key] = new_status
            
            # Mark as modified
            self.modified_scans.add(scan_name)
            
            # Show save button
            self._show_save_button(scan_name)
            
            print(f"Changed {scan_name} -> {step_key}: {new_status}")
    
    def _show_save_button(self, scan_name):
        """Show the save button for a scan"""
        # Find and show the save button
        for row in range(self.table.rowCount()):
            item = self.table.item(row, 0)
            if item and item.data(Qt.UserRole) == scan_name:
                actions_widget = self.table.cellWidget(row, len(self.STEP_COLUMNS) + 1)
                if actions_widget:
                    save_btn = actions_widget.findChild(QPushButton, f"save_{scan_name}")
                    if save_btn:
                        save_btn.setVisible(True)
                break
    
    def _save_scan(self, scan_name):
        """Save scan metadata"""
        if scan_name not in self.scans_data:
            return

        try:
            save_scan_metadata(self.assets_path / scan_name, self.scans_data[scan_name])
            self.modified_scans.discard(scan_name)
            self.add_notification(f"Saved changes for {scan_name}", "done")
            self.refresh_scans()
            print(f"✓ Saved metadata for {scan_name}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save: {e}")
    
    def _on_view_step(self, scan_name, step_key):
        """Handle view button click"""
        title = self.STEP_DETAILS[step_key]["title"]
        print(f"VIEW clicked: {scan_name} -> {step_key}")
        self.add_notification(f"View requested: {scan_name} - {title}", "info")
        if step_key == "filtering":
            scan_dir = self.assets_path / scan_name
            metadata = self.scans_data.get(scan_name, {})
            files_dict = metadata.get("files", {})
            for key in ("filtered", "las", "pcd"):
                filepath = resolve_scan_path(scan_dir, files_dict.get(key))
                if filepath is not None and filepath.exists():
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
                
                # Update metadata
                if scan_name in self.scans_data:
                    self.scans_data[scan_name]["wire_params"] = params
                    self.modified_scans.add(scan_name)
                    self._show_save_button(scan_name)
                
                self.add_notification(f"Modified wire parameters for {scan_name}", "info")
        elif step_key == "filtering":
            print(f"OPEN FILTER clicked: {scan_name} -> {step_key}")
            if scan_name in self.scans_data:
                files_dict = metadata.get("files", {})
                filepath = files_dict.get("filtered") or files_dict.get("las") or files_dict.get("pcd") or ""
                print(f"Opening filter viewer for {scan_name} with file: {filepath}")
                if filepath:
                    resolved = resolve_scan_path(scan_dir, filepath)
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
                self.modified_scans.add(scan_name)
                self._show_save_button(scan_name)
                self.add_notification(f"Updated inference runtime settings for {scan_name}", "info")
        elif step_key == "fusion":
            current_config = metadata.get("config", {})
            dialog = FusionParametersDialog(
                fusion_config=current_config.get("fusion", {}),
                gps_config=current_config.get("gps", {}),
                scan_dir=scan_dir,
                parent=self,
            )
            if dialog.exec() == QDialog.Accepted:
                fusion_config, gps_config = dialog.get_configs()
                metadata.setdefault("config", {})["fusion"] = fusion_config
                metadata.setdefault("config", {})["gps"] = gps_config
                self.modified_scans.add(scan_name)
                self._show_save_button(scan_name)
                self.add_notification(f"Updated calibration/GPS settings for {scan_name}", "info")
        elif step_key == "slam":
            StepInfoDialog(
                "Rosbag Preprocessing Outputs",
                (
                    "### What this stage does\n"
                    "- Replays the rosbag inside the Docker preprocessing environment.\n"
                    "- Runs FAST-LIO SLAM to produce the local map point cloud.\n"
                    "- Saves every `/image/compressed` frame as a JPG under the pose-recovery run folder.\n"
                    "- Writes `image_timestamps.csv`, `tf_camera_out.csv`, and `tf_gps_out.csv`.\n\n"
                    "### What this stage does not do\n"
                    "- It does not compute camera intrinsics or LiDAR-camera extrinsics.\n"
                    "- Those come from the separate Calibration Mode, where intrinsics are extracted from `/camera/camera_info` and direct visual LiDAR calibration solves the extrinsic transform.\n\n"
                    "### Main outputs\n"
                    "- `processed/pose_recovery/<run>/pcd/scans.pcd`\n"
                    "- `processed/pose_recovery/<run>/images/*.jpg`\n"
                    "- `processed/pose_recovery/<run>/image_timestamps.csv`\n"
                    "- `processed/pose_recovery/<run>/tf_camera_out.csv`\n"
                    "- `processed/pose_recovery/<run>/tf_gps_out.csv`\n\n"
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
                self.modified_scans.discard(scan_name)
                self.refresh_scans()
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
        "files": "assets/scan_001/processed/slam/LAW1.las",
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
