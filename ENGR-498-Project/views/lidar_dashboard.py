"""
LiDAR Processing Dashboard (Auto Mode)
A modern control center for managing multiple scans and running processing pipelines.
"""

import json
import os
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QTableWidget, QTableWidgetItem, QHeaderView, QComboBox,
    QFrame, QScrollArea, QMenu, QFileDialog, QMessageBox,
    QToolButton, QSizePolicy, QTextEdit, QCheckBox
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QColor, QFont, QIcon

from scan_metadata import load_scan_metadata, resolve_scan_path, save_scan_metadata
from timing_settings import load_global_timing_settings, save_global_timing_settings


class StatusIndicator(QWidget):
    """Visual status indicator with icon and color"""
    
    STATUS_STYLES = {
        "done": ("✔", "#4CAF50", "Complete"),
        "running": ("⏳", "#FF9800", "Running"),
        "pending": ("●", "#9E9E9E", "Not Started"),
        "error": ("❌", "#F44336", "Error"),
        "paused": ("⏸", "#2196F3", "Paused")
    }
    
    def __init__(self, status="pending", parent=None):
        super().__init__(parent)
        self.status = status
        self._setup_ui()
        
    def _setup_ui(self):
        layout = QHBoxLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        self.setLayout(layout)
        
        icon, color, tooltip = self.STATUS_STYLES.get(self.status, ("?", "#000", "Unknown"))
        
        self.label = QLabel(icon)
        self.label.setStyleSheet(f"color: {color}; font-size: 16px; font-weight: bold;")
        self.label.setToolTip(tooltip)
        layout.addWidget(self.label, alignment=Qt.AlignCenter)
        
    def set_status(self, status):
        """Update the status indicator"""
        self.status = status
        icon, color, tooltip = self.STATUS_STYLES.get(status, ("?", "#000", "Unknown"))
        self.label.setText(icon)
        self.label.setStyleSheet(f"color: {color}; font-size: 16px; font-weight: bold;")
        self.label.setToolTip(tooltip)


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
        
        # Style
        self.setStyleSheet("""
            QWidget {
                background-color: white;
                border: 1px solid #ddd;
                border-radius: 8px;
            }
        """)
        
        # Header
        header = QLabel("Notifications")
        header.setStyleSheet("font-size: 16px; font-weight: bold; border: none;")
        layout.addWidget(header)
        
        # Scroll area for notifications
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("border: none;")
        self.notification_container = QWidget()
        self.notification_layout = QVBoxLayout()
        self.notification_layout.setContentsMargins(0, 0, 0, 0)
        self.notification_container.setLayout(self.notification_layout)
        scroll.setWidget(self.notification_container)
        layout.addWidget(scroll)
        
        # Clear button
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
        
        # Status icon
        status_colors = {
            "done": "✔ 🟢",
            "running": "⏳ 🟡",
            "error": "❌ 🔴",
            "info": "ℹ️"
        }
        icon = QLabel(status_colors.get(status, "ℹ️"))
        notif_layout.addWidget(icon)
        
        # Message
        msg_label = QLabel(message)
        msg_label.setWordWrap(True)
        msg_label.setStyleSheet("border: none; background: transparent;")
        notif_layout.addWidget(msg_label, stretch=1)
        
        # Timestamp
        time_label = QLabel(datetime.now().strftime("%H:%M"))
        time_label.setStyleSheet("color: #999; border: none; background: transparent;")
        notif_layout.addWidget(time_label)
        
        self.notification_layout.insertWidget(0, notif_widget)
        self.notifications.append(notif_widget)
        
        # Limit to 50 notifications
        if len(self.notifications) > 50:
            old = self.notifications.pop()
            old.deleteLater()
    
    def clear_notifications(self):
        """Clear all notifications"""
        for notif in self.notifications:
            notif.deleteLater()
        self.notifications.clear()


class DashboardView(QWidget):
    """Main dashboard control center (Auto Mode)"""
    
    # Signals
    openViewerRequested = Signal(str)  # scan_path
    openMapRequested = Signal(str)  # scan_path
    runPipelineRequested = Signal(str)  # scan_path
    openSlamPointCloudRequested = Signal(str)  # point_cloud_path
    openImagesFolderRequested = Signal(str)  # images_dir_path
    createScanRequested = Signal()
    switchToStepModeRequested = Signal()  # NEW: Switch to step-by-step mode
    
    STEP_COLUMNS = {
        "Rosbag Preprocessing": "slam",
        "Filtering": "filtering",
        "FLAI": "flai",
        "Wires": "wire_extraction",
        "Image Inference": "inference",
        "Fusion + GPS": "fusion"
    }

    STEP_TOOLTIPS = {
        "slam": "Runs rosbag preprocessing: FAST-LIO SLAM, JPG export from /image/compressed, dense /tf sampling, and GPS TF sampling when available.",
        "filtering": "Optional manual point-cloud cleanup stage.",
        "flai": "External/manual segmentation stage if used.",
        "wire_extraction": "MATLAB wire extraction outputs for wires_points.npz, wire_info.json, and ground points.",
        "inference": "YOLO segmentation on the JPG images exported during Rosbag Preprocessing. The GUI defaults to the local NVIDIA GPU and bundled repo weights when available.",
        "fusion": "Mask projection, dense-cluster cleanup, instance clustering, pole spacing, and GPS georeferencing when usable GPS samples exist. Developer mode can skip georeferencing for no-GPS bags.",
    }
    
    def __init__(self, assets_path="assets", parent=None):
        super().__init__(parent)
        self.assets_path = Path(assets_path)
        self.selected_scan = None
        self.scans_data = {}  # {scan_name: metadata}
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
        self.status_label = QLabel("Post-Processing Auto Mode runs Rosbag Preprocessing, Wires, Image Inference, and Fusion + GPS as one backend chain. Calibration is a separate mode.")
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
                border-bottom: 2px solid #2196F3;
                font-weight: bold;
            }
        """)
    
    def _create_top_bar(self):
        """Create top bar with title and notifications"""
        bar = QWidget()
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        bar.setLayout(layout)
        
        # Title
        title = QLabel("LiDAR Processing Dashboard")
        title_font = QFont()
        title_font.setPointSize(20)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setStyleSheet("color: #1976D2;")
        layout.addWidget(title)
        
        # Mode badge
        mode_badge = QLabel("AUTO MODE")
        mode_badge.setStyleSheet("""
            QLabel {
                background-color: #4CAF50;
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
        """Create controls bar with buttons"""
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
        btn_new_scan = QPushButton("➕ Create New Scan")
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
        
        # Run Full Pipeline button
        self.btn_run_pipeline = QPushButton("▶️ Run Full Pipeline")
        self.btn_run_pipeline.setEnabled(False)
        self.btn_run_pipeline.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                padding: 10px 20px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover:enabled {
                background-color: #0b7dda;
            }
            QPushButton:disabled {
                background-color: #bbb;
            }
        """)
        self.btn_run_pipeline.setToolTip("Run the automated backend chain for the selected scan: Rosbag Preprocessing -> Wires -> Image Inference -> Fusion + GPS.")
        self.btn_run_pipeline.clicked.connect(self._run_full_pipeline)
        layout.addWidget(self.btn_run_pipeline)

        self.rviz_checkbox = QCheckBox("Show RViz during Rosbag Preprocessing")
        self.rviz_checkbox.setEnabled(False)
        self.rviz_checkbox.setToolTip(
            "If enabled, Rosbag Preprocessing launches RViz so you can watch FAST-LIO build the point cloud in real time."
        )
        self.rviz_checkbox.stateChanged.connect(self._on_rviz_checkbox_changed)
        layout.addWidget(self.rviz_checkbox)

        btn_global_settings = QPushButton("Global Settings")
        btn_global_settings.setToolTip(
            "Set global timing defaults, the GPS developer-mode override, and the Fusion visualization default."
        )
        btn_global_settings.setStyleSheet("""
            QPushButton {
                background-color: #0F766E;
                color: white;
                border: none;
                padding: 10px 18px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #115E59;
            }
        """)
        btn_global_settings.clicked.connect(self._open_global_settings_dialog)
        layout.addWidget(btn_global_settings)
        
        layout.addStretch()
        
        # Switch to Step-by-Step Mode button
        btn_step_mode = QPushButton("🔧 Step-by-Step Mode")
        btn_step_mode.setToolTip("Switch to advanced step-by-step processing mode")
        btn_step_mode.setStyleSheet("""
            QPushButton {
                background-color: #673AB7;
                color: white;
                border: none;
                padding: 10px 20px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #5E35B1;
            }
        """)
        btn_step_mode.clicked.connect(self.switchToStepModeRequested.emit)
        layout.addWidget(btn_step_mode)
        
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
        btn_refresh.clicked.connect(lambda: self.refresh_scans(force=True))
        layout.addWidget(btn_refresh)
        
        return bar
    
    def _create_scan_table(self):
        """Create the main scan table"""
        table = QTableWidget()
        
        # Configure columns
        columns = ["Scan Name"] + list(self.STEP_COLUMNS.keys()) + ["Status", "Actions"]
        table.setColumnCount(len(columns))
        table.setHorizontalHeaderLabels(columns)

        for idx, header_text in enumerate(columns):
            item = table.horizontalHeaderItem(idx)
            if item is None:
                continue
            if header_text in self.STEP_COLUMNS:
                item.setToolTip(self.STEP_TOOLTIPS[self.STEP_COLUMNS[header_text]])
            elif header_text == "Status":
                item.setToolTip("Overall scan state derived from the stage statuses shown in this row.")
            elif header_text == "Actions":
                item.setToolTip("Run the full chain, open semantic/map views, open the latest SLAM point cloud, open exported images, or delete the scan.")
            else:
                item.setToolTip("The scan directory under assets/ containing raw bags, processed outputs, and metadata.")
        
        # Configure table properties
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
        for i in range(1, len(columns) - 2):  # Step columns
            header.setSectionResizeMode(i, QHeaderView.Fixed)
            table.setColumnWidth(i, 135)
        header.setSectionResizeMode(len(columns) - 2, QHeaderView.Fixed)  # Status
        table.setColumnWidth(len(columns) - 2, 150)
        header.setSectionResizeMode(len(columns) - 1, QHeaderView.Fixed)  # Actions
        table.setColumnWidth(len(columns) - 1, 430)
        
        table.setMinimumHeight(300)
        table.itemSelectionChanged.connect(self._on_selection_changed)
        
        return table

    def _create_log_panel(self) -> QTextEdit:
        log_output = QTextEdit()
        log_output.setReadOnly(True)
        log_output.setMinimumHeight(170)
        log_output.setPlaceholderText("Backend console output will appear here during rosbag preprocessing, wire extraction, inference, and fusion.")
        log_output.setStyleSheet(
            "QTextEdit { background-color: #0f172a; color: #e5e7eb; border: 1px solid #1f2937; "
            "border-radius: 8px; padding: 8px; font-family: Consolas, 'Courier New', monospace; font-size: 11px; }"
        )
        return log_output
    
    def _setup_refresh_timer(self):
        """Setup timer to auto-refresh table"""
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_scans)
        self.refresh_timer.start(3000)  # Refresh every 3 seconds

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

    def refresh_scans(self, force: bool = False):
        """Refresh scan data from assets directory"""
        if not self.assets_path.exists():
            self.assets_path.mkdir(parents=True, exist_ok=True)
            return

        snapshot = self._collect_scan_snapshot()
        if not force and snapshot == self._scan_snapshot:
            return

        self._scan_snapshot = snapshot
        selected_scan = self.selected_scan
        self.scans_data = {}
        
        # Load all scan metadata
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
        self._sync_pose_recovery_checkbox()

    def select_scan(self, scan_name: str) -> None:
        for row in range(self.table.rowCount()):
            item = self.table.item(row, 0)
            if item is None:
                continue
            if item.data(Qt.UserRole) == scan_name:
                self.table.setCurrentCell(row, 0)
                self.table.scrollToItem(item, QTableWidget.PositionAtCenter)
                self._sync_pose_recovery_checkbox()
                return
    
    def _update_table(self):
        """Update table with current scan data"""
        self.table.setRowCount(len(self.scans_data))
        
        for row, (scan_name, metadata) in enumerate(sorted(self.scans_data.items())):
            # Scan Name
            name_item = QTableWidgetItem(scan_name)
            name_item.setData(Qt.UserRole, scan_name)
            self.table.setItem(row, 0, name_item)
            
            # Step columns
            status_dict = metadata.get("status", {})
            
            for col_idx, (col_name, step_key) in enumerate(self.STEP_COLUMNS.items(), start=1):
                step_status = status_dict.get(step_key, "pending")
                
                # Status indicator
                indicator = StatusIndicator(step_status)
                indicator.setToolTip(self.STEP_TOOLTIPS[step_key])
                self.table.setCellWidget(row, col_idx, indicator)
            
            # Overall Status column
            overall_status = self._get_overall_status(metadata)
            status_widget = self._create_status_widget(overall_status, metadata, scan_name)
            self.table.setCellWidget(row, len(self.STEP_COLUMNS) + 1, status_widget)
            
            # Actions column
            actions_widget = self._create_actions_widget(metadata, scan_name)
            self.table.setCellWidget(row, len(self.STEP_COLUMNS) + 2, actions_widget)
    
    def _get_overall_status(self, metadata):
        """Determine overall scan status"""
        status_dict = metadata.get("status", {})
        required_steps = ("slam", "wire_extraction", "inference", "fusion")
        
        if any(s == "running" for s in status_dict.values()):
            return "processing"
        elif all(status_dict.get(step) == "done" for step in required_steps):
            return "complete"
        elif any(s == "error" for s in status_dict.values()):
            return "error"
        else:
            return "pending"
    
    def _create_status_widget(self, status, metadata, scan_name):
        """Create status column widget"""
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        widget.setLayout(layout)
        
        # Status label
        status_labels = {
            "processing": ("⏳ Processing", "#FF9800"),
            "complete": ("✔ Complete", "#4CAF50"),
            "error": ("❌ Error", "#F44336"),
            "pending": ("● Pending", "#9E9E9E")
        }
        
        text, color = status_labels.get(status, ("Unknown", "#000"))
        label = QLabel(text)
        label.setStyleSheet(f"color: {color}; font-weight: bold;")
        layout.addWidget(label)
        
        return widget
    
    def _create_actions_widget(self, metadata, scan_name):
        """Create actions column widget"""
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
        
        # View Result button (if any step is complete)
        if any(s == "done" for s in status_dict.values()):
            btn_view = QPushButton("View Result")
            btn_view.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    padding: 6px 12px;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
            """)
            scan_path = str(self.assets_path / scan_name)
            btn_view.clicked.connect(lambda checked=False, sp=scan_path: self.openViewerRequested.emit(sp))
            layout.addWidget(btn_view)

            btn_map = QPushButton("Map")
            btn_map.setStyleSheet("""
                QPushButton {
                    background-color: #1976D2;
                    color: white;
                    border: none;
                    padding: 6px 10px;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #125A9C;
                }
            """)
            btn_map.clicked.connect(lambda checked=False, sp=scan_path: self.openMapRequested.emit(sp))
            layout.addWidget(btn_map)

        btn_pcd = QPushButton("PCD")
        btn_pcd.setStyleSheet("""
            QPushButton {
                background-color: #2563EB;
                color: white;
                border: none;
                padding: 6px 10px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover:enabled {
                background-color: #1D4ED8;
            }
            QPushButton:disabled {
                background-color: #CBD5E1;
                color: #64748B;
            }
        """)
        btn_pcd.setEnabled(pcd_ready)
        btn_pcd.setToolTip(
            "Open the latest SLAM point cloud in the Open3D viewer."
            if pcd_ready
            else "Run Rosbag Preprocessing first to generate scans.pcd."
        )
        if pcd_ready:
            btn_pcd.clicked.connect(lambda checked=False, fp=str(pcd_path): self.openSlamPointCloudRequested.emit(fp))
        layout.addWidget(btn_pcd)

        btn_images = QPushButton("Images")
        btn_images.setStyleSheet("""
            QPushButton {
                background-color: #0F766E;
                color: white;
                border: none;
                padding: 6px 10px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover:enabled {
                background-color: #115E59;
            }
            QPushButton:disabled {
                background-color: #CBD5E1;
                color: #64748B;
            }
        """)
        btn_images.setEnabled(images_ready)
        btn_images.setToolTip(
            "Open the folder containing exported JPG frames."
            if images_ready
            else "Run Rosbag Preprocessing first to export the image frames."
        )
        if images_ready:
            btn_images.clicked.connect(lambda checked=False, fp=str(images_dir): self.openImagesFolderRequested.emit(fp))
        layout.addWidget(btn_images)
        
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
        
        layout.addStretch()
        
        return widget
    
    def _get_file_key(self, step_key):
        """Map step key to file key"""
        mapping = {
            "slam": "pcd",
            "filtering": "filtered_las",
            "flai": "segmented",
            "inference": "masks_dir",
            "fusion": "fused",
            "wire_extraction": "wires"
        }
        return mapping.get(step_key)
    
    def _on_selection_changed(self):
        """Handle table selection change"""
        selected_items = self.table.selectedItems()
        if selected_items:
            self.selected_scan = selected_items[0].data(Qt.UserRole)
            self.btn_run_pipeline.setEnabled(True)
            self.status_label.setText(f"Selected: {self.selected_scan}")
        else:
            self.selected_scan = None
            self.btn_run_pipeline.setEnabled(False)
            self.status_label.setText("Post-Processing Auto Mode runs Rosbag Preprocessing, Wires, Image Inference, and Fusion + GPS as one backend chain. Calibration is a separate mode.")
        self._sync_pose_recovery_checkbox()

    def _sync_pose_recovery_checkbox(self):
        self.rviz_checkbox.blockSignals(True)
        if not self.selected_scan or self.selected_scan not in self.scans_data:
            self.rviz_checkbox.setChecked(False)
            self.rviz_checkbox.setEnabled(False)
            self.rviz_checkbox.blockSignals(False)
            return

        metadata = self.scans_data[self.selected_scan]
        enabled = bool(metadata.get("config", {}).get("pose_recovery", {}).get("enable_rviz", False))
        self.rviz_checkbox.setEnabled(True)
        self.rviz_checkbox.setChecked(enabled)
        self.rviz_checkbox.blockSignals(False)

    @staticmethod
    def _is_checked_state(state) -> bool:
        if isinstance(state, bool):
            return state
        try:
            return int(state) == int(Qt.CheckState.Checked.value)
        except Exception:
            return False

    def _on_rviz_checkbox_changed(self, state: int):
        if not self.selected_scan:
            return

        scan_dir, metadata = load_scan_metadata(self.assets_path / self.selected_scan)
        is_checked = self._is_checked_state(state)
        metadata.setdefault("config", {}).setdefault("pose_recovery", {})["enable_rviz"] = is_checked
        save_scan_metadata(scan_dir, metadata)
        self.scans_data[self.selected_scan] = metadata
        self._scan_snapshot = self._collect_scan_snapshot()
        state_text = "enabled" if is_checked else "disabled"
        self.add_notification(f"RViz preview {state_text} for {self.selected_scan}", "info")
        self.status_label.setText(f"Rosbag Preprocessing RViz preview {state_text} for {self.selected_scan}")
    
    def _run_full_pipeline(self):
        """Run full pipeline for selected scan"""
        if self.selected_scan:
            scan_path = str(self.assets_path / self.selected_scan)
            self.runPipelineRequested.emit(scan_path)
            self.add_notification(
                f"Started backend chain for {self.selected_scan}: Rosbag Preprocessing -> Wires -> Image Inference -> Fusion + GPS",
                "running",
            )

    def _open_global_settings_dialog(self):
        from views.lidar_dashboard_stepbystep import GlobalTimingSettingsDialog

        settings = load_global_timing_settings()
        dialog = GlobalTimingSettingsDialog(current_settings=settings, parent=self)
        if dialog.exec() == QDialog.Accepted:
            settings.update(dialog.get_settings())
            save_global_timing_settings(settings)
            self.add_notification("Updated global timing, GPS, and Fusion defaults", "done")
            self.status_label.setText("Updated global timing, GPS, and Fusion defaults.")
    
    def _delete_scan(self, scan_name):
        """Delete a scan after confirmation"""
        reply = QMessageBox.question(
            self,
            "Confirm Delete",
            f"Are you sure you want to delete scan '{scan_name}'?\nThis cannot be undone.",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            try:
                import shutil
                scan_path = self.assets_path / scan_name
                shutil.rmtree(scan_path)
                del self.scans_data[scan_name]
                self.refresh_scans()
                self.add_notification(f"Deleted {scan_name}", "info")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to delete scan: {e}")
    
    def _show_notifications(self):
        """Show notification panel"""
        # Position below the button
        button_pos = self.btn_notifications.mapToGlobal(self.btn_notifications.rect().bottomLeft())
        panel_x = button_pos.x() - self.notification_panel.width() + self.btn_notifications.width()
        panel_y = button_pos.y() + 5
        self.notification_panel.move(panel_x, panel_y)
        self.notification_panel.show()
    
    def add_notification(self, message, status="info"):
        """Add a notification to the panel"""
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
    
    # Create test directory structure
    assets_path = Path("assets")
    assets_path.mkdir(exist_ok=True)
    
    # Create sample scan
    scan1 = assets_path / "scan_001"
    scan1.mkdir(exist_ok=True)
    
    metadata = {
        "name": "scan_001",
        "created_at": "2026-03-17",
        "status": {
            "slam": "done",
            "filtering": "done",
            "flai": "running",
            "wire_extraction": "pending",
            "fusion": "pending"
        },
        "files": {
            "raw_pcd": "processed/point_clouds/scans.pcd",
            "raw_las": "processed/point_clouds/cloud.las",
            "filtered_pcd": "processed/point_clouds/scans_filtered.pcd",
            "filtered_las": "processed/point_clouds/cloud_filtered.las"
        },
        "params": {
            "voxel_size": 0.1,
            "outlier_neighbors": 20
        },
        "last_updated": datetime.now().isoformat()
    }
    
    with open(scan1 / "metadata.json", 'w') as f:
        json.dump(metadata, f, indent=2)
    
    # Create dashboard
    dashboard = DashboardView(assets_path="assets")
    
    # Connect signals for demo
    dashboard.openViewerRequested.connect(lambda path: print(f"Open viewer: {path}"))
    dashboard.runPipelineRequested.connect(lambda path: print(f"Run pipeline: {path}"))
    dashboard.createScanRequested.connect(lambda: print("Create new scan"))
    dashboard.switchToStepModeRequested.connect(lambda: print("Switch to step-by-step mode"))
    
    dashboard.resize(1400, 800)
    dashboard.show()
    
    sys.exit(app.exec())
    """Visual status indicator with icon and color"""
    
    STATUS_STYLES = {
        "done": ("✔", "#4CAF50", "Complete"),
        "running": ("⏳", "#FF9800", "Running"),
        "pending": ("●", "#9E9E9E", "Not Started"),
        "error": ("❌", "#F44336", "Error"),
        "paused": ("⏸", "#2196F3", "Paused")
    }
    
    def __init__(self, status="pending", parent=None):
        super().__init__(parent)
        self.status = status
        self._setup_ui()
        
    def _setup_ui(self):
        layout = QHBoxLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        self.setLayout(layout)
        
        icon, color, tooltip = self.STATUS_STYLES.get(self.status, ("?", "#000", "Unknown"))
        
        self.label = QLabel(icon)
        self.label.setStyleSheet(f"color: {color}; font-size: 16px; font-weight: bold;")
        self.label.setToolTip(tooltip)
        layout.addWidget(self.label, alignment=Qt.AlignCenter)
        
    def set_status(self, status):
        """Update the status indicator"""
        self.status = status
        icon, color, tooltip = self.STATUS_STYLES.get(status, ("?", "#000", "Unknown"))
        self.label.setText(icon)
        self.label.setStyleSheet(f"color: {color}; font-size: 16px; font-weight: bold;")
        self.label.setToolTip(tooltip)


class StepButton(QPushButton):
    """Context-aware button for pipeline steps"""
    
    viewRequested = Signal(str)  # file_path
    runRequested = Signal()
    
    def __init__(self, status="pending", file_path=None, parent=None):
        super().__init__(parent)
        self.status = status
        self.file_path = file_path
        self._update_button()
        self.clicked.connect(self._on_clicked)
        
    def _update_button(self):
        """Update button appearance based on status"""
        if self.status == "done":
            self.setText("View")
            self.setEnabled(True)
            self.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    padding: 6px 12px;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
            """)
        elif self.status == "running":
            self.setText("Running...")
            self.setEnabled(False)
            self.setStyleSheet("""
                QPushButton {
                    background-color: #FF9800;
                    color: white;
                    border: none;
                    padding: 6px 12px;
                    border-radius: 4px;
                }
            """)
        elif self.status == "error":
            self.setText("Retry")
            self.setEnabled(True)
            self.setStyleSheet("""
                QPushButton {
                    background-color: #F44336;
                    color: white;
                    border: none;
                    padding: 6px 12px;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #da190b;
                }
            """)
        else:  # pending
            self.setText("Run")
            self.setEnabled(True)
            self.setStyleSheet("""
                QPushButton {
                    background-color: #2196F3;
                    color: white;
                    border: none;
                    padding: 6px 12px;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #0b7dda;
                }
            """)
    
    def _on_clicked(self):
        """Handle button click"""
        if self.status == "done" and self.file_path:
            self.viewRequested.emit(self.file_path)
        else:
            self.runRequested.emit()
    
    def set_status(self, status, file_path=None):
        """Update button status"""
        self.status = status
        self.file_path = file_path
        self._update_button()


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
        
        # Style
        self.setStyleSheet("""
            QWidget {
                background-color: white;
                border: 1px solid #ddd;
                border-radius: 8px;
            }
        """)
        
        # Header
        header = QLabel("Notifications")
        header.setStyleSheet("font-size: 16px; font-weight: bold; border: none;")
        layout.addWidget(header)
        
        # Scroll area for notifications
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("border: none;")
        self.notification_container = QWidget()
        self.notification_layout = QVBoxLayout()
        self.notification_layout.setContentsMargins(0, 0, 0, 0)
        self.notification_container.setLayout(self.notification_layout)
        scroll.setWidget(self.notification_container)
        layout.addWidget(scroll)
        
        # Clear button
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
        
        # Status icon
        status_colors = {
            "done": "✔ 🟢",
            "running": "⏳ 🟡",
            "error": "❌ 🔴",
            "info": "ℹ️"
        }
        icon = QLabel(status_colors.get(status, "ℹ️"))
        notif_layout.addWidget(icon)
        
        # Message
        msg_label = QLabel(message)
        msg_label.setWordWrap(True)
        msg_label.setStyleSheet("border: none; background: transparent;")
        notif_layout.addWidget(msg_label, stretch=1)
        
        # Timestamp
        time_label = QLabel(datetime.now().strftime("%H:%M"))
        time_label.setStyleSheet("color: #999; border: none; background: transparent;")
        notif_layout.addWidget(time_label)
        
        self.notification_layout.insertWidget(0, notif_widget)
        self.notifications.append(notif_widget)
        
        # Limit to 50 notifications
        if len(self.notifications) > 50:
            old = self.notifications.pop()
            old.deleteLater()
    
    def clear_notifications(self):
        """Clear all notifications"""
        for notif in self.notifications:
            notif.deleteLater()
        self.notifications.clear()


# class DashboardView(QWidget):
#     """Main dashboard control center"""
    
#     # Signals
#     openViewerRequested = Signal(str)  # file_path
#     runPipelineRequested = Signal(str)  # scan_path
#     runStepRequested = Signal(str, str)  # scan_path, step_name
#     continuePipelineRequested = Signal(str)  # scan_path
#     createScanRequested = Signal()
    
#     STEP_COLUMNS = {
#         "SLAM": "slam",
#         "Filtering": "filtering",
#         "LAS": "las_conversion",
#         "FLAI": "flai",
#         "Fusion": "fusion",
#         "Wires": "wire_extraction"
#     }
    
#     def __init__(self, assets_path="assets", parent=None):
#         super().__init__(parent)
#         self.assets_path = Path(assets_path)
#         self.selected_scan = None
#         self.scans_data = {}  # {scan_name: metadata}
        
#         self._setup_ui()
#         self._setup_refresh_timer()
#         self.refresh_scans()
        
#     def _setup_ui(self):
#         """Setup the main UI layout"""
#         main_layout = QVBoxLayout()
#         main_layout.setContentsMargins(16, 16, 16, 16)
#         main_layout.setSpacing(12)
#         self.setLayout(main_layout)
        
#         # Top bar
#         top_bar = self._create_top_bar()
#         main_layout.addWidget(top_bar)
        
#         # Controls bar
#         controls_bar = self._create_controls_bar()
#         main_layout.addWidget(controls_bar)
        
#         # Scan table
#         self.table = self._create_scan_table()
#         main_layout.addWidget(self.table, stretch=1)
        
#         # Status bar
#         self.status_label = QLabel("Ready")
#         self.status_label.setStyleSheet("color: #666; padding: 8px;")
#         main_layout.addWidget(self.status_label)
        
#         # Apply global styles
#         self.setStyleSheet("""
#             QWidget {
#                 background-color: #fafafa;
#                 font-family: 'Segoe UI', Arial, sans-serif;
#             }
#             QTableWidget {
#                 background-color: white;
#                 border: 1px solid #ddd;
#                 border-radius: 8px;
#                 gridline-color: #e0e0e0;
#             }
#             QTableWidget::item {
#                 padding: 8px;
#             }
#             QTableWidget::item:selected {
#                 background-color: #e3f2fd;
#                 color: black;
#             }
#             QHeaderView::section {
#                 background-color: #f5f5f5;
#                 padding: 10px;
#                 border: none;
#                 border-bottom: 2px solid #2196F3;
#                 font-weight: bold;
#             }
#         """)
    
#     def _create_top_bar(self):
#         """Create top bar with title and notifications"""
#         bar = QWidget()
#         layout = QHBoxLayout()
#         layout.setContentsMargins(0, 0, 0, 0)
#         bar.setLayout(layout)
        
#         # Title
#         title = QLabel("LiDAR Processing Dashboard")
#         title_font = QFont()
#         title_font.setPointSize(20)
#         title_font.setBold(True)
#         title.setFont(title_font)
#         title.setStyleSheet("color: #1976D2;")
#         layout.addWidget(title)
        
#         layout.addStretch()
        
#         # Notification bell
#         self.notification_panel = NotificationPanel(self)
#         self.btn_notifications = QToolButton()
#         self.btn_notifications.setText("🔔")
#         self.btn_notifications.setStyleSheet("""
#             QToolButton {
#                 font-size: 24px;
#                 border: none;
#                 background: transparent;
#                 padding: 8px;
#             }
#             QToolButton:hover {
#                 background-color: #e0e0e0;
#                 border-radius: 4px;
#             }
#         """)
#         self.btn_notifications.clicked.connect(self._show_notifications)
#         layout.addWidget(self.btn_notifications)
        
#         return bar
    
#     def _create_controls_bar(self):
#         """Create controls bar with buttons and mode toggle"""
#         bar = QFrame()
#         bar.setStyleSheet("""
#             QFrame {
#                 background-color: white;
#                 border: 1px solid #ddd;
#                 border-radius: 8px;
#                 padding: 12px;
#             }
#         """)
#         layout = QHBoxLayout()
#         bar.setLayout(layout)
        
#         # Create New Scan button
#         btn_new_scan = QPushButton("➕ Create New Scan")
#         btn_new_scan.setStyleSheet("""
#             QPushButton {
#                 background-color: #4CAF50;
#                 color: white;
#                 border: none;
#                 padding: 10px 20px;
#                 border-radius: 6px;
#                 font-weight: bold;
#                 font-size: 14px;
#             }
#             QPushButton:hover {
#                 background-color: #45a049;
#             }
#         """)
#         btn_new_scan.clicked.connect(self.createScanRequested.emit)
#         layout.addWidget(btn_new_scan)
        
#         # Run Full Pipeline button
#         self.btn_run_pipeline = QPushButton("▶️ Run Full Pipeline")
#         self.btn_run_pipeline.setEnabled(False)
#         self.btn_run_pipeline.setStyleSheet("""
#             QPushButton {
#                 background-color: #2196F3;
#                 color: white;
#                 border: none;
#                 padding: 10px 20px;
#                 border-radius: 6px;
#                 font-weight: bold;
#                 font-size: 14px;
#             }
#             QPushButton:hover:enabled {
#                 background-color: #0b7dda;
#             }
#             QPushButton:disabled {
#                 background-color: #bbb;
#             }
#         """)
#         self.btn_run_pipeline.clicked.connect(self._run_full_pipeline)
#         layout.addWidget(self.btn_run_pipeline)
        
#         layout.addStretch()
        
#         # Pipeline Mode toggle
#         mode_label = QLabel("Pipeline Mode:")
#         mode_label.setStyleSheet("font-weight: bold;")
#         layout.addWidget(mode_label)
        
#         self.mode_combo = QComboBox()
#         self.mode_combo.addItems(["Auto", "Step-by-Step"])
#         self.mode_combo.setStyleSheet("""
#             QComboBox {
#                 padding: 8px 12px;
#                 border: 1px solid #ddd;
#                 border-radius: 4px;
#                 background-color: white;
#                 min-width: 150px;
#             }
#             QComboBox:hover {
#                 border-color: #2196F3;
#             }
#             QComboBox::drop-down {
#                 border: none;
#             }
#         """)
#         self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
#         layout.addWidget(self.mode_combo)
        
#         # Refresh button
#         btn_refresh = QPushButton("🔄")
#         btn_refresh.setToolTip("Refresh scans")
#         btn_refresh.setStyleSheet("""
#             QPushButton {
#                 background-color: #f5f5f5;
#                 border: 1px solid #ddd;
#                 padding: 8px 12px;
#                 border-radius: 4px;
#                 font-size: 16px;
#             }
#             QPushButton:hover {
#                 background-color: #e0e0e0;
#             }
#         """)
#         btn_refresh.clicked.connect(self.refresh_scans)
#         layout.addWidget(btn_refresh)
        
#         return bar
    
#     def _create_scan_table(self):
#         """Create the main scan table"""
#         table = QTableWidget()
        
#         # Configure columns
#         columns = ["Scan Name"] + list(self.STEP_COLUMNS.keys()) + ["Status", "Actions"]
#         table.setColumnCount(len(columns))
#         table.setHorizontalHeaderLabels(columns)
        
#         # Configure table properties
#         table.setSelectionBehavior(QTableWidget.SelectRows)
#         table.setSelectionMode(QTableWidget.SingleSelection)
#         table.setEditTriggers(QTableWidget.NoEditTriggers)
#         table.verticalHeader().setVisible(False)
#         table.setAlternatingRowColors(True)
        
#         # Column sizing
#         header = table.horizontalHeader()
#         header.setSectionResizeMode(0, QHeaderView.Stretch)  # Scan Name
#         for i in range(1, len(columns) - 2):  # Step columns
#             header.setSectionResizeMode(i, QHeaderView.Fixed)
#             table.setColumnWidth(i, 120)
#         header.setSectionResizeMode(len(columns) - 2, QHeaderView.Fixed)  # Status
#         table.setColumnWidth(len(columns) - 2, 150)
#         header.setSectionResizeMode(len(columns) - 1, QHeaderView.Fixed)  # Actions
#         table.setColumnWidth(len(columns) - 1, 200)
        
#         table.setMinimumHeight(300)
#         table.itemSelectionChanged.connect(self._on_selection_changed)
        
#         return table
    
#     def _setup_refresh_timer(self):
#         """Setup timer to auto-refresh table"""
#         self.refresh_timer = QTimer()
#         self.refresh_timer.timeout.connect(self.refresh_scans)
#         self.refresh_timer.start(3000)  # Refresh every 3 seconds
    
#     def refresh_scans(self):
#         """Refresh scan data from assets directory"""
#         if not self.assets_path.exists():
#             self.assets_path.mkdir(parents=True, exist_ok=True)
#             return
        
#         # Load all scan metadata
#         for scan_dir in self.assets_path.iterdir():
#             if not scan_dir.is_dir():
#                 continue
            
#             metadata_file = scan_dir / "metadata.json"
#             if metadata_file.exists():
#                 try:
#                     with open(metadata_file, 'r') as f:
#                         metadata = json.load(f)
#                         self.scans_data[scan_dir.name] = metadata
#                 except Exception as e:
#                     print(f"Error loading metadata for {scan_dir.name}: {e}")
        
#         self._update_table()
    
#     def _update_table(self):
#         """Update table with current scan data"""
#         self.table.setRowCount(len(self.scans_data))
        
#         for row, (scan_name, metadata) in enumerate(sorted(self.scans_data.items())):
#             # Scan Name
#             name_item = QTableWidgetItem(scan_name)
#             name_item.setData(Qt.UserRole, scan_name)
#             self.table.setItem(row, 0, name_item)
            
#             # Step columns
#             status_dict = metadata.get("status", {})
#             files_dict = metadata.get("files", {})
            
#             for col_idx, (col_name, step_key) in enumerate(self.STEP_COLUMNS.items(), start=1):
#                 step_status = status_dict.get(step_key, "pending")
                
#                 # Status indicator
#                 indicator = StatusIndicator(step_status)
#                 self.table.setCellWidget(row, col_idx, indicator)
                
#             # Overall Status column
#             overall_status = self._get_overall_status(metadata)
#             status_widget = self._create_status_widget(overall_status, metadata, scan_name)
#             self.table.setCellWidget(row, len(self.STEP_COLUMNS) + 1, status_widget)
            
#             # Actions column
#             actions_widget = self._create_actions_widget(metadata, scan_name)
#             self.table.setCellWidget(row, len(self.STEP_COLUMNS) + 2, actions_widget)
    
#     def _get_overall_status(self, metadata):
#         """Determine overall scan status"""
#         status_dict = metadata.get("status", {})
        
#         if any(s == "running" for s in status_dict.values()):
#             return "processing"
#         elif metadata.get("mode") == "step" and metadata.get("current_step"):
#             return "paused"
#         elif all(s == "done" for s in status_dict.values()):
#             return "complete"
#         elif any(s == "error" for s in status_dict.values()):
#             return "error"
#         else:
#             return "pending"
    
#     def _create_status_widget(self, status, metadata, scan_name):
#         """Create status column widget"""
#         widget = QWidget()
#         layout = QVBoxLayout()
#         layout.setContentsMargins(4, 4, 4, 4)
#         layout.setSpacing(4)
#         widget.setLayout(layout)
        
#         # Status label
#         status_labels = {
#             "processing": ("⏳ Processing", "#FF9800"),
#             "paused": ("⏸ Paused", "#2196F3"),
#             "complete": ("✔ Complete", "#4CAF50"),
#             "error": ("❌ Error", "#F44336"),
#             "pending": ("● Pending", "#9E9E9E")
#         }
        
#         text, color = status_labels.get(status, ("Unknown", "#000"))
#         label = QLabel(text)
#         label.setStyleSheet(f"color: {color}; font-weight: bold;")
#         layout.addWidget(label)
        
#         # Resume button (if paused)
#         if status == "paused":
#             btn_resume = QPushButton("Continue")
#             btn_resume.setStyleSheet("""
#                 QPushButton {
#                     background-color: #2196F3;
#                     color: white;
#                     border: none;
#                     padding: 4px 8px;
#                     border-radius: 3px;
#                     font-size: 11px;
#                 }
#                 QPushButton:hover {
#                     background-color: #0b7dda;
#                 }
#             """)
#             btn_resume.clicked.connect(lambda: self.continuePipelineRequested.emit(str(self.assets_path / scan_name)))
#             layout.addWidget(btn_resume)
        
#         return widget
    
#     def _create_actions_widget(self, metadata, scan_name):
#         """Create actions column widget"""
#         widget = QWidget()
#         layout = QHBoxLayout()
#         layout.setContentsMargins(4, 4, 4, 4)
#         layout.setSpacing(4)
#         widget.setLayout(layout)
        
#         status_dict = metadata.get("status", {})
#         files_dict = metadata.get("files", {})
        
#         # View Result button (if any step is complete)
#         if any(s == "done" for s in status_dict.values()):
#             # Find the most recent completed step
#             for step_key in reversed(list(self.STEP_COLUMNS.values())):
#                 if status_dict.get(step_key) == "done":
#                     file_key = self._get_file_key(step_key)
#                     if file_key and file_key in files_dict:
#                         btn_view = StepButton("done", str(self.assets_path / scan_name / files_dict[file_key]))
#                         btn_view.setText("View Result")
#                         btn_view.viewRequested.connect(self.openViewerRequested.emit)
#                         layout.addWidget(btn_view)
#                         break
        
#         # Delete button
#         btn_delete = QPushButton("🗑")
#         btn_delete.setToolTip("Delete scan")
#         btn_delete.setStyleSheet("""
#             QPushButton {
#                 background-color: #f5f5f5;
#                 border: 1px solid #ddd;
#                 padding: 6px 10px;
#                 border-radius: 4px;
#             }
#             QPushButton:hover {
#                 background-color: #ffebee;
#                 border-color: #F44336;
#             }
#         """)
#         btn_delete.clicked.connect(lambda: self._delete_scan(scan_name))
#         layout.addWidget(btn_delete)
        
#         layout.addStretch()
        
#         return widget
    
#     def _get_file_key(self, step_key):
#         """Map step key to file key"""
#         mapping = {
#             "slam": "pcd",
#             "filtering": "filtered",
#             "las_conversion": "las",
#             "flai": "segmented",
#             "fusion": "fused",
#             "wire_extraction": "wires"
#         }
#         return mapping.get(step_key)
    
#     def _on_selection_changed(self):
#         """Handle table selection change"""
#         selected_items = self.table.selectedItems()
#         if selected_items:
#             self.selected_scan = selected_items[0].data(Qt.UserRole)
#             self.btn_run_pipeline.setEnabled(True)
#             self.status_label.setText(f"Selected: {self.selected_scan}")
#         else:
#             self.selected_scan = None
#             self.btn_run_pipeline.setEnabled(False)
#             self.status_label.setText("Ready")
    
#     def _run_full_pipeline(self):
#         """Run full pipeline for selected scan"""
#         if self.selected_scan:
#             scan_path = str(self.assets_path / self.selected_scan)
#             self.runPipelineRequested.emit(scan_path)
#             self.add_notification(f"Started pipeline for {self.selected_scan}", "running")
    
#     def _on_mode_changed(self, mode):
#         """Handle pipeline mode change"""
#         if self.selected_scan:
#             metadata_file = self.assets_path / self.selected_scan / "metadata.json"
#             if metadata_file.exists():
#                 try:
#                     with open(metadata_file, 'r') as f:
#                         metadata = json.load(f)
                    
#                     metadata["mode"] = "step" if mode == "Step-by-Step" else "auto"
                    
#                     with open(metadata_file, 'w') as f:
#                         json.dump(metadata, f, indent=2)
                    
#                     self.refresh_scans()
#                 except Exception as e:
#                     print(f"Error updating mode: {e}")
    
#     def _delete_scan(self, scan_name):
#         """Delete a scan after confirmation"""
#         reply = QMessageBox.question(
#             self,
#             "Confirm Delete",
#             f"Are you sure you want to delete scan '{scan_name}'?\nThis cannot be undone.",
#             QMessageBox.Yes | QMessageBox.No
#         )
        
#         if reply == QMessageBox.Yes:
#             try:
#                 import shutil
#                 scan_path = self.assets_path / scan_name
#                 shutil.rmtree(scan_path)
#                 del self.scans_data[scan_name]
#                 self.refresh_scans()
#                 self.add_notification(f"Deleted {scan_name}", "info")
#             except Exception as e:
#                 QMessageBox.critical(self, "Error", f"Failed to delete scan: {e}")
    
#     def _show_notifications(self):
#         """Show notification panel"""
#         # Position below the button
#         button_pos = self.btn_notifications.mapToGlobal(self.btn_notifications.rect().bottomLeft())
#         panel_x = button_pos.x() - self.notification_panel.width() + self.btn_notifications.width()
#         panel_y = button_pos.y() + 5
#         self.notification_panel.move(panel_x, panel_y)
#         self.notification_panel.show()
    
#     def add_notification(self, message, status="info"):
#         """Add a notification to the panel"""
#         self.notification_panel.add_notification(message, status)
    
#     def update_scan_status(self, scan_name, step, status):
#         """Update status of a specific step for a scan"""
#         if scan_name in self.scans_data:
#             metadata = self.scans_data[scan_name]
#             if "status" not in metadata:
#                 metadata["status"] = {}
#             metadata["status"][step] = status
#             metadata["last_updated"] = datetime.now().isoformat()
            
#             # Save to file
#             metadata_file = self.assets_path / scan_name / "metadata.json"
#             try:
#                 with open(metadata_file, 'w') as f:
#                     json.dump(metadata, f, indent=2)
#             except Exception as e:
#                 print(f"Error saving metadata: {e}")
            
#             self.refresh_scans()
            
#             # Add notification
#             step_names = {v: k for k, v in self.STEP_COLUMNS.items()}
#             step_display = step_names.get(step, step)
#             self.add_notification(f"{scan_name} - {step_display}: {status}", status)


if __name__ == "__main__":
    from PySide6.QtWidgets import QApplication
    import sys
    
    app = QApplication(sys.argv)
    
    # Create test directory structure
    assets_path = Path("assets")
    assets_path.mkdir(exist_ok=True)
    
    # Create sample scan
    scan1 = assets_path / "scan_001"
    scan1.mkdir(exist_ok=True)
    
    metadata = {
        "name": "scan_001",
        "created_at": "2026-03-17",
        "status": {
            "slam": "done",
            "filtering": "done",
            "las_conversion": "running",
            "flai": "pending",
            "fusion": "pending",
            "wire_extraction": "pending"
        },
        "files": {
            "raw_pcd": "processed/point_clouds/scans.pcd",
            "raw_las": "processed/point_clouds/cloud.las",
            "filtered_pcd": "processed/point_clouds/scans_filtered.pcd",
            "filtered_las": "processed/point_clouds/cloud_filtered.las"
        },
        "params": {
            "voxel_size": 0.1,
            "outlier_neighbors": 20
        },
        "mode": "auto",
        "last_updated": datetime.now().isoformat()
    }
    
    with open(scan1 / "metadata.json", 'w') as f:
        json.dump(metadata, f, indent=2)
    
    # Create dashboard
    dashboard = DashboardView(assets_path="assets")
    
    # Connect signals for demo
    dashboard.openViewerRequested.connect(lambda path: print(f"Open viewer: {path}"))
    dashboard.runPipelineRequested.connect(lambda path: print(f"Run pipeline: {path}"))
    dashboard.createScanRequested.connect(lambda: print("Create new scan"))
    
    dashboard.resize(1400, 800)
    dashboard.show()
    
    sys.exit(app.exec())
