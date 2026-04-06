"""
LiDAR Processing Dashboard (Step-by-Step Mode)
Advanced control center with detailed step control, checkboxes, and parameter modification.
"""

import json
from pathlib import Path
from datetime import datetime

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QTableWidget, QTableWidgetItem, QHeaderView, QCheckBox,
    QFrame, QScrollArea, QFileDialog, QMessageBox,
    QToolButton, QDialog, QDoubleSpinBox, QSpinBox, QFormLayout,
    QDialogButtonBox, QTextEdit
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QFont


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
    openViewerRequested = Signal(str)  # file_path
    createScanRequested = Signal()
    switchToAutoModeRequested = Signal()  # Switch back to auto mode
    openFilterViewerRequested = Signal(str)  # Open the point cloud filter viewer
    
    # Reordered: Wires before Fusion, removed LAS
    STEP_COLUMNS = {
        "SLAM": "slam",
        "Filtering": "filtering",
        "FLAI": "flai",
        "Wires": "wire_extraction",
        "Fusion": "fusion"
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
        
        # Scan table
        self.table = self._create_scan_table()
        main_layout.addWidget(self.table, stretch=1)
        
        # Status bar
        self.status_label = QLabel("Ready - Step-by-Step Mode allows detailed control over each processing step")
        self.status_label.setStyleSheet("color: #666; padding: 8px;")
        main_layout.addWidget(self.status_label)
        
        # Apply global styles
        self.setStyleSheet("""
            QWidget {
                background-color: #fafafa;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QTableWidget {
                background-color: white;
                border: 1px solid #ddd;
                border-radius: 8px;
                gridline-color: #e0e0e0;
            }
            QTableWidget::item {
                padding: 8px;
            }
            QTableWidget::item:selected {
                background-color: #e3f2fd;
                color: black;
            }
            QHeaderView::section {
                background-color: #f5f5f5;
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
        print("refresh button clicked");
        btn_refresh.clicked.connect(self.refresh_scans)
        layout.addWidget(btn_refresh)
        
        return bar
    
    def _create_scan_table(self):
        """Create the scan table with checkboxes"""
        table = QTableWidget()
        
        columns = ["Scan Name"] + list(self.STEP_COLUMNS.keys()) + ["Actions"]
        table.setColumnCount(len(columns))
        table.setHorizontalHeaderLabels(columns)
        
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
            table.setColumnWidth(i, 180)
        header.setSectionResizeMode(len(columns) - 1, QHeaderView.Fixed)  # Actions
        table.setColumnWidth(len(columns) - 1, 280)
        
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
        
        for scan_dir in self.assets_path.iterdir():
            if not scan_dir.is_dir():
                continue
            
            metadata_file = scan_dir / "metadata.json"
            if metadata_file.exists():
                try:
                    with open(metadata_file, 'r') as f:
                        metadata = json.load(f)
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
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        widget.setLayout(layout)
        
        # Checkbox for completion
        checkbox = QCheckBox("Complete")
        checkbox.setChecked(status_dict.get(step_key) == "done")
        checkbox.stateChanged.connect(lambda state, sn=scan_name, sk=step_key: self._on_checkbox_changed(sn, sk, state))
        layout.addWidget(checkbox)
        
        # Buttons layout
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(4)
        
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
        btn_view.clicked.connect(lambda: self._on_view_step(scan_name, step_key))
        btn_layout.addWidget(btn_view)
        
        # Modify button
        btn_modify = QPushButton("Modify")
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
        btn_view.clicked.connect(lambda: print(f"View result for {scan_name}"))
        layout.addWidget(btn_view)
        
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
        
        metadata_file = self.assets_path / scan_name / "metadata.json"
        try:
            with open(metadata_file, 'w') as f:
                json.dump(self.scans_data[scan_name], f, indent=2)
            
            self.modified_scans.discard(scan_name)
            self.add_notification(f"Saved changes for {scan_name}", "done")
            self.refresh_scans()
            print(f"✓ Saved metadata for {scan_name}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save: {e}")
    
    def _on_view_step(self, scan_name, step_key):
        """Handle view button click"""
        print(f"VIEW clicked: {scan_name} -> {step_key}")
        self.add_notification(f"View requested: {scan_name} - {step_key}", "info")
    

    #TODO I have to pass the parameters to wire extraction TODO TODO TODO TODO 
    def _on_modify_step(self, scan_name, step_key):
        """Handle modify button click"""
        if step_key == "wire_extraction":
            # Show parameter dialog for wires
            current_params = self.scans_data.get(scan_name, {}).get("wire_params", {})
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
        if step_key == "filtering":
            print(f"MODIFY clicked: {scan_name} -> {step_key}")
            if scan_name in self.scans_data:
                filepath = self.scans_data[scan_name]["files"]
                print(f"Opening filter viewer for {scan_name} with file: {filepath}")
                # self.scans_data[scan_name]["wire_params"] = params
                
            #pass in filepath instead of scan name
                self.openFilterViewerRequested.emit(filepath)
            
        else:
            print(f"MODIFY clicked: {scan_name} -> {step_key}")
            self.add_notification(f"Modify requested: {scan_name} - {step_key}", "info")
    
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
        else:
            self.selected_scan = None
            self.status_label.setText("Ready - Step-by-Step Mode")
    
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
