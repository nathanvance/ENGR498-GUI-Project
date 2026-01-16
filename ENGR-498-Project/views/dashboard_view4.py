# dashboard_view.py
import os
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QFileDialog, QMessageBox
)
from PySide6.QtCore import Qt, Signal


class DashboardView(QWidget):
    # -------------------------------------------
    # Signals expected by MainWindow
    # -------------------------------------------
    openViewerRequested = Signal(str)      # emits LAS file path
    uploadFlaiRequested = Signal(str)      # emits LAS file path (future)
    # -------------------------------------------

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dashboard")
        self.resize(900, 500)

        self.selected_file = None

        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignTop)
        self.setLayout(main_layout)

        # ---- Title ----
        title = QLabel("LiDAR Application Dashboard")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 26px; font-weight: bold; margin: 20px;")
        main_layout.addWidget(title)

        # ---- Row: Left side file select  |  Right side buttons ----
        row = QHBoxLayout()
        row.setSpacing(40)
        main_layout.addLayout(row)

        # ------------------------------------------------------------
        # LEFT SIDE – File selection
        # ------------------------------------------------------------
        left_box = QVBoxLayout()
        left_box.setAlignment(Qt.AlignTop)
        row.addLayout(left_box, stretch=2)

        self.file_label = QLabel("No file selected")
        self.file_label.setStyleSheet("font-size: 15px; margin-bottom: 10px;")
        left_box.addWidget(self.file_label)

        select_btn = QPushButton("Select LAS File")
        select_btn.setFixedHeight(45)
        select_btn.clicked.connect(self.select_file)
        left_box.addWidget(select_btn)

        # ------------------------------------------------------------
        # RIGHT SIDE – Buttons (Open viewer & Upload to FLAI)
        # ------------------------------------------------------------
        right_box = QVBoxLayout()
        right_box.setAlignment(Qt.AlignTop)
        row.addLayout(right_box, stretch=1)

        # Open viewer button
        self.open_viewer_btn = QPushButton("Open Viewer")
        self.open_viewer_btn.setFixedHeight(45)
        self.open_viewer_btn.setEnabled(False)
        self.open_viewer_btn.clicked.connect(self.open_viewer_clicked)
        right_box.addWidget(self.open_viewer_btn)

        # Upload to FLAI button
        self.upload_flai_btn = QPushButton("Upload to FLAI")
        self.upload_flai_btn.setFixedHeight(45)
        self.upload_flai_btn.setEnabled(False)
        self.upload_flai_btn.clicked.connect(self.upload_to_flai_clicked)
        right_box.addWidget(self.upload_flai_btn)

    # ------------------------------------------------------------
    # File selection
    # ------------------------------------------------------------
    def select_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select .las file",
            "",
            "LAS Files (*.las)"
        )
        if not file_path:
            return

        self.selected_file = file_path
        self.file_label.setText(os.path.basename(file_path))

        # enable both action buttons
        self.open_viewer_btn.setEnabled(True)
        self.upload_flai_btn.setEnabled(True)

    # ------------------------------------------------------------
    # Emit signal to open viewer
    # ------------------------------------------------------------
    def open_viewer_clicked(self):
        if not self.selected_file:
            QMessageBox.warning(self, "No file", "Please select a .las file first.")
            return

        # Emit path so the main window reacts properly
        self.openViewerRequested.emit(self.selected_file)

    # ------------------------------------------------------------
    # Emit signal to upload to FLAI (stub)
    # ------------------------------------------------------------
    def upload_to_flai_clicked(self):
        if not self.selected_file:
            QMessageBox.warning(self, "No file", "Please select a .las file first.")
            return

        # For now just emit a signal or popup; real API coming later
        self.uploadFlaiRequested.emit(self.selected_file)

        QMessageBox.information(
            self,
            "FLAI Upload (Stub)",
            "FLAI upload would happen here.\n\n"
            "Later we will add:\n"
            "- API token handling\n"
            "- Upload request\n"
            "- Flow execution\n"
            "- Status polling\n"
            "- Download of results"
        )
