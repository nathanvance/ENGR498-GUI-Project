# dashboard_view.py
import os
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QFileDialog, QMessageBox
)
from PySide6.QtCore import Qt, Signal


class DashboardView(QWidget):
    
    # openViewerRequested = Signal(str)      # emits LAS file path
    # uploadFlaiRequested = Signal(str)      # emits LAS file path
    # runMatlabRequested = Signal(str)       # MATLAB extraction LAS path
    # openSemanticRequested = Signal()       # open MATLAB semantic viewer

    openViewerRequested = Signal()
    uploadFlaiRequested = Signal()
    runMatlabRequested = Signal()
    openSemanticRequested = Signal()  #this one might be unused and can probably be removed

    #projectInitialized = Signal()   # NEW


    def __init__(self):
        super().__init__()
        #self.project_state = project_state
        self.setWindowTitle("Dashboard")
        self.resize(900, 500)

        self.selected_file = None  #not needed anymore, project state is used instead.

        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignTop)
        self.setLayout(main_layout)

        title = QLabel("LiDAR Application Dashboard")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 26px; font-weight: bold; margin: 20px;")
        main_layout.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(40)
        main_layout.addLayout(row)

        # LEFT BOX (file selection)
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

        # RIGHT BOX (buttons)
        right_box = QVBoxLayout()
        right_box.setAlignment(Qt.AlignTop)
        row.addLayout(right_box, stretch=1)

        # -------------------------------
        # REORDERED BUTTONS
        # -------------------------------

        # Upload to FLAI FIRST
        self.upload_flai_btn = QPushButton("Upload to FLAI")
        self.upload_flai_btn.setFixedHeight(45)
        self.upload_flai_btn.setEnabled(False)
        self.upload_flai_btn.clicked.connect(self.upload_to_flai_clicked)
        right_box.addWidget(self.upload_flai_btn)

        # Open Viewer (Python Open3D viewer)
        self.open_viewer_btn = QPushButton("Open Viewer")
        self.open_viewer_btn.setFixedHeight(45)
        self.open_viewer_btn.setEnabled(False)
        self.open_viewer_btn.clicked.connect(self.open_viewer_clicked)
        right_box.addWidget(self.open_viewer_btn)

        # MATLAB extraction
        self.run_matlab_btn = QPushButton("Execute Wire Extraction and Modeling")
        self.run_matlab_btn.setFixedHeight(45)
        self.run_matlab_btn.setEnabled(False)
        self.run_matlab_btn.clicked.connect(self.run_matlab_clicked)
        right_box.addWidget(self.run_matlab_btn)

        # MATLAB Semantic Viewer
        self.open_semantic_btn = QPushButton("Open Semantic Viewer")
        self.open_semantic_btn.setFixedHeight(45)
        self.open_semantic_btn.clicked.connect(
            lambda: self.openSemanticRequested.emit()
        )
        right_box.addWidget(self.open_semantic_btn)


    # ----------------------------
    def select_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select .las file",
            "",
            "LAS Files (*.las)"
        )
        if not file_path:
            return

        #self.project_state.las_path = file_path
        #self.project_state.reset_to_raw()   # important if user reloads a file
        self.file_label.setText(os.path.basename(file_path))

        #self.projectInitialized.emit()

        self.open_viewer_btn.setEnabled(True)
        self.upload_flai_btn.setEnabled(True)
        self.run_matlab_btn.setEnabled(True)

    # ----------------------------
    def open_viewer_clicked(self):
        # if not self.selected_file:
        #     QMessageBox.warning(self, "No file", "Please select a .las file first.")
        #     return
        self.openViewerRequested.emit(self.selected_file)

    # ----------------------------
    def upload_to_flai_clicked(self):
        if not self.selected_file:
            QMessageBox.warning(self, "No file", "Please select a .las file first.")
            return
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

    # ----------------------------
    def run_matlab_clicked(self):
        if not self.selected_file:
            QMessageBox.warning(self, "No file", "Select a .las file first.")
            return
        self.runMatlabRequested.emit(self.selected_file)
