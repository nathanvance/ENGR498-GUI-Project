# views/dashboard_view.py
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from PySide6.QtCore import Signal, Qt

class DashboardView(QWidget):
    # A simple signal to tell the main window to open the viewer
    openViewerRequested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        self.setLayout(layout)

        title = QLabel("LiDAR Dashboard")
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        desc = QLabel("This dashboard is hardcoded to open a single .las\nUse the viewer to inspect the point cloud.")
        desc.setAlignment(Qt.AlignCenter)
        layout.addWidget(desc)

        open_btn = QPushButton("Open Viewer (testSemanticPC.las)")
        open_btn.setFixedWidth(300)
        open_btn.clicked.connect(self._on_open_clicked)
        layout.addWidget(open_btn)

    def _on_open_clicked(self):
        # emit the signal asking MainWindow to show the viewer page
        self.openViewerRequested.emit()
