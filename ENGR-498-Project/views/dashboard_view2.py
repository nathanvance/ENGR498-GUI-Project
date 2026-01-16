from PySide6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QFileDialog, QLabel
from PySide6.QtCore import Signal

class DashboardView(QWidget):
    fileSelected = Signal(str)  # send LAS path to MainWindow

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()

        self.label = QLabel("No file selected.")
        self.btnLoad = QPushButton("Load Point Cloud File")
        self.btnContinue = QPushButton("Open Semantic Viewer")
        self.btnContinue.setEnabled(False)

        layout.addWidget(self.label)
        layout.addWidget(self.btnLoad)
        layout.addWidget(self.btnContinue)
        self.setLayout(layout)

        self.btnLoad.clicked.connect(self.select_file)
        self.btnContinue.clicked.connect(self.confirm_and_switch)

        self.las_path = None

    def select_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Select LAS File",
            "",
            "LAS Files (*.las)"
        )
        if path:
            self.las_path = path
            self.label.setText(f"Selected file:\n{path}")
            self.btnContinue.setEnabled(True)

    def confirm_and_switch(self):
        if self.las_path:
            self.fileSelected.emit(self.las_path)