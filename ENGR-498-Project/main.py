# main.py
import sys
import threading
import matlab.engine
import numpy as np
from PySide6.QtWidgets import QApplication, QMainWindow, QStackedWidget

from views.dashboard_view import DashboardView

# IMPORT EACH VIEWER WITH A UNIQUE NAME
from views.semantic_viewer import SemanticViewer as PySemanticViewer
from Matlab_ExtractPowerLine.testSemanticLidarViewer import SemanticViewer as MatlabSemanticViewer
#from ProjectState import ProjectState #import the shared state object



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        #self.project_state = ProjectState() #initialize shared state
        self.setWindowTitle("LiDAR App")

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.dashboard = DashboardView()
        #self.dashboard = DashboardView(self.project_state)  

        # Two DIFFERENT viewer objects
        self.python_viewer = PySemanticViewer()         # used by Open Viewer
        self.matlab_viewer = MatlabSemanticViewer()     # used by Open Semantic Viewer

        self.stack.addWidget(self.dashboard)        # index 0
        self.stack.addWidget(self.python_viewer)    # index 1
        self.stack.addWidget(self.matlab_viewer)    # index 2

        # Signals
        self.dashboard.openViewerRequested.connect(self.openPythonViewer)
        self.dashboard.openSemanticRequested.connect(self.openMatlabViewer)
        self.dashboard.runMatlabRequested.connect(self.run_matlab_async)

        # Back buttons
        self.python_viewer.backRequested.connect(
            lambda: self.stack.setCurrentIndex(0)
        )
        self.matlab_viewer.backRequested.connect(
            lambda: self.stack.setCurrentIndex(0)
        )

    # -------------------------------
    def openPythonViewer(self):
        self.python_viewer.load_las_file()
        self.stack.setCurrentIndex(1)

        # self.semantic_viewer.load_las_file()  # loads the built-in LAS
        # self.stack.setCurrentIndex(1)

    # -------------------------------
    def openMatlabViewer(self):
        self.matlab_viewer.initialize_viewer()
        self.stack.setCurrentIndex(2)

    # -------------------------------
    def run_matlab_async(self, las_path):
        def worker():
            print("\n--- MATLAB extraction started ---")

            eng = matlab.engine.start_matlab()
            eng.addpath(
                r"C:\Users\henry\Downloads\3DLiDAR-main\3DLiDAR-main\ExtractPowerLine",
                #r"C:\Users\henry\OneDrive\Documents\GitHub\ENGR498-GUI-Project\ENGR-498-Project\Matlab_ExtractPowerLine",
                nargout=0
            )

            PL, poly, ground_pts = eng.demo_extract_powerline(las_path, nargout=3)

            print("MATLAB extraction finished!")
            print("PL:", np.array(PL).shape)
            print("Ground:", np.array(ground_pts).shape)

        threading.Thread(target=worker, daemon=True).start()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(1300, 800)
    window.show()
    sys.exit(app.exec())
