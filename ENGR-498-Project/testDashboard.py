import sys
import json
from pathlib import Path
import os
import shutil
from datetime import datetime
from PySide6.QtWidgets import QFileDialog, QMessageBox
from PySide6.QtWidgets import QApplication, QMainWindow, QStackedWidget

from project_paths import ASSETS_DIR
from views.lidar_dashboard import DashboardView
from views.lidar_dashboard_stepbystep import StepByStepDashboard
from PreProcessing_GUI.point_cloud_filter_gui import PointCloudFilterViewer


class DashboardTestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dashboard Test")
        self.resize(1600, 900)

        self.setup_demo_assets()

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.auto_dashboard = DashboardView(assets_path=str(ASSETS_DIR))
        self.step_dashboard = StepByStepDashboard(assets_path=str(ASSETS_DIR))
        self.filter_viewer = PointCloudFilterViewer(filename=None)

        self.stack.addWidget(self.auto_dashboard)   # index 0
        self.stack.addWidget(self.step_dashboard)   # index 1
        self.stack.addWidget(self.filter_viewer)    # index 2

        self.stack.setCurrentWidget(self.auto_dashboard)

        self.auto_dashboard.switchToStepModeRequested.connect(
            lambda: self.switch_to(self.step_dashboard)
        )
        self.step_dashboard.switchToAutoModeRequested.connect(
            lambda: self.switch_to(self.auto_dashboard)
        )
        #new - connect create scan signal from both dashboards to the same slot
        self.auto_dashboard.createScanRequested.connect(self.create_new_scan)
        self.step_dashboard.createScanRequested.connect(self.create_new_scan)

        # FIXED: accept filepath from signal
        self.step_dashboard.openFilterViewerRequested.connect(
            self.open_filter_viewer
        )

        #NEW - connect back button in filter viewer to step dashboard
        self.filter_viewer.backRequested.connect(
            lambda: self.switch_to(self.step_dashboard)
        )

        #include another connection to switch back to the step by step dashboard

    def switch_to(self, widget):
        self.stack.setCurrentWidget(widget)

    def open_filter_viewer(self, filepath):
        print("Received filepath:", filepath)

        self.filter_viewer.filename = filepath

        # Call a load method if your viewer has one
        if hasattr(self.filter_viewer, "load_point_cloud"):
            self.filter_viewer.load_point_cloud()

        self.switch_to(self.filter_viewer)

    def setup_demo_assets(self):
        assets_path = ASSETS_DIR
        assets_path.mkdir(exist_ok=True)

        scan1 = assets_path / "scan_001"
        scan1.mkdir(exist_ok=True)

        metadata1 = {
            "name": "scan_001",
            "created_at": "2026-03-17",
            "status": {
                "slam": "done",
                "filtering": "done",
                "flai": "pending",
                "wire_extraction": "pending",
                "fusion": "pending"
            },
            "files": "assets/scan_001/processed/slam/lt1.las",
            "wire_params": {
                "R": 0.5,
                "angleThr": 10,
                "linearity": 0.98
            }
        }

        with open(scan1 / "metadata.json", "w") as f:
            json.dump(metadata1, f, indent=2)

        scan2 = assets_path / "scan_002"
        scan2.mkdir(exist_ok=True)

        metadata2 = {
            "name": "scan_002",
            "created_at": "2026-03-20",
            "status": {
                "slam": "done",
                "filtering": "pending",
                "flai": "pending",
                "wire_extraction": "pending",
                "fusion": "pending"
            },
            "files": "assets/scan_002/processed/slam/lt3.las",
            "wire_params": {
                "R": 0.6,
                "angleThr": 12,
                "linearity": 0.97
            }
        }

        with open(scan2 / "metadata.json", "w") as f:
            json.dump(metadata2, f, indent=2)
    
    #this function is used to upload a new scan and create the necessary folder structure and metadata for it. 
    #It also copies the selected rosbag into the new scan's raw folder. Finally, it refreshes the dashboards to show the new scan.
    def create_new_scan(self):
        """Open file dialog, create new scan folder structure, and copy rosbag."""
        
        # Ask user to select a rosbag file
        bag_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Rosbag File",
            "",
            "Rosbag Files (*.bag);;All Files (*)"
        )

        if not bag_path:
            return  # User cancelled

        assets_path = ASSETS_DIR
        assets_path.mkdir(exist_ok=True)

        # ---------------------------------------------------
        # Find next scan number automatically
        # ---------------------------------------------------
        existing_scans = []
        for folder in assets_path.iterdir():
            if folder.is_dir() and folder.name.startswith("scan_"):
                try:
                    num = int(folder.name.split("_")[1])
                    existing_scans.append(num)
                except:
                    pass

        next_num = max(existing_scans, default=0) + 1
        scan_name = f"scan_{next_num:03d}"
        scan_path = assets_path / scan_name

        # ---------------------------------------------------
        # Create folder structure
        # ---------------------------------------------------
        raw_path = scan_path / "raw"
        processed_path = scan_path / "processed"

        (raw_path / "images").mkdir(parents=True, exist_ok=True)
        (processed_path / "slam").mkdir(parents=True, exist_ok=True)
        (processed_path / "filtered").mkdir(parents=True, exist_ok=True)
        (processed_path / "flai").mkdir(parents=True, exist_ok=True)
        (processed_path / "wires").mkdir(parents=True, exist_ok=True)
        (processed_path / "fusion").mkdir(parents=True, exist_ok=True)

        # ---------------------------------------------------
        # Copy rosbag into raw folder
        # ---------------------------------------------------
        bag_filename = os.path.basename(bag_path)
        destination_bag = raw_path / bag_filename
        shutil.copy2(bag_path, destination_bag)

        # ---------------------------------------------------
        # Create metadata.json
        # ---------------------------------------------------
        metadata = {
            "name": scan_name,
            "created_at": datetime.now().strftime("%Y-%m-%d"),
            "status": {
                "slam": "pending",
                "filtering": "pending",
                "flai": "pending",
                "wire_extraction": "pending",
                "fusion": "pending"
            },
            "filepaths": {
                "rosbag": f"raw/{bag_filename}",
                "images_dir": "raw/images",
                "pcd": "processed/slam/cloud.pcd",
                "las": "processed/slam/cloud.las",
                "filtered": "processed/filtered/cloud_filtered.las",
                "flai": "processed/flai/segmented.las",
                "fused": "processed/fusion/fused.las",
                "wires": "processed/wires/wires.npz",
                "ground": "processed/wires/ground.npz"
            },
            "wire_params": {
                "R": 0.5,
                "angleThr": 10,
                "linearity": 0.98
            }
        }

        with open(scan_path / "metadata.json", "w") as f:
            json.dump(metadata, f, indent=2)

        QMessageBox.information(
            self,
            "Scan Created",
            f"Created {scan_name}\n\nRosbag copied to:\n{destination_bag}"
        )

        # ---------------------------------------------------
        # Refresh both dashboards so new scan appears
        # ---------------------------------------------------
        self.refresh_dashboards()

    #this method refreshes the scan table in the dashboard
    def refresh_dashboards(self):
        print("Refreshing dashboards to show new scan...")
        """Reload dashboard scan tables after creating/updating scans."""
        if hasattr(self.auto_dashboard, "load_scans"):
            self.auto_dashboard.refresh_scans()

        if hasattr(self.step_dashboard, "load_scans"):
            self.step_dashboard.refresh_scans()

    


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DashboardTestWindow()
    window.show()
    sys.exit(app.exec())
