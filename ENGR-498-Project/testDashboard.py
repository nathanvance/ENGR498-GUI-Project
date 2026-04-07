import json
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path

from PySide6.QtWidgets import QApplication, QFileDialog, QMainWindow, QMessageBox, QStackedWidget, QToolBar

from Matlab_ExtractPowerLine.testSemanticLidarViewer import SemanticViewer as CombinedSemanticViewer
from PreProcessing_GUI.point_cloud_filter_gui import PointCloudFilterViewer
from calibration_bridge import CALIBRATION_OUTPUT_ROOT, newest_calibration_run, resolve_calibration_run
from gui_pipeline import BackendPipelineThread, CalibrationWorkflowThread, LeafletServerManager
from project_paths import ASSETS_DIR
from scan_metadata import (
    ensure_scan_structure,
    load_scan_metadata,
    relativize_for_scan,
    resolve_artifact_paths,
    resolve_scan_path,
    save_scan_metadata,
)
from views.calibration_mode import CalibrationModeView
from views.lidar_dashboard import DashboardView
from views.lidar_dashboard_stepbystep import StepByStepDashboard


class DashboardTestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiDAR Processing Dashboard")
        self.resize(1680, 960)

        self._leaflet_manager = LeafletServerManager()
        self._pipeline_thread: BackendPipelineThread | None = None
        self._calibration_thread: CalibrationWorkflowThread | None = None
        self._previous_widget = None

        self.setup_demo_assets()

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)
        self._setup_mode_toolbar()

        self.auto_dashboard = DashboardView(assets_path=str(ASSETS_DIR))
        self.step_dashboard = StepByStepDashboard(assets_path=str(ASSETS_DIR))
        self.calibration_view = CalibrationModeView(default_output_root=CALIBRATION_OUTPUT_ROOT)
        self.filter_viewer = PointCloudFilterViewer(filename=None)
        self.semantic_viewer = CombinedSemanticViewer()
        self.semantic_viewer.set_open_map_callback(self.open_map_for_scan)

        self.stack.addWidget(self.auto_dashboard)   # index 0
        self.stack.addWidget(self.step_dashboard)   # index 1
        self.stack.addWidget(self.calibration_view) # index 2
        self.stack.addWidget(self.filter_viewer)    # index 3
        self.stack.addWidget(self.semantic_viewer)  # index 4

        self.stack.setCurrentWidget(self.auto_dashboard)
        self._connect_signals()

    def _setup_mode_toolbar(self):
        toolbar = QToolBar("Modes", self)
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        post_auto = toolbar.addAction("Post-Processing Auto")
        post_auto.triggered.connect(lambda: self.switch_to(self.auto_dashboard))

        post_step = toolbar.addAction("Post-Processing Step-By-Step")
        post_step.triggered.connect(lambda: self.switch_to(self.step_dashboard))

        calibration = toolbar.addAction("Calibration Mode")
        calibration.triggered.connect(lambda: self.switch_to(self.calibration_view))

    def _connect_signals(self):
        self.auto_dashboard.switchToStepModeRequested.connect(lambda: self.switch_to(self.step_dashboard))
        self.step_dashboard.switchToAutoModeRequested.connect(lambda: self.switch_to(self.auto_dashboard))
        self.calibration_view.switchToPostProcessingRequested.connect(lambda: self.switch_to(self.auto_dashboard))

        self.auto_dashboard.createScanRequested.connect(self.create_new_scan)
        self.step_dashboard.createScanRequested.connect(self.create_new_scan)

        self.auto_dashboard.runPipelineRequested.connect(self.run_full_pipeline)
        self.step_dashboard.runPipelineRequested.connect(self.run_full_pipeline)
        self.step_dashboard.runStepRequested.connect(self.run_pipeline_step)

        self.auto_dashboard.openViewerRequested.connect(self.open_semantic_viewer)
        self.step_dashboard.openViewerRequested.connect(self.open_semantic_viewer)

        self.auto_dashboard.openMapRequested.connect(self.open_map_for_scan)
        self.step_dashboard.openMapRequested.connect(self.open_map_for_scan)

        self.step_dashboard.openFilterViewerRequested.connect(self.open_filter_viewer)
        self.calibration_view.runCalibrationRequested.connect(self.run_calibration_workflow)

        self.filter_viewer.backRequested.connect(self._restore_previous_widget)
        self.semantic_viewer.backRequested.connect(self._restore_previous_widget)

    def switch_to(self, widget):
        self.stack.setCurrentWidget(widget)

    def _show_temporary_view(self, widget):
        self._previous_widget = self.stack.currentWidget()
        self.switch_to(widget)

    def _restore_previous_widget(self):
        target = self._previous_widget if self._previous_widget is not None else self.auto_dashboard
        self.switch_to(target)

    def closeEvent(self, event):
        self._leaflet_manager.shutdown()
        super().closeEvent(event)

    def setup_demo_assets(self):
        ASSETS_DIR.mkdir(exist_ok=True)
        if any(path.is_dir() for path in ASSETS_DIR.iterdir()):
            for scan_dir in ASSETS_DIR.iterdir():
                if scan_dir.is_dir():
                    try:
                        scan_path, metadata = load_scan_metadata(scan_dir)
                        save_scan_metadata(scan_path, metadata)
                    except Exception:
                        pass
            return

        for index in (1, 2, 3):
            scan_name = f"scan_{index:03d}"
            scan_dir = ASSETS_DIR / scan_name
            ensure_scan_structure(scan_dir)
            metadata = {
                "name": scan_name,
                "created_at": datetime.now().strftime("%Y-%m-%d"),
                "status": {
                    "slam": "pending",
                    "filtering": "pending",
                    "flai": "pending",
                    "wire_extraction": "pending",
                    "inference": "pending",
                    "fusion": "pending",
                },
                "files": {},
                "notes": "Add a rosbag under raw/, complete calibration mode separately, then run post-processing.",
            }
            save_scan_metadata(scan_dir, metadata)

    def refresh_dashboards(self):
        self.auto_dashboard.refresh_scans()
        self.step_dashboard.refresh_scans()

    def create_new_scan(self):
        bag_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Rosbag File",
            "",
            "Rosbag Files (*.bag);;All Files (*)",
        )
        if not bag_path:
            return

        existing = []
        for folder in ASSETS_DIR.iterdir():
            if folder.is_dir() and folder.name.startswith("scan_"):
                try:
                    existing.append(int(folder.name.split("_")[1]))
                except Exception:
                    pass

        next_num = max(existing, default=0) + 1
        scan_name = f"scan_{next_num:03d}"
        scan_dir = ASSETS_DIR / scan_name
        ensure_scan_structure(scan_dir)

        bag_filename = os.path.basename(bag_path)
        destination_bag = scan_dir / "raw" / bag_filename
        shutil.copy2(bag_path, destination_bag)

        metadata = {
            "name": scan_name,
            "created_at": datetime.now().strftime("%Y-%m-%d"),
            "notes": "",
            "files": {
                "rosbag": "raw/" + bag_filename,
            },
        }
        save_scan_metadata(scan_dir, metadata)

        QMessageBox.information(
            self,
            "Scan Created",
            f"Created {scan_name}\n\nRosbag copied to:\n{destination_bag}",
        )
        self.refresh_dashboards()

    def _prompt_for_file(self, title: str, pattern: str) -> str:
        chosen, _ = QFileDialog.getOpenFileName(self, title, "", pattern)
        return chosen

    def ensure_pipeline_configuration(self, scan_ref: str | Path) -> tuple[Path, dict] | None:
        scan_dir, metadata = load_scan_metadata(scan_ref)
        changed = False
        fusion_cfg = metadata.setdefault("config", {}).setdefault("fusion", {})
        calibration_run = resolve_scan_path(scan_dir, fusion_cfg.get("calibration_run_dir")) or resolve_scan_path(
            scan_dir, metadata.get("files", {}).get("calibration_run_dir")
        )
        calibration_run = resolve_calibration_run(calibration_run, fallback_to_latest=True)
        if calibration_run is None:
            chosen = QFileDialog.getExistingDirectory(
                self,
                "Select completed calibration run folder",
                str(CALIBRATION_OUTPUT_ROOT),
            )
            if not chosen:
                QMessageBox.warning(
                    self,
                    "Pipeline Cancelled",
                    "Fusion requires a completed calibration mode output. Run calibration mode first or select a calibration run folder.",
                )
                return None
            calibration_run = Path(chosen)

        fusion_cfg["calibration_run_dir"] = relativize_for_scan(scan_dir, calibration_run)
        changed = True

        if changed:
            save_scan_metadata(scan_dir, metadata)
            scan_dir, metadata = load_scan_metadata(scan_dir)
        return scan_dir, metadata

    def _start_pipeline_thread(self, scan_dir: Path, mode: str):
        if self._pipeline_thread is not None and self._pipeline_thread.isRunning():
            QMessageBox.information(self, "Pipeline Busy", "A backend pipeline is already running.")
            return

        self._pipeline_thread = BackendPipelineThread(scan_dir, mode=mode, parent=self)
        self._pipeline_thread.logLine.connect(self._on_pipeline_log)
        self._pipeline_thread.stageChanged.connect(self._on_pipeline_stage)
        self._pipeline_thread.completed.connect(self._on_pipeline_complete)
        self._pipeline_thread.failed.connect(self._on_pipeline_failed)
        self._pipeline_thread.start()

    def run_full_pipeline(self, scan_ref: str):
        prepared = self.ensure_pipeline_configuration(scan_ref)
        if prepared is None:
            return
        scan_dir, _ = prepared
        summary = (
            f"Running full backend chain for {scan_dir.name}: Rosbag Preprocessing -> Wires -> Image Inference -> Fusion + GPS"
        )
        self.auto_dashboard.add_notification(summary, "running")
        self.step_dashboard.add_notification(summary, "running")
        self._start_pipeline_thread(scan_dir, mode="full")

    def run_pipeline_step(self, scan_ref: str, step_key: str):
        scan_dir, metadata = load_scan_metadata(scan_ref)

        if step_key == "slam":
            self.step_dashboard.add_notification(f"Running rosbag preprocessing for {scan_dir.name}", "running")
            self._start_pipeline_thread(scan_dir, mode="pose-recovery")
            return

        if step_key == "wire_extraction":
            self.step_dashboard.add_notification(f"Running wire extraction for {scan_dir.name}", "running")
            self._start_pipeline_thread(scan_dir, mode="wire-extraction")
            return

        if step_key == "inference":
            self.step_dashboard.add_notification(f"Running image inference for {scan_dir.name}", "running")
            self._start_pipeline_thread(scan_dir, mode="inference")
            return

        if step_key == "fusion":
            prepared = self.ensure_pipeline_configuration(scan_dir)
            if prepared is None:
                return
            scan_dir, metadata = prepared
            self.step_dashboard.add_notification(f"Running fusion and GPS georeferencing for {scan_dir.name}", "running")
            self._start_pipeline_thread(scan_dir, mode="fusion")
            return

        if step_key == "filtering":
            files = resolve_artifact_paths(scan_dir, metadata)
            for candidate_key in ("filtered", "las", "pcd"):
                candidate = files.get(candidate_key)
                if candidate is not None and candidate.exists():
                    self.open_filter_viewer(str(candidate))
                    return
            QMessageBox.information(
                self,
                "Filtering",
                "No LAS or point cloud file is available yet for filtering. Run rosbag preprocessing first.",
            )
            return

        if step_key == "flai":
            QMessageBox.information(
                self,
                "FLAI",
                "FLAI is not automated in this experimental branch yet. Import or place the segmented LAS file into "
                "the scan folder, then open the semantic viewer to inspect the current scan outputs.",
            )
            return

        QMessageBox.information(self, "Pipeline", f"No GUI pipeline action is defined for step '{step_key}'.")

    def _on_pipeline_log(self, line: str):
        if line:
            print(line)

    def _on_pipeline_stage(self, step_key: str, status: str):
        stage_names = {
            "slam": "Rosbag Preprocessing",
            "filtering": "Filtering",
            "flai": "FLAI",
            "wire_extraction": "Wire Extraction",
            "inference": "Image Inference",
            "fusion": "Fusion + GPS",
        }
        message = f"{stage_names.get(step_key, step_key)}: {status}"
        self.auto_dashboard.status_label.setText(message)
        self.step_dashboard.status_label.setText(message)
        self.auto_dashboard.add_notification(message, "running" if status == "running" else "done")
        self.step_dashboard.add_notification(message, "running" if status == "running" else "done")
        self.refresh_dashboards()

    def _on_pipeline_complete(self, scan_dir_str: str):
        scan_dir = Path(scan_dir_str)
        self.auto_dashboard.add_notification(f"Pipeline complete for {scan_dir.name}", "done")
        self.step_dashboard.add_notification(f"Pipeline complete for {scan_dir.name}", "done")
        self.refresh_dashboards()
        self.open_semantic_viewer(scan_dir)

    def _on_pipeline_failed(self, message: str):
        self.auto_dashboard.add_notification(message, "error")
        self.step_dashboard.add_notification(message, "error")
        self.refresh_dashboards()
        QMessageBox.warning(self, "Pipeline", message)

    def run_calibration_workflow(self, dataset_path: str, run_name: str, stop_after: str):
        if self._calibration_thread is not None and self._calibration_thread.isRunning():
            QMessageBox.information(self, "Calibration Busy", "A calibration workflow is already running.")
            return

        self.calibration_view.set_status(f"Running calibration workflow: {run_name}")
        self.calibration_view.append_log(f"[calibration] dataset={dataset_path}")
        self.calibration_view.append_log(f"[calibration] run_name={run_name}")
        self._calibration_thread = CalibrationWorkflowThread(
            dataset_path,
            run_name=run_name,
            stop_after=stop_after or None,
            parent=self,
        )
        self._calibration_thread.logLine.connect(self.calibration_view.append_log)
        self._calibration_thread.completed.connect(self._on_calibration_complete)
        self._calibration_thread.failed.connect(self._on_calibration_failed)
        self._calibration_thread.start()

    def _on_calibration_complete(self, run_name: str):
        self.calibration_view.set_status(f"Calibration workflow complete: {run_name}")
        latest_run = newest_calibration_run()
        if latest_run is not None:
            self.calibration_view.append_log(f"[calibration] latest output: {latest_run}")

    def _on_calibration_failed(self, message: str):
        self.calibration_view.set_status("Calibration workflow failed")
        self.calibration_view.append_log(message)
        QMessageBox.warning(self, "Calibration", message)

    def open_filter_viewer(self, filepath):
        if not filepath or not Path(filepath).exists():
            QMessageBox.information(self, "Filter Viewer", "The requested filter input file does not exist yet.")
            return
        self.filter_viewer.filename = filepath
        if hasattr(self.filter_viewer, "load_point_cloud"):
            self.filter_viewer.load_point_cloud()
        elif hasattr(self.filter_viewer, "load_las_file"):
            self.filter_viewer.load_las_file()
        self._show_temporary_view(self.filter_viewer)

    def open_semantic_viewer(self, scan_ref: str | Path):
        scan_dir, _ = load_scan_metadata(scan_ref)
        self.semantic_viewer.initialize_viewer(scan_dir)
        self._show_temporary_view(self.semantic_viewer)

    def open_map_for_scan(self, scan_ref: str | Path, metadata_override: dict | None = None):
        scan_dir, metadata = load_scan_metadata(scan_ref)
        if metadata_override:
            metadata = metadata_override
        artifacts = resolve_artifact_paths(scan_dir, metadata)
        objects_json = artifacts.get("georeferenced_objects") or artifacts.get("fused_objects")
        powerlines_json = artifacts.get("georeferenced_powerline_overlay") or artifacts.get("powerline_overlay")

        if (objects_json is None or not objects_json.exists()) and (powerlines_json is None or not powerlines_json.exists()):
            QMessageBox.information(
                self,
                "Map Not Ready",
                "This scan does not yet have Fusion objects or exported powerline overlays to display on the map.",
            )
            return

        url = self._leaflet_manager.open_map(objects_json=objects_json, powerlines_json=powerlines_json)
        self.auto_dashboard.add_notification(f"Opened map for {scan_dir.name}", "info")
        self.step_dashboard.add_notification(f"Opened map for {scan_dir.name}", "info")
        print(f"Leaflet URL: {url}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DashboardTestWindow()
    window.show()
    sys.exit(app.exec())
