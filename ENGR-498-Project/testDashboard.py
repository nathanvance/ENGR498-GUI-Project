import json
import os
import subprocess
import shutil
import sys
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QThread, Signal
from PySide6.QtWidgets import QApplication, QFileDialog, QMainWindow, QMessageBox, QStackedWidget, QToolBar

from calibration_bridge import CALIBRATION_OUTPUT_ROOT, newest_calibration_run, resolve_calibration_run
from gui_pipeline import (
    BackendPipelineThread,
    CalibrationPreflightThread,
    CalibrationWorkflowThread,
    LeafletServerManager,
)
from project_paths import ASSETS_DIR, PROJECT_ROOT
from scan_metadata import (
    ensure_scan_structure,
    first_existing_artifact,
    load_scan_metadata,
    relativize_for_scan,
    resolve_artifact_paths,
    resolve_scan_path,
    save_scan_metadata,
)
from timing_settings import load_global_timing_settings, save_global_timing_settings
from views.calibration_mode import CalibrationModeView
from views.lidar_dashboard import DashboardView
from views.lidar_dashboard_stepbystep import GlobalTimingSettingsDialog, StepByStepDashboard


class ScanImportThread(QThread):
    completed = Signal(str, str, str)  # scan_name, bag_filename, destination
    failed = Signal(str, str)  # scan_name, message

    def __init__(self, scan_name: str, source_bag: str | Path, destination_bag: str | Path, parent=None):
        super().__init__(parent)
        self.scan_name = scan_name
        self.source_bag = Path(source_bag)
        self.destination_bag = Path(destination_bag)

    def run(self) -> None:
        try:
            self.destination_bag.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(self.source_bag, self.destination_bag)
            self.completed.emit(self.scan_name, self.source_bag.name, str(self.destination_bag))
        except Exception as exc:  # pragma: no cover - GUI-facing error path
            self.failed.emit(self.scan_name, str(exc))


class DashboardTestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiDAR Processing Dashboard")
        self.resize(1680, 960)

        self._leaflet_manager = LeafletServerManager()
        self._pipeline_thread: BackendPipelineThread | None = None
        self._calibration_thread: CalibrationWorkflowThread | None = None
        self._calibration_preflight_thread: CalibrationPreflightThread | None = None
        self._scan_import_thread: ScanImportThread | None = None
        self._previous_widget = None

        self.setup_demo_assets()

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)
        self._setup_mode_toolbar()

        self.auto_dashboard = DashboardView(assets_path=str(ASSETS_DIR))
        self.step_dashboard = StepByStepDashboard(assets_path=str(ASSETS_DIR))
        self.calibration_view = CalibrationModeView(default_output_root=CALIBRATION_OUTPUT_ROOT)
        self.filter_viewer = None
        self.semantic_viewer = None

        self.stack.addWidget(self.auto_dashboard)   # index 0
        self.stack.addWidget(self.step_dashboard)   # index 1
        self.stack.addWidget(self.calibration_view) # index 2

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

        timing = toolbar.addAction("Timing Calibration")
        timing.triggered.connect(self.open_global_timing_dialog)

    def open_global_timing_dialog(self):
        settings = load_global_timing_settings()
        dialog = GlobalTimingSettingsDialog(current_timing=settings.get("timing", {}), parent=self)
        if dialog.exec():
            settings["timing"] = dialog.get_timing_settings()
            save_global_timing_settings(settings)
            self.auto_dashboard.add_notification("Updated global timing calibration defaults", "done")
            self.step_dashboard.add_notification("Updated global timing calibration defaults", "done")

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
        self.auto_dashboard.openSlamPointCloudRequested.connect(self.open_pose_recovery_point_cloud)
        self.step_dashboard.openSlamPointCloudRequested.connect(self.open_pose_recovery_point_cloud)
        self.auto_dashboard.openImagesFolderRequested.connect(self.open_images_folder)
        self.step_dashboard.openImagesFolderRequested.connect(self.open_images_folder)

        self.auto_dashboard.openMapRequested.connect(self.open_map_for_scan)
        self.step_dashboard.openMapRequested.connect(self.open_map_for_scan)

        self.step_dashboard.openFilterViewerRequested.connect(self.open_filter_viewer)
        self.calibration_view.runCalibrationRequested.connect(self.run_calibration_workflow)
        self.calibration_view.checkRequirementsRequested.connect(self.run_calibration_requirements_check)

    def _get_filter_viewer(self):
        if self.filter_viewer is None:
            from PreProcessing_GUI.point_cloud_filter_gui import PointCloudFilterViewer

            self.filter_viewer = PointCloudFilterViewer(filename=None)
            self.filter_viewer.backRequested.connect(self._restore_previous_widget)
            self.stack.addWidget(self.filter_viewer)
        return self.filter_viewer

    def _get_semantic_viewer(self):
        if self.semantic_viewer is None:
            from Matlab_ExtractPowerLine.testSemanticLidarViewer import SemanticViewer as CombinedSemanticViewer

            self.semantic_viewer = CombinedSemanticViewer()
            self.semantic_viewer.set_open_map_callback(self.open_map_for_scan)
            self.semantic_viewer.backRequested.connect(self._restore_previous_widget)
            self.stack.addWidget(self.semantic_viewer)
        return self.semantic_viewer

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
        for scan_dir in ASSETS_DIR.iterdir():
            if scan_dir.is_dir():
                try:
                    scan_path, metadata = load_scan_metadata(scan_dir)
                    save_scan_metadata(scan_path, metadata)
                except Exception:
                    pass

    def refresh_dashboards(self):
        self.auto_dashboard.refresh_scans()
        self.step_dashboard.refresh_scans()

    def refresh_dashboards_for_scan(self, scan_name: str | None = None):
        self.refresh_dashboards()
        if scan_name:
            self.auto_dashboard.select_scan(scan_name)
            self.step_dashboard.select_scan(scan_name)

    def _scan_import_running(self) -> bool:
        return self._scan_import_thread is not None and self._scan_import_thread.isRunning()

    def create_new_scan(self):
        if self._scan_import_running():
            QMessageBox.information(self, "Scan Import Busy", "A rosbag import is already running.")
            return

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

        metadata = {
            "name": scan_name,
            "created_at": datetime.now().strftime("%Y-%m-%d"),
            "notes": f"Importing rosbag: {bag_filename}",
            "files": {},
        }
        save_scan_metadata(scan_dir, metadata)
        self.refresh_dashboards_for_scan(scan_name)
        self.auto_dashboard.add_notification(f"Importing {bag_filename} into {scan_name}", "running")
        self.step_dashboard.add_notification(f"Importing {bag_filename} into {scan_name}", "running")
        self.auto_dashboard.status_label.setText(f"Importing {scan_name}...")
        self.step_dashboard.status_label.setText(f"Importing {scan_name}...")

        self._scan_import_thread = ScanImportThread(scan_name, bag_path, destination_bag, parent=self)
        self._scan_import_thread.completed.connect(self._on_scan_import_complete)
        self._scan_import_thread.failed.connect(self._on_scan_import_failed)
        self._scan_import_thread.start()

    def _on_scan_import_complete(self, scan_name: str, bag_filename: str, destination_bag: str):
        scan_dir, metadata = load_scan_metadata(ASSETS_DIR / scan_name)
        metadata["notes"] = ""
        metadata.setdefault("files", {})
        metadata["files"]["rosbag"] = f"raw/{bag_filename}"
        save_scan_metadata(scan_dir, metadata)
        self.refresh_dashboards_for_scan(scan_name)
        self.auto_dashboard.add_notification(f"Created {scan_name}", "done")
        self.step_dashboard.add_notification(f"Created {scan_name}", "done")
        QMessageBox.information(
            self,
            "Scan Created",
            f"Created {scan_name}\n\nRosbag copied to:\n{destination_bag}",
        )

    def _on_scan_import_failed(self, scan_name: str, message: str):
        try:
            shutil.rmtree(ASSETS_DIR / scan_name)
        except Exception:
            pass
        self.refresh_dashboards()
        self.auto_dashboard.add_notification(f"Failed to create {scan_name}", "error")
        self.step_dashboard.add_notification(f"Failed to create {scan_name}", "error")
        QMessageBox.warning(self, "Scan Import Failed", f"Failed to import rosbag for {scan_name}:\n{message}")

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

        self.auto_dashboard.clear_log()
        self.step_dashboard.clear_log()
        self._on_pipeline_log(f"[pipeline] Starting {mode} for {scan_dir.name}")

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
            _, candidate = first_existing_artifact(
                scan_dir,
                metadata,
                ("filtered_las", "raw_las", "filtered_pcd", "raw_pcd", "las", "pcd"),
            )
            if candidate is not None:
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
            self.auto_dashboard.append_log(line)
            self.step_dashboard.append_log(line)
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
        self._on_pipeline_log(f"[stage] {message}")
        self.refresh_dashboards()

    def _build_pipeline_output_summary(self, scan_dir: Path, mode: str) -> tuple[str, str]:
        _, metadata = load_scan_metadata(scan_dir)
        artifacts = resolve_artifact_paths(scan_dir, metadata)

        def _add_path(lines: list[str], label: str, key: str):
            path = artifacts.get(key)
            if path is not None and path.exists():
                lines.append(f"{label}: {path}")

        lines: list[str] = []
        title_by_mode = {
            "pose-recovery": "Rosbag Preprocessing Complete",
            "wire-extraction": "Wire Extraction Complete",
            "inference": "Image Inference Complete",
            "fusion": "Fusion + GPS Complete",
            "full": "Full Backend Pipeline Complete",
        }
        title = title_by_mode.get(mode, "Pipeline Complete")

        if mode == "pose-recovery":
            _add_path(lines, "Pose recovery run", "latest_pose_recovery_run")
            _add_path(lines, "Raw SLAM PCD", "raw_pcd")
            _add_path(lines, "Raw SLAM LAS", "raw_las")
            _add_path(lines, "Images folder", "images_dir")
            _add_path(lines, "Image timestamps CSV", "image_timestamps_csv")
            _add_path(lines, "Camera TF CSV", "tf_camera_csv")
            _add_path(lines, "GPS TF CSV", "tf_gps_csv")
        elif mode == "wire-extraction":
            _add_path(lines, "Wire points", "wires_points")
            _add_path(lines, "Wire info JSON", "wire_info")
            _add_path(lines, "Ground points", "ground_points")
            _add_path(lines, "Powerline overlay", "powerline_overlay")
            _add_path(lines, "Georeferenced overlay", "georeferenced_powerline_overlay")
        elif mode == "inference":
            _add_path(lines, "Inference output", "yolo_output_dir")
            _add_path(lines, "Annotated prediction images", "pred_images_dir")
            _add_path(lines, "Masks directory", "masks_dir")
            _add_path(lines, "Metadata directory", "meta_dir")
        elif mode == "fusion":
            _add_path(lines, "Fusion objects", "fused_objects")
            _add_path(lines, "Fused semantic map", "fused_map")
            _add_path(lines, "Pole spacing JSON", "pole_neighbor_distances")
            _add_path(lines, "GPS alignment", "gps_alignment")
            _add_path(lines, "Georeferenced objects", "georeferenced_objects")
            _add_path(lines, "Georeferenced powerlines", "georeferenced_powerline_overlay")
        else:
            _add_path(lines, "Pose recovery run", "latest_pose_recovery_run")
            _add_path(lines, "Raw SLAM PCD", "raw_pcd")
            _add_path(lines, "Raw SLAM LAS", "raw_las")
            _add_path(lines, "Filtered LAS", "filtered_las")
            _add_path(lines, "Filtered PCD", "filtered_pcd")
            _add_path(lines, "Images folder", "images_dir")
            _add_path(lines, "Wire points", "wires_points")
            _add_path(lines, "Inference output", "yolo_output_dir")
            _add_path(lines, "Fusion objects", "fused_objects")
            _add_path(lines, "Fused semantic map", "fused_map")
            _add_path(lines, "Georeferenced objects", "georeferenced_objects")
            _add_path(lines, "Georeferenced powerlines", "georeferenced_powerline_overlay")

        if not lines:
            lines.append(f"No output paths were recorded for {scan_dir.name}.")

        summary = f"{title} for {scan_dir.name}\n\n" + "\n".join(lines)
        return title, summary

    def _on_pipeline_complete(self, scan_dir_str: str, mode: str):
        scan_dir = Path(scan_dir_str)
        title, summary = self._build_pipeline_output_summary(scan_dir, mode)
        self.auto_dashboard.add_notification(f"{title} for {scan_dir.name}", "done")
        self.step_dashboard.add_notification(f"{title} for {scan_dir.name}", "done")
        self.refresh_dashboards()
        self.auto_dashboard.status_label.setText(f"{title} for {scan_dir.name}")
        self.step_dashboard.status_label.setText(f"{title} for {scan_dir.name}")
        for line in summary.splitlines():
            self._on_pipeline_log(line)
        self._pipeline_thread = None
        QMessageBox.information(self, title, summary)

    def _on_pipeline_failed(self, message: str):
        self.auto_dashboard.add_notification(message, "error")
        self.step_dashboard.add_notification(message, "error")
        self._on_pipeline_log(f"[error] {message}")
        self.refresh_dashboards()
        self._pipeline_thread = None
        QMessageBox.warning(self, "Pipeline", message)

    def _calibration_job_running(self) -> bool:
        return (
            (self._calibration_thread is not None and self._calibration_thread.isRunning())
            or (
                self._calibration_preflight_thread is not None
                and self._calibration_preflight_thread.isRunning()
            )
        )

    def run_calibration_requirements_check(self):
        if self._calibration_job_running():
            QMessageBox.information(self, "Calibration Busy", "A calibration requirements check or workflow is already running.")
            return

        self.calibration_view.set_status("Checking calibration requirements...")
        self.calibration_view.set_requirements_status(
            "Checking WSL, Docker, WSLg, and hardware-backed OpenGL support...",
            ok=None,
        )
        self.calibration_view.append_log("[preflight] Starting calibration requirements check...")
        self._calibration_preflight_thread = CalibrationPreflightThread(parent=self)
        self._calibration_preflight_thread.logLine.connect(self.calibration_view.append_log)
        self._calibration_preflight_thread.completed.connect(self._on_calibration_requirements_complete)
        self._calibration_preflight_thread.failed.connect(self._on_calibration_requirements_failed)
        self._calibration_preflight_thread.start()

    def run_calibration_workflow(self, dataset_path: str, run_name: str, stop_after: str):
        if self._calibration_job_running():
            QMessageBox.information(self, "Calibration Busy", "A calibration requirements check or workflow is already running.")
            return

        self.calibration_view.set_status(f"Running calibration workflow: {run_name}")
        self.calibration_view.set_requirements_status(
            "Calibration launch will verify requirements before opening the calibration GUI.",
            ok=None,
        )
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

    def _on_calibration_requirements_complete(self, summary: str):
        self.calibration_view.set_status("Calibration requirements check passed")
        self.calibration_view.set_requirements_status(summary, ok=True)

    def _on_calibration_requirements_failed(self, message: str):
        self.calibration_view.set_status("Calibration requirements check failed")
        self.calibration_view.set_requirements_status(message, ok=False)
        QMessageBox.warning(self, "Calibration Requirements", message)

    def _on_calibration_complete(self, run_name: str):
        self.calibration_view.set_status(f"Calibration workflow complete: {run_name}")
        self.calibration_view.set_requirements_status(
            "Calibration launch passed the hardware-backed OpenGL requirements check.",
            ok=True,
        )
        latest_run = newest_calibration_run()
        if latest_run is not None:
            self.calibration_view.append_log(f"[calibration] latest output: {latest_run}")

    def _on_calibration_failed(self, message: str):
        self.calibration_view.set_status("Calibration workflow failed")
        if "requirements" in message.lower() or "opengl" in message.lower() or "software" in message.lower():
            self.calibration_view.set_requirements_status(message, ok=False)
        self.calibration_view.append_log(message)
        QMessageBox.warning(self, "Calibration", message)

    def open_filter_viewer(self, filepath):
        if not filepath or not Path(filepath).exists():
            QMessageBox.information(self, "Filter Viewer", "The requested filter input file does not exist yet.")
            return
        filter_viewer = self._get_filter_viewer()
        filter_viewer.filename = filepath
        if hasattr(filter_viewer, "load_point_cloud"):
            filter_viewer.load_point_cloud()
        elif hasattr(filter_viewer, "load_las_file"):
            filter_viewer.load_las_file()
        self._show_temporary_view(filter_viewer)

    def open_semantic_viewer(self, scan_ref: str | Path):
        scan_dir, _ = load_scan_metadata(scan_ref)
        semantic_viewer = self._get_semantic_viewer()
        semantic_viewer.initialize_viewer(scan_dir)
        self._show_temporary_view(semantic_viewer)

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

    def open_pose_recovery_point_cloud(self, point_cloud_path: str):
        point_cloud = Path(point_cloud_path)
        if not point_cloud.is_file():
            QMessageBox.information(self, "SLAM Point Cloud", "The requested SLAM point cloud does not exist yet.")
            return

        viewer_script = PROJECT_ROOT / "view_pcd_open3d.py"
        if not viewer_script.is_file():
            QMessageBox.warning(self, "SLAM Point Cloud", f"Viewer script not found:\n{viewer_script}")
            return

        subprocess.Popen([sys.executable, str(viewer_script), str(point_cloud)])
        self.auto_dashboard.add_notification(f"Opened SLAM point cloud viewer for {point_cloud.name}", "info")
        self.step_dashboard.add_notification(f"Opened SLAM point cloud viewer for {point_cloud.name}", "info")

    def open_images_folder(self, images_dir_path: str):
        images_dir = Path(images_dir_path)
        if not images_dir.is_dir():
            QMessageBox.information(self, "Images Folder", "The exported images folder does not exist yet.")
            return

        if os.name == "nt":
            os.startfile(str(images_dir))
        elif sys.platform == "darwin":
            subprocess.Popen(["open", str(images_dir)])
        else:
            subprocess.Popen(["xdg-open", str(images_dir)])

        self.auto_dashboard.add_notification(f"Opened images folder for {images_dir.name}", "info")
        self.step_dashboard.add_notification(f"Opened images folder for {images_dir.name}", "info")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DashboardTestWindow()
    window.show()
    sys.exit(app.exec())
