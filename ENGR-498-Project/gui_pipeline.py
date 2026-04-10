from __future__ import annotations

import os
import shutil
import socket
import subprocess
import sys
import json
import urllib.parse
import webbrowser
from pathlib import Path

from PySide6.QtCore import QThread, Signal

from calibration_bridge import export_fusion_calibration_artifacts, resolve_calibration_run
from project_paths import FUSION_DIR, MATLAB_EXTRACT_DIR, PROJECT_ROOT, ROSBAG_PREPROCESSING_DIR
from scan_metadata import (
    first_existing_artifact,
    load_scan_metadata,
    newest_directory,
    resolve_artifact_paths,
    resolve_scan_path,
    save_scan_metadata,
    update_file_entry,
    update_status,
)
from timing_settings import resolve_effective_timing


RUN_POSE_RECOVERY_SCRIPT = ROSBAG_PREPROCESSING_DIR / "launcher" / "run_transform_reading_workflow.py"
RUN_CALIBRATION_SCRIPT = ROSBAG_PREPROCESSING_DIR / "launcher" / "run_calibration_workflow.py"
RUN_YOLO_SCRIPT = FUSION_DIR / "run_yolo_inference.py"
RUN_FUSION_SCRIPT = FUSION_DIR / "fuse_masks_to_slam.py"
RUN_GEOREF_SCRIPT = FUSION_DIR / "georeference_from_tf_gps.py"
RUN_POWERLINE_EXPORT_SCRIPT = FUSION_DIR / "export_powerlines_to_leaflet.py"
RUN_WIRE_EXTRACTION_SCRIPT = MATLAB_EXTRACT_DIR / "testMatlab.py"


class ColabFallbackRequiredError(RuntimeError):
    """Raised when inference packaged a Colab bundle instead of local masks/meta outputs."""


def _is_port_open(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.3)
        return sock.connect_ex((host, port)) == 0


def _extract_prefixed_summary(lines: list[str], prefix: str) -> str | None:
    for line in lines:
        if line.startswith(prefix):
            return line[len(prefix) :].strip()
    return None


def _summarize_calibration_failure(lines: list[str], return_code: int, *, context: str) -> str:
    preflight_summary = _extract_prefixed_summary(lines, "[preflight] FAIL:")
    if preflight_summary:
        return f"{context} failed. {preflight_summary}"

    for line in reversed(lines):
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("[preflight]"):
            continue
        if stripped.startswith("ERROR:"):
            return stripped
        if "failed" in stripped.lower():
            return stripped

    return f"{context} failed with exit code {return_code}."


def convert_point_cloud_to_las(source_path: Path, output_path: Path) -> Path:
    import laspy
    import numpy as np
    import open3d as o3d

    cloud = o3d.io.read_point_cloud(str(source_path))
    xyz = np.asarray(cloud.points, dtype=np.float64)
    if xyz.size == 0:
        raise ValueError(f"Point cloud is empty and cannot be converted to LAS: {source_path}")

    colors = np.asarray(cloud.colors, dtype=np.float64) if cloud.has_colors() else None
    point_format = 3 if colors is not None and colors.size else 0

    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = laspy.LasHeader(point_format=point_format, version="1.2")
    header.offsets = xyz.min(axis=0)
    header.scales = np.array([0.001, 0.001, 0.001], dtype=np.float64)

    las = laspy.LasData(header)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    if point_format == 3 and colors is not None and colors.size:
        clipped = np.clip(colors, 0.0, 1.0)
        las.red = (clipped[:, 0] * 65535).astype(np.uint16)
        las.green = (clipped[:, 1] * 65535).astype(np.uint16)
        las.blue = (clipped[:, 2] * 65535).astype(np.uint16)
    las.write(str(output_path))
    return output_path


class LeafletServerManager:
    def __init__(self, port: int = 8765) -> None:
        self.port = port
        self._process: subprocess.Popen[str] | None = None

    def ensure_running(self) -> None:
        if _is_port_open("127.0.0.1", self.port):
            return
        if self._process is not None and self._process.poll() is None:
            return

        creationflags = 0
        if os.name == "nt":
            creationflags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)

        self._process = subprocess.Popen(
            [sys.executable, "-m", "http.server", str(self.port)],
            cwd=str(PROJECT_ROOT),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            creationflags=creationflags,
        )

    def open_map(self, *, objects_json: Path | None = None, powerlines_json: Path | None = None) -> str:
        self.ensure_running()
        params: dict[str, str] = {}
        params["_viewer"] = "20260409a"

        if objects_json is not None and objects_json.exists():
            params["data"] = "/" + os.path.relpath(objects_json, PROJECT_ROOT).replace("\\", "/")
        if powerlines_json is not None and powerlines_json.exists():
            params["powerlines"] = "/" + os.path.relpath(powerlines_json, PROJECT_ROOT).replace("\\", "/")

        query = urllib.parse.urlencode(params)
        url = f"http://localhost:{self.port}/fusion/leaflet_viewer/index.html"
        if query:
            url = f"{url}?{query}"
        webbrowser.open(url)
        return url

    def shutdown(self) -> None:
        if self._process is None:
            return
        if self._process.poll() is None:
            self._process.terminate()
        self._process = None


class CalibrationWorkflowThread(QThread):
    logLine = Signal(str)
    completed = Signal(str)
    failed = Signal(str)

    def __init__(self, dataset_path: str | Path, *, run_name: str, stop_after: str | None = None, parent=None) -> None:
        super().__init__(parent)
        self.dataset_path = Path(dataset_path)
        self.run_name = run_name
        self.stop_after = stop_after

    def run(self) -> None:
        command = [
            sys.executable,
            str(RUN_CALIBRATION_SCRIPT),
            str(self.dataset_path),
            "--run-name",
            self.run_name,
        ]
        if self.stop_after:
            command.extend(["--stop-after", self.stop_after])

        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        assert process.stdout is not None
        lines: list[str] = []
        for line in process.stdout:
            stripped = line.rstrip()
            lines.append(stripped)
            self.logLine.emit(stripped)
        return_code = process.wait()
        if return_code == 0:
            self.completed.emit(self.run_name)
        else:
            self.failed.emit(_summarize_calibration_failure(lines, return_code, context="Calibration workflow"))


class CalibrationPreflightThread(QThread):
    logLine = Signal(str)
    completed = Signal(str)
    failed = Signal(str)

    def run(self) -> None:
        command = [
            sys.executable,
            str(RUN_CALIBRATION_SCRIPT),
            "--check-only",
        ]
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        assert process.stdout is not None
        lines: list[str] = []
        for line in process.stdout:
            stripped = line.rstrip()
            lines.append(stripped)
            self.logLine.emit(stripped)

        return_code = process.wait()
        if return_code == 0:
            summary = _extract_prefixed_summary(lines, "[preflight] PASS:") or "Calibration requirements check passed."
            self.completed.emit(summary)
        else:
            self.failed.emit(_summarize_calibration_failure(lines, return_code, context="Calibration requirements check"))


class BackendPipelineThread(QThread):
    logLine = Signal(str)
    stageChanged = Signal(str, str)  # stage_key, status
    completed = Signal(str, str)
    failed = Signal(str)

    def __init__(self, scan_ref: str | Path, *, mode: str = "full", parent=None) -> None:
        super().__init__(parent)
        self.scan_ref = str(scan_ref)
        self.mode = mode

    def emit_log(self, text: str) -> None:
        self.logLine.emit(text.rstrip())

    def run_command(self, command: list[str], *, cwd: Path | None = None) -> None:
        self.emit_log(f"[cmd] {' '.join(command)}")
        process = subprocess.Popen(
            command,
            cwd=str(cwd) if cwd else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        assert process.stdout is not None
        for line in process.stdout:
            self.emit_log(line.rstrip())
        return_code = process.wait()
        if return_code != 0:
            raise RuntimeError(f"Command failed with exit code {return_code}: {' '.join(command)}")

    def _run_pose_recovery(self, scan_dir: Path, metadata: dict) -> Path:
        pose_cfg = metadata.get("config", {}).get("pose_recovery", {})
        effective_timing = resolve_effective_timing(metadata)
        preprocess_offset_enabled = bool(effective_timing.get("preprocess_camera_offset_enabled", False))
        preprocess_offset_sec = (
            float(effective_timing.get("preprocess_camera_offset_sec", 0.0))
            if preprocess_offset_enabled
            else 0.0
        )
        files = metadata["files"]
        bag_path = resolve_scan_path(scan_dir, files.get("rosbag"))
        if bag_path is None or not bag_path.is_file():
            raw_bags = sorted((scan_dir / "raw").glob("*.bag"))
            if not raw_bags:
                raise FileNotFoundError(f"No rosbag found for scan {scan_dir.name}")
            bag_path = raw_bags[0]
            update_file_entry(metadata, scan_dir, "rosbag", bag_path)

        output_root = scan_dir / "processed" / "pose_recovery"
        output_root.mkdir(parents=True, exist_ok=True)

        command = [
            sys.executable,
            str(RUN_POSE_RECOVERY_SCRIPT),
            str(bag_path),
            "--output-root",
            str(output_root),
        ]
        if pose_cfg.get("enable_rviz"):
            command.append("--rviz")
        command.extend(["--camera-time-offset-sec", str(preprocess_offset_sec)])
        self.emit_log(
            "[timing] pose-recovery camera offset "
            f"{'enabled' if preprocess_offset_enabled else 'disabled'} "
            f"(effective={preprocess_offset_sec:.6f} sec)"
        )
        self.run_command(command)

        latest_run = newest_directory(output_root)
        if latest_run is None:
            raise FileNotFoundError(f"Rosbag preprocessing did not create an output run in {output_root}")

        update_file_entry(metadata, scan_dir, "pose_recovery_root", output_root)
        update_file_entry(metadata, scan_dir, "latest_pose_recovery_run", latest_run)
        update_file_entry(metadata, scan_dir, "images_dir", latest_run / "images")
        update_file_entry(metadata, scan_dir, "image_timestamps_csv", latest_run / "image_timestamps.csv")
        update_file_entry(metadata, scan_dir, "tf_camera_csv", latest_run / "tf_camera_out.csv")
        update_file_entry(metadata, scan_dir, "tf_gps_csv", latest_run / "tf_gps_out.csv")

        timing_config_path = latest_run / "timing_config.json"
        timing_config_path.write_text(
            json.dumps(
                {
                    "timing": effective_timing,
                    "effective": {
                        "preprocess_camera_offset_sec": preprocess_offset_sec,
                    },
                },
                indent=2,
            ),
            encoding="utf-8",
        )

        pose_cfg_store = metadata.setdefault("config", {}).setdefault("pose_recovery", {})
        prev_applied = float(pose_cfg_store.get("last_camera_time_offset_sec", 0.0))
        prev_enabled = bool(pose_cfg_store.get("last_camera_time_offset_enabled", False))
        if prev_enabled != preprocess_offset_enabled or abs(prev_applied - preprocess_offset_sec) > 1e-12:
            update_status(metadata, "inference", "pending")
            update_status(metadata, "fusion", "pending")
            self.emit_log("[timing] preprocessing camera offset changed; marked inference and fusion as pending.")
        pose_cfg_store["last_camera_time_offset_sec"] = preprocess_offset_sec
        pose_cfg_store["last_camera_time_offset_enabled"] = preprocess_offset_enabled

        # Copy the raw SLAM PCD into processed/point_clouds/raw/ and generate a LAS
        # conversion alongside it. The pose_recovery output stays untouched as
        # the source-of-truth raw export.
        raw_pcd = latest_run / "pcd" / "scans.pcd"
        raw_dir = scan_dir / "processed" / "point_clouds" / "raw"
        raw_dir.mkdir(parents=True, exist_ok=True)
        working_pcd = raw_dir / "scans.pcd"
        working_las = raw_dir / "cloud.las"
        if raw_pcd.is_file():
            shutil.copy2(str(raw_pcd), str(working_pcd))
            convert_point_cloud_to_las(working_pcd, working_las)
        update_file_entry(metadata, scan_dir, "raw_pcd", working_pcd)
        update_file_entry(metadata, scan_dir, "raw_las", working_las)
        update_file_entry(metadata, scan_dir, "pcd", working_pcd)
        update_file_entry(metadata, scan_dir, "las", working_las)

        update_status(metadata, "slam", "done")
        save_scan_metadata(scan_dir, metadata)
        self.emit_log(f"[outputs] pose recovery run: {latest_run}")
        self.emit_log(f"[outputs] slam point cloud: {latest_run / 'pcd' / 'scans.pcd'}")
        self.emit_log(f"[outputs] working point cloud: {working_pcd}")
        self.emit_log(f"[outputs] working LAS: {working_las}")
        self.emit_log(f"[outputs] image folder: {latest_run / 'images'}")
        self.emit_log(f"[outputs] camera poses: {latest_run / 'tf_camera_out.csv'}")
        self.emit_log(f"[outputs] gps poses: {latest_run / 'tf_gps_out.csv'}")
        return latest_run

    def _resolve_wire_input_las(self, scan_dir: Path, metadata: dict) -> Path:
        point_clouds_dir = scan_dir / "processed" / "point_clouds"
        key, candidate = first_existing_artifact(
            scan_dir,
            metadata,
            ("segmented", "filtered_las", "raw_las", "filtered_pcd", "raw_pcd"),
        )
        if candidate is None or key is None:
            raise FileNotFoundError(
                "Wire extraction requires an input point cloud. Expected one of: segmented LAS, "
                "filtered LAS, raw LAS, filtered PCD, or raw PCD."
            )

        suffix = candidate.suffix.lower()
        if suffix in {".las", ".laz"}:
            return candidate

        if suffix in {".pcd", ".ply"}:
            las_name = "cloud_filtered.las" if key == "filtered_pcd" else "cloud.las"
            exported = convert_point_cloud_to_las(candidate, point_clouds_dir / las_name)
            target_key = "filtered_las" if key == "filtered_pcd" else "raw_las"
            update_file_entry(metadata, scan_dir, target_key, exported)
            if target_key == "raw_las":
                update_file_entry(metadata, scan_dir, "las", exported)
            save_scan_metadata(scan_dir, metadata)
            return exported

        raise FileNotFoundError(f"Unsupported wire-extraction point cloud type: {candidate}")

    def _export_powerline_overlay(self, scan_dir: Path, metadata: dict) -> Path | None:
        artifacts = resolve_artifact_paths(scan_dir, metadata)
        wires_npz = artifacts.get("wires_points")
        if wires_npz is None or not wires_npz.is_file():
            return None

        wire_info = artifacts.get("wire_info")
        ground_points = artifacts.get("ground_points")
        powerline_overlay = scan_dir / "processed" / "wires" / "eng498_powerlines_overlay.json"

        powerline_command = [
            sys.executable,
            str(RUN_POWERLINE_EXPORT_SCRIPT),
            "--wires-npz",
            str(wires_npz),
            "--output-json",
            str(powerline_overlay),
        ]
        if wire_info is not None and wire_info.is_file():
            powerline_command.extend(["--wire-info-json", str(wire_info)])
        if ground_points is not None and ground_points.is_file():
            powerline_command.extend(["--ground-points-npz", str(ground_points)])
        self.run_command(powerline_command)
        update_file_entry(metadata, scan_dir, "powerline_overlay", powerline_overlay)
        save_scan_metadata(scan_dir, metadata)
        return powerline_overlay

    def _maybe_georeference(self, scan_dir: Path, metadata: dict, *, tf_gps_path: Path | None, objects_json: Path | None = None, powerline_overlay: Path | None = None) -> None:
        if tf_gps_path is None or not tf_gps_path.is_file():
            return
        if (objects_json is None or not objects_json.is_file()) and (powerline_overlay is None or not powerline_overlay.is_file()):
            return

        config = metadata.get("config", {})
        gps_cfg = config.get("gps", {})
        gps_output_dir = scan_dir / "processed" / "fusion" / "gps"
        gps_output_dir.mkdir(parents=True, exist_ok=True)
        georef_command = [
            sys.executable,
            str(RUN_GEOREF_SCRIPT),
            "--tf-gps-csv",
            str(tf_gps_path),
            "--output-dir",
            str(gps_output_dir),
            "--gps-to-lidar-offset-body",
            str(config.get("gps", {}).get("offset_body_xyz_m", "0,0,0")),
            "--max-horizontal-cov-m2",
            str(gps_cfg.get("max_horizontal_cov_m2", 1000.0)),
        ]
        if objects_json is not None and objects_json.is_file():
            georef_command.extend(["--objects-json", str(objects_json)])
        if powerline_overlay is not None and powerline_overlay.is_file():
            georef_command.extend(["--powerlines-json", str(powerline_overlay)])
        self.run_command(georef_command)

        update_file_entry(metadata, scan_dir, "gps_output_dir", gps_output_dir)
        update_file_entry(metadata, scan_dir, "gps_alignment", gps_output_dir / "gps_alignment.json")
        update_file_entry(metadata, scan_dir, "tf_gps_georeferenced_csv", gps_output_dir / "tf_gps_georeferenced.csv")
        georef_objects = gps_output_dir / "fused_objects_georeferenced.json"
        georef_powerlines = gps_output_dir / "eng498_powerlines_overlay_georeferenced.json"
        if georef_objects.is_file():
            update_file_entry(metadata, scan_dir, "georeferenced_objects", georef_objects)
        if georef_powerlines.is_file():
            update_file_entry(metadata, scan_dir, "georeferenced_powerline_overlay", georef_powerlines)
        save_scan_metadata(scan_dir, metadata)

    def _run_wire_extraction(self, scan_dir: Path, metadata: dict) -> Path:
        las_input = self._resolve_wire_input_las(scan_dir, metadata)
        output_dir = scan_dir / "processed" / "wires"
        output_dir.mkdir(parents=True, exist_ok=True)

        command = [
            sys.executable,
            str(RUN_WIRE_EXTRACTION_SCRIPT),
            "--las-path",
            str(las_input),
            "--output-dir",
            str(output_dir),
        ]
        self.run_command(command)

        update_file_entry(metadata, scan_dir, "wires_points", output_dir / "wires_points.npz")
        update_file_entry(metadata, scan_dir, "wire_info", output_dir / "wire_info.json")
        update_file_entry(metadata, scan_dir, "ground_points", output_dir / "ground_points.npz")
        update_status(metadata, "wire_extraction", "done")
        save_scan_metadata(scan_dir, metadata)
        self.emit_log(f"[outputs] wire extraction dir: {output_dir}")

        powerline_overlay = self._export_powerline_overlay(scan_dir, metadata)
        tf_gps_path = resolve_scan_path(scan_dir, metadata["files"].get("tf_gps_csv"))
        self._maybe_georeference(scan_dir, metadata, tf_gps_path=tf_gps_path, powerline_overlay=powerline_overlay)
        return output_dir

    def _resolve_calibration_artifacts(self, scan_dir: Path, metadata: dict) -> tuple[Path, Path]:
        fusion_cfg = metadata.get("config", {}).get("fusion", {})
        files = metadata.get("files", {})

        linked_run = resolve_scan_path(scan_dir, files.get("calibration_run_dir")) or resolve_scan_path(
            scan_dir, fusion_cfg.get("calibration_run_dir")
        )
        calibration_run = resolve_calibration_run(linked_run, fallback_to_latest=True)
        if calibration_run is None or not calibration_run.is_dir():
            raise FileNotFoundError(
                "No calibration run was found. Complete calibration mode first, then link the calibration output to this scan."
            )

        calibration_output_dir = scan_dir / "processed" / "calibration"
        calib_json_path, intrinsics_json, extrinsics_json = export_fusion_calibration_artifacts(
            calibration_run, calibration_output_dir
        )
        update_file_entry(metadata, scan_dir, "calibration_run_dir", calibration_run)
        update_file_entry(metadata, scan_dir, "calibration_calib_json", calib_json_path)
        update_file_entry(metadata, scan_dir, "resolved_intrinsics_json", intrinsics_json)
        update_file_entry(metadata, scan_dir, "resolved_extrinsics_json", extrinsics_json)
        save_scan_metadata(scan_dir, metadata)
        return intrinsics_json, extrinsics_json

    def _run_yolo_inference(self, scan_dir: Path, metadata: dict, pose_run: Path) -> tuple[Path, Path, Path]:
        config = metadata.get("config", {})
        inference_cfg = config.get("inference", {})

        yolo_output_dir = scan_dir / "processed" / "fusion" / "yolo_inference"
        yolo_output_dir.mkdir(parents=True, exist_ok=True)

        yolo_command = [
            sys.executable,
            str(RUN_YOLO_SCRIPT),
            "--pose-recovery-run-dir",
            str(pose_run),
            "--runtime",
            str(inference_cfg.get("runtime", "auto")),
            "--local-device",
            str(inference_cfg.get("local_device", "0")),
            "--preferred-colab-gpu",
            str(inference_cfg.get("preferred_colab_gpu", "A100")),
            "--output-dir",
            str(yolo_output_dir),
        ]
        weights_path = resolve_scan_path(scan_dir, inference_cfg.get("weights"))
        if weights_path is not None and weights_path.is_file():
            yolo_command.extend(["--weights", str(weights_path)])

        self.run_command(yolo_command)

        masks_dir = yolo_output_dir / "masks_npz"
        meta_dir = yolo_output_dir / "meta_json"
        pred_images_dir = yolo_output_dir / "pred_images"
        if not masks_dir.is_dir() or not meta_dir.is_dir():
            bundle_dir = yolo_output_dir / "colab_bundle"
            update_file_entry(metadata, scan_dir, "yolo_output_dir", yolo_output_dir)
            save_scan_metadata(scan_dir, metadata)
            raise ColabFallbackRequiredError(
                "Local inference did not produce masks/meta outputs. A Colab bundle was prepared at "
                f"{bundle_dir}. Complete Colab inference, import the results, then rerun Fusion."
            )

        update_file_entry(metadata, scan_dir, "yolo_output_dir", yolo_output_dir)
        update_file_entry(metadata, scan_dir, "masks_dir", masks_dir)
        update_file_entry(metadata, scan_dir, "meta_dir", meta_dir)
        if pred_images_dir.is_dir():
            update_file_entry(metadata, scan_dir, "pred_images_dir", pred_images_dir)
        update_status(metadata, "inference", "done")
        save_scan_metadata(scan_dir, metadata)
        self.emit_log(f"[outputs] inference dir: {yolo_output_dir}")
        return yolo_output_dir, masks_dir, meta_dir

    def _run_yolo_and_fusion(self, scan_dir: Path, metadata: dict, pose_run: Path) -> None:
        config = metadata.get("config", {})
        fusion_cfg = config.get("fusion", {})
        effective_timing = resolve_effective_timing(metadata)
        fusion_offset_enabled = bool(effective_timing.get("fusion_time_offset_enabled", False))
        fusion_offset_sec = (
            float(effective_timing.get("fusion_time_offset_sec", 0.0))
            if fusion_offset_enabled
            else 0.0
        )
        self.emit_log(
            "[timing] fusion offset "
            f"{'enabled' if fusion_offset_enabled else 'disabled'} "
            f"(effective={fusion_offset_sec:.6f} sec)"
        )
        if bool(effective_timing.get("preprocess_camera_offset_enabled", False)) and fusion_offset_enabled:
            self.emit_log("[timing] both preprocessing and fusion offsets are enabled for this run.")

        masks_dir = resolve_scan_path(scan_dir, metadata.get("files", {}).get("masks_dir"))
        meta_dir = resolve_scan_path(scan_dir, metadata.get("files", {}).get("meta_dir"))
        if masks_dir is None or not masks_dir.is_dir() or meta_dir is None or not meta_dir.is_dir():
            _, masks_dir, meta_dir = self._run_yolo_inference(scan_dir, metadata, pose_run)
        else:
            update_status(metadata, "inference", "done")
            save_scan_metadata(scan_dir, metadata)

        intrinsics_path, extrinsics_path = self._resolve_calibration_artifacts(scan_dir, metadata)
        _, fusion_point_cloud = first_existing_artifact(
            scan_dir,
            metadata,
            ("filtered_pcd", "raw_pcd", "pcd"),
        )
        if fusion_point_cloud is None:
            raise FileNotFoundError(
                "Fusion requires a point cloud. Expected one of: filtered PCD, raw PCD, or a legacy pcd entry."
            )

        fusion_output_dir = scan_dir / "processed" / "fusion"
        fusion_output_dir.mkdir(parents=True, exist_ok=True)

        fusion_command = [
            sys.executable,
            str(RUN_FUSION_SCRIPT),
            "--intrinsics-json",
            str(intrinsics_path),
            "--extrinsics-json",
            str(extrinsics_path),
            "--pose-csv",
            str(pose_run / "tf_camera_out.csv"),
            "--image-timestamps-csv",
            str(pose_run / "image_timestamps.csv"),
            "--point-cloud",
            str(fusion_point_cloud),
            "--mask-dir",
            str(masks_dir),
            "--meta-dir",
            str(meta_dir),
            "--output-dir",
            str(fusion_output_dir),
            "--time-column",
            "t_query_sec",
            "--image-filename-column",
            "filename",
            "--time-offset-sec",
            str(fusion_offset_sec),
            "--no-visualize",
        ]
        if fusion_offset_enabled:
            fusion_command.append("--time-offset-enabled")
        self.run_command(fusion_command)

        update_file_entry(metadata, scan_dir, "fused_objects", fusion_output_dir / "fused_objects.json")
        update_file_entry(metadata, scan_dir, "fused_map", fusion_output_dir / "fused_semantic_map.ply")
        update_file_entry(metadata, scan_dir, "fused_labels", fusion_output_dir / "fused_semantic_labels.npz")
        update_file_entry(metadata, scan_dir, "pole_neighbor_distances", fusion_output_dir / "pole_neighbor_distances.json")

        powerline_overlay = self._export_powerline_overlay(scan_dir, metadata)

        tf_gps_path = pose_run / "tf_gps_out.csv"
        objects_json = fusion_output_dir / "fused_objects.json"
        self._maybe_georeference(
            scan_dir,
            metadata,
            tf_gps_path=tf_gps_path,
            objects_json=objects_json,
            powerline_overlay=powerline_overlay,
        )

        update_status(metadata, "fusion", "done")
        save_scan_metadata(scan_dir, metadata)
        self.emit_log(f"[outputs] fusion dir: {fusion_output_dir}")

    def run(self) -> None:
        active_stage = "fusion"
        try:
            scan_dir, metadata = load_scan_metadata(self.scan_ref)
            save_scan_metadata(scan_dir, metadata)
            pose_run: Path | None = None

            if self.mode in {"full", "pose-recovery"}:
                active_stage = "slam"
                self.stageChanged.emit("slam", "running")
                pose_run = self._run_pose_recovery(scan_dir, metadata)
                self.stageChanged.emit("slam", "done")
            elif self.mode in {"fusion", "inference"}:
                pose_run = resolve_scan_path(scan_dir, metadata["files"].get("latest_pose_recovery_run"))
                if pose_run is None or not pose_run.is_dir():
                    pose_root = resolve_scan_path(scan_dir, metadata["files"].get("pose_recovery_root"))
                    pose_run = newest_directory(pose_root) if pose_root is not None else None
                if pose_run is None or not pose_run.is_dir():
                    raise FileNotFoundError("No rosbag preprocessing output was found for this scan.")

            if self.mode in {"full", "wire-extraction"}:
                active_stage = "wire_extraction"
                self.stageChanged.emit("wire_extraction", "running")
                self._run_wire_extraction(scan_dir, metadata)
                self.stageChanged.emit("wire_extraction", "done")

            if self.mode in {"full", "inference"}:
                active_stage = "inference"
                self.stageChanged.emit("inference", "running")
                if pose_run is None:
                    raise FileNotFoundError("Image inference requires a rosbag preprocessing output run for this scan.")
                self._run_yolo_inference(scan_dir, metadata, pose_run)
                self.stageChanged.emit("inference", "done")

            if self.mode in {"full", "fusion"}:
                active_stage = "fusion"
                self.stageChanged.emit("fusion", "running")
                if pose_run is None:
                    raise FileNotFoundError("Fusion requires a rosbag preprocessing output run for this scan.")
                self._run_yolo_and_fusion(scan_dir, metadata, pose_run)
                self.stageChanged.emit("fusion", "done")

            self.completed.emit(str(scan_dir), self.mode)
        except ColabFallbackRequiredError as exc:
            self.stageChanged.emit(active_stage, "pending")
            self.failed.emit(str(exc))
        except Exception as exc:  # pragma: no cover - GUI-facing error path
            self.failed.emit(str(exc))
