from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from calibration_bridge import export_fusion_calibration_artifacts


def test_exported_fusion_extrinsics_are_lidar_to_camera(tmp_path: Path) -> None:
    calibration_run_dir = tmp_path / "calibration_run"
    output_dir = tmp_path / "fusion_ready"
    calibration_run_dir.mkdir()

    Image.new("RGB", (640, 480), color=0).save(calibration_run_dir / "frame.jpg")

    sin_45 = math.sqrt(0.5)
    cos_45 = math.sqrt(0.5)
    calib_payload = {
        "camera": {
            "intrinsics": [1000.0, 1001.0, 320.0, 240.0],
            "distortion_coeffs": [0.1, -0.2, 0.0, 0.0, 0.0],
        },
        "results": {
            # Calibration stores camera->lidar under T_lidar_camera.
            "T_lidar_camera": [1.0, 2.0, 3.0, 0.0, 0.0, sin_45, cos_45],
        },
    }
    (calibration_run_dir / "calib.json").write_text(json.dumps(calib_payload), encoding="utf-8")

    _, intrinsics_json, extrinsics_json = export_fusion_calibration_artifacts(calibration_run_dir, output_dir)

    intrinsics = json.loads(intrinsics_json.read_text(encoding="utf-8"))
    extrinsics = np.asarray(json.loads(extrinsics_json.read_text(encoding="utf-8"))["T_lidar_cam"], dtype=np.float64)

    expected_camera_to_lidar = np.array(
        [
            [0.0, -1.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    expected_lidar_to_camera = np.array(
        [
            [0.0, 1.0, 0.0, -2.0],
            [-1.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0, -3.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    assert intrinsics["width"] == 640
    assert intrinsics["height"] == 480
    np.testing.assert_allclose(extrinsics, expected_lidar_to_camera, atol=1e-9)
    np.testing.assert_allclose(extrinsics @ expected_camera_to_lidar, np.eye(4), atol=1e-9)
