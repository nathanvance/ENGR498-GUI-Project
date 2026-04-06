from __future__ import annotations

import ctypes
import subprocess
import sys
from pathlib import Path

import numpy as np


_DLL = None


def _native_dir() -> Path:
    return Path(__file__).resolve().parent


def _dll_path() -> Path:
    return _native_dir() / "build" / "pointcloud_accel.dll"


def ensure_built(force: bool = False) -> Path:
    dll_path = _dll_path()
    source_path = _native_dir() / "pointcloud_accel.cpp"
    if (
        not force
        and dll_path.exists()
        and dll_path.stat().st_mtime >= source_path.stat().st_mtime
    ):
        return dll_path

    build_script = _native_dir() / "build_accel.py"
    command = [sys.executable, str(build_script)]
    if force:
        command.append("--force")
    subprocess.run(command, check=True, cwd=_native_dir())
    if not dll_path.exists():
        raise FileNotFoundError(f"Expected accelerator DLL at {dll_path}")
    return dll_path


def _load_library() -> ctypes.CDLL:
    global _DLL
    if _DLL is not None:
        return _DLL

    dll = ctypes.WinDLL(str(ensure_built()))
    func = dll.project_assign_best_detection
    func.argtypes = [
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int64,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_int32,
        ctypes.c_int32,
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int32,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_int32,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_double),
    ]
    func.restype = ctypes.c_int
    _DLL = dll
    return dll


def _ptr(array: np.ndarray, c_type):
    return array.ctypes.data_as(ctypes.POINTER(c_type))


def project_assign_best_detection(
    points_map: np.ndarray,
    map_to_lidar: np.ndarray,
    lidar_to_cam: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    width: int,
    height: int,
    distortion: np.ndarray,
    masks: np.ndarray,
    confidences: np.ndarray,
    allowed: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    dll = _load_library()
    func = dll.project_assign_best_detection

    points_map = np.ascontiguousarray(points_map, dtype=np.float64)
    map_to_lidar = np.ascontiguousarray(map_to_lidar.reshape(-1), dtype=np.float64)
    lidar_to_cam = np.ascontiguousarray(lidar_to_cam.reshape(-1), dtype=np.float64)
    distortion = np.ascontiguousarray(distortion.reshape(-1), dtype=np.float64)
    masks = np.ascontiguousarray(masks, dtype=np.uint8)
    confidences = np.ascontiguousarray(confidences.reshape(-1), dtype=np.float64)
    allowed = np.ascontiguousarray(allowed.reshape(-1), dtype=np.uint8)

    if points_map.ndim != 2 or points_map.shape[1] != 3:
        raise ValueError("points_map must have shape (N, 3)")
    if masks.ndim != 3:
        raise ValueError("masks must have shape (num_instances, H, W)")

    num_points = points_map.shape[0]
    num_instances = masks.shape[0]

    out_detection_index = np.full(num_points, -1, dtype=np.int32)
    out_detection_confidence = np.zeros(num_points, dtype=np.float64)

    distortion_ptr = (
        _ptr(distortion, ctypes.c_double)
        if distortion.size > 0
        else ctypes.POINTER(ctypes.c_double)()
    )

    rc = func(
        _ptr(points_map, ctypes.c_double),
        num_points,
        _ptr(map_to_lidar, ctypes.c_double),
        _ptr(lidar_to_cam, ctypes.c_double),
        float(fx),
        float(fy),
        float(cx),
        float(cy),
        int(width),
        int(height),
        distortion_ptr,
        int(distortion.size),
        _ptr(masks, ctypes.c_uint8),
        int(num_instances),
        _ptr(confidences, ctypes.c_double),
        _ptr(allowed, ctypes.c_uint8),
        _ptr(out_detection_index, ctypes.c_int32),
        _ptr(out_detection_confidence, ctypes.c_double),
    )
    if rc != 0:
        raise RuntimeError(f"project_assign_best_detection failed with code {rc}")

    return out_detection_index, out_detection_confidence
