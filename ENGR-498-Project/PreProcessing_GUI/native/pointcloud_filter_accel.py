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
    return _native_dir() / "build" / "pointcloud_filter_accel.dll"


def ensure_built(force: bool = False) -> Path:
    dll_path = _dll_path()
    source_path = _native_dir() / "pointcloud_filter_accel.cpp"
    if not force and dll_path.exists() and dll_path.stat().st_mtime >= source_path.stat().st_mtime:
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

    voxel = dll.voxel_downsample_indices
    voxel.argtypes = [
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int64,
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_int64),
        ctypes.c_int64,
    ]
    voxel.restype = ctypes.c_int64

    crop = dll.crop_mask
    crop.argtypes = [
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int64,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_uint8),
    ]
    crop.restype = None

    _DLL = dll
    return dll


def _ptr(array: np.ndarray, c_type):
    return array.ctypes.data_as(ctypes.POINTER(c_type))


def voxel_downsample_indices(points: np.ndarray, voxel_size: float) -> np.ndarray:
    dll = _load_library()
    func = dll.voxel_downsample_indices
    points = np.ascontiguousarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    out_indices = np.empty(points.shape[0], dtype=np.int64)
    count = func(_ptr(points, ctypes.c_double), int(points.shape[0]), float(voxel_size), _ptr(out_indices, ctypes.c_int64), int(out_indices.size))
    if count < 0:
        raise RuntimeError("voxel_downsample_indices failed")
    return out_indices[: int(count)]


def crop_mask(points: np.ndarray, xmin: float, xmax: float, ymin: float, ymax: float, zmin: float, zmax: float) -> np.ndarray:
    dll = _load_library()
    func = dll.crop_mask
    points = np.ascontiguousarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    mask = np.empty(points.shape[0], dtype=np.uint8)
    func(_ptr(points, ctypes.c_double), int(points.shape[0]), float(xmin), float(xmax), float(ymin), float(ymax), float(zmin), float(zmax), _ptr(mask, ctypes.c_uint8))
    return mask.astype(bool, copy=False)

