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
    return _native_dir() / "build" / "wire_accel.dll"


def ensure_built(force: bool = False) -> Path:
    dll_path = _dll_path()
    source_path = _native_dir() / "wire_accel.cpp"
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
    func = dll.compute_local_principal_features
    func.argtypes = [
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int64,
        ctypes.POINTER(ctypes.c_int64),
        ctypes.POINTER(ctypes.c_int64),
        ctypes.c_int64,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_uint8),
    ]
    func.restype = ctypes.c_int
    _DLL = dll
    return dll


def _ptr(array: np.ndarray, c_type):
    return array.ctypes.data_as(ctypes.POINTER(c_type))


def compute_local_principal_features(
    points: np.ndarray,
    neighbor_indices: np.ndarray,
    neighbor_offsets: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    dll = _load_library()
    func = dll.compute_local_principal_features

    points = np.ascontiguousarray(points, dtype=np.float64)
    neighbor_indices = np.ascontiguousarray(neighbor_indices, dtype=np.int64)
    neighbor_offsets = np.ascontiguousarray(neighbor_offsets, dtype=np.int64)

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if neighbor_offsets.ndim != 1:
        raise ValueError("neighbor_offsets must be a 1D array")
    if neighbor_offsets.size == 0:
        raise ValueError("neighbor_offsets must contain at least one offset")

    n_queries = neighbor_offsets.size - 1
    out_dirs = np.zeros((n_queries, 3), dtype=np.float64)
    out_linearness = np.zeros(n_queries, dtype=np.float64)
    out_valid = np.zeros(n_queries, dtype=np.uint8)

    rc = func(
        _ptr(points, ctypes.c_double),
        int(points.shape[0]),
        _ptr(neighbor_indices, ctypes.c_int64),
        _ptr(neighbor_offsets, ctypes.c_int64),
        int(n_queries),
        _ptr(out_dirs, ctypes.c_double),
        _ptr(out_linearness, ctypes.c_double),
        _ptr(out_valid, ctypes.c_uint8),
    )
    if rc != 0:
        raise RuntimeError(f"compute_local_principal_features failed with code {rc}")

    return out_dirs, out_linearness, out_valid.astype(bool, copy=False)
