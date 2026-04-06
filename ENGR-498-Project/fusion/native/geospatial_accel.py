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
    return _native_dir() / "build" / "geospatial_accel.dll"


def ensure_built(force: bool = False) -> Path:
    dll_path = _dll_path()
    source_path = _native_dir() / "geospatial_accel.cpp"
    if (
        not force
        and dll_path.exists()
        and dll_path.stat().st_mtime >= source_path.stat().st_mtime
    ):
        return dll_path

    build_script = _native_dir() / "build_geospatial_accel.py"
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
    func = dll.apply_similarity_xyz
    func.argtypes = [
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int64,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_double),
    ]
    func.restype = ctypes.c_int
    _DLL = dll
    return dll


def _ptr(array: np.ndarray, c_type):
    return array.ctypes.data_as(ctypes.POINTER(c_type))


def apply_similarity_xyz(
    points_xyz: np.ndarray,
    *,
    scale: float,
    cos_yaw: float,
    sin_yaw: float,
    tx: float,
    ty: float,
    tz: float,
) -> np.ndarray:
    dll = _load_library()
    func = dll.apply_similarity_xyz

    points_xyz = np.ascontiguousarray(points_xyz, dtype=np.float64)
    if points_xyz.ndim != 2 or points_xyz.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")

    out = np.empty_like(points_xyz)
    rc = func(
        _ptr(points_xyz, ctypes.c_double),
        int(points_xyz.shape[0]),
        float(scale),
        float(cos_yaw),
        float(sin_yaw),
        float(tx),
        float(ty),
        float(tz),
        _ptr(out, ctypes.c_double),
    )
    if rc != 0:
        raise RuntimeError(f"apply_similarity_xyz failed with code {rc}")
    return out
