from __future__ import annotations

import argparse
import os
import shutil
import subprocess
from pathlib import Path


def find_vs_installation() -> Path:
    if os.environ.get("VSWHERE_PATH"):
        vswhere = Path(os.environ["VSWHERE_PATH"])
    else:
        path_hit = shutil.which("vswhere.exe")
        if path_hit:
            vswhere = Path(path_hit)
        else:
            program_files_x86 = os.environ.get("ProgramFiles(x86)")
            if not program_files_x86:
                raise FileNotFoundError("Set VSWHERE_PATH or install Visual Studio Build Tools with vswhere.exe.")
            vswhere = Path(program_files_x86) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
    if not vswhere.exists():
        raise FileNotFoundError(f"vswhere.exe not found at {vswhere}")

    result = subprocess.run(
        [
            str(vswhere),
            "-latest",
            "-products",
            "*",
            "-requires",
            "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
            "-property",
            "installationPath",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    installation = result.stdout.strip()
    if not installation:
        raise RuntimeError("Visual Studio Build Tools were not found.")
    return Path(installation)


def build(force: bool = False) -> Path:
    native_dir = Path(__file__).resolve().parent
    source = native_dir / "pointcloud_filter_accel.cpp"
    build_dir = native_dir / "build"
    build_dir.mkdir(parents=True, exist_ok=True)
    dll_path = build_dir / "pointcloud_filter_accel.dll"
    obj_path = build_dir / "pointcloud_filter_accel.obj"

    if not force and dll_path.exists() and dll_path.stat().st_mtime >= source.stat().st_mtime:
        return dll_path

    installation = find_vs_installation()
    vsdevcmd = installation / "Common7" / "Tools" / "VsDevCmd.bat"
    if not vsdevcmd.exists():
        raise FileNotFoundError(f"VsDevCmd.bat not found at {vsdevcmd}")

    batch_path = build_dir / "build_native.bat"
    batch_path.write_text(
        "@echo off\n"
        f'call "{vsdevcmd}" -arch=amd64\n'
        "if errorlevel 1 exit /b %errorlevel%\n"
        f'cl /nologo /LD /O2 /EHsc /std:c++17 /Fo"{obj_path}" "{source}" /link /OUT:"{dll_path}"\n',
        encoding="utf-8",
    )

    result = subprocess.run(["cmd.exe", "/d", "/c", str(batch_path)], cwd=native_dir, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            "Failed to build pointcloud_filter_accel.dll\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    return dll_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Build the point-cloud filter accelerator DLL.")
    parser.add_argument("--force", action="store_true", help="Rebuild even if the DLL is up to date.")
    args = parser.parse_args()

    print(build(force=args.force))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

