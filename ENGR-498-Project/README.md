# ENGR-498 Project

## Overview

This folder contains the portable project code used across the current Senior
Design workflow:

- `fusion/`
  Camera-to-LiDAR semantic fusion, GPS georeferencing, Leaflet output, and
  local/Colab YOLO inference helpers.
- `rosbag_preprocessing/`
  Docker + WSL preprocessing for:
  - direct visual LiDAR calibration
  - FAST-LIO map generation
  - TF sampling on image and GPS events
- `views/`, `widgets/`, `main.py`
  Windows-side GUI and dashboard code.
- `PreProcessing_GUI/`
  Windows-side LAS / point cloud preprocessing utilities.
- `Matlab_ExtractPowerLine/`
  MATLAB-connected powerline viewer and test scripts.

For a detailed backend architecture walkthrough, open:

- [BACKEND_SOFTWARE_DESCRIPTION.md](BACKEND_SOFTWARE_DESCRIPTION.md)


## Host Requirements

### Windows-side requirements

Required:

- Windows
- Python 3.11
- a Python virtual environment for this repo

Recommended:

- NVIDIA GPU with current CUDA-capable drivers for local YOLO inference
- Visual Studio Build Tools for native Fusion acceleration builds

Optional:

- MATLAB with Python engine support if you want to run the MATLAB-connected
  Python scripts

### Docker / ROS requirements

Required for `rosbag_preprocessing/`:

- WSL2
- Docker Desktop with WSL integration enabled

Required for calibration GUIs inside Docker:

- WSLg
- working OpenGL / GPU acceleration through WSLg


## Windows Python Environment Setup

Create or choose a Python 3.11 virtual environment, then install the repo's
Windows-side Python dependencies:

```powershell
powershell -ExecutionPolicy Bypass -File .\install_repo_python_env.ps1 `
  -Python C:\path\to\python.exe
```

Use the lock file instead of the curated requirements file if you want the
exact currently validated package snapshot:

```powershell
powershell -ExecutionPolicy Bypass -File .\install_repo_python_env.ps1 `
  -Python C:\path\to\python.exe `
  -UseLockFile
```

If you also want the MATLAB Python bridge and your machine already has MATLAB
configured for Python, add:

```powershell
-IncludeMatlabEngine
```

Important:

- the actual virtual environment directory is intentionally not committed to
  the repo
- portability is handled through:
  - `requirements-repo-python.txt`
  - `requirements-repo-python-lock.txt`
  - optional MATLAB files:
    - `requirements-repo-python-matlab.txt`
    - `requirements-repo-python-matlab-lock.txt`


## Typical Setup Flow

### 1. Set up the Windows Python environment

Run the installer above.

### 2. Set up Docker / WSL for rosbag preprocessing

Open:

- [rosbag_preprocessing/README.md](rosbag_preprocessing/README.md)

That covers:

- runtime staging from WSL
- Docker image build
- calibration workflow
- pose recovery workflow
- output locations

### 3. Run rosbag preprocessing

Outputs land under:

```text
rosbag_preprocessing/outputs/
```

The pose-recovery workflow produces:

- `images/`
- `image_timestamps.csv`
- `tf_camera_out.csv`
- `tf_gps_out.csv`
- `pcd/scans.pcd`

### 4. Run YOLO inference

Open:

- [fusion/README.txt](fusion/README.txt)

The inference helpers consume the JPG output from `rosbag_preprocessing`
directly.

Supported modes:

- local GPU inference
- auto local/Colab selection
- explicit Colab bundle generation

### 5. Run fusion and georeferencing

Use the scripts in `fusion/` to:

- project masks into the SLAM cloud
- segment instances
- georeference objects and powerlines
- generate Leaflet-ready outputs


## Folder-Specific Documentation

- [fusion/README.txt](fusion/README.txt)
- [rosbag_preprocessing/README.md](rosbag_preprocessing/README.md)


## Notes on Scope

This repo uses more than one runtime domain:

- Windows Python for the GUI, viewers, preprocessing helpers, and Fusion
- Docker / WSL for ROS preprocessing
- optional Colab for cloud YOLO inference fallback

So there is now one general Windows-side Python installer for the repo, while
the ROS/Docker stack remains documented and managed separately under
`rosbag_preprocessing/`.
