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
- `testDashboard.py`
  Integrated dashboard harness that connects the current GUI to rosbag
  preprocessing, YOLO inference, fusion, georeferencing, and the combined
  semantic viewer.
- `PreProcessing_GUI/`
  Windows-side LAS / point cloud preprocessing utilities.
- `Matlab_ExtractPowerLine/`
  MATLAB-connected powerline viewer and test scripts.

For backend documentation, use:

- click-through docs set:
  - [backend_docs/README.md](backend_docs/README.md)
- single-file reference:
  - [BACKEND_SOFTWARE_DESCRIPTION.md](BACKEND_SOFTWARE_DESCRIPTION.md)


## Integrated GUI Flow

The current `main.py` entrypoint launches the integrated dashboard window.

From the GUI, the current experimental integration supports:

- scan creation by importing a rosbag into `assets/<scan_name>/raw/`
- Docker pose recovery from the dashboard
- MATLAB-driven wire extraction from the dashboard
- local YOLO inference or Colab-bundle fallback preparation
- Fusion and GPS georeferencing
- opening the semantic viewer with:
  - semantic LAS / point cloud background
  - wire extraction overlays
  - Fusion object overlays
  - pole-to-pole distance overlays
- opening the Leaflet map from the GUI when Fusion objects or powerline
  overlays exist

The semantic viewer is driven by:

- `Matlab_ExtractPowerLine/testViewer.py`
- `Matlab_ExtractPowerLine/testSemanticLidarViewer.py`

and the dashboard orchestration layer is currently in:

- `testDashboard.py`
- `gui_pipeline.py`
- `scan_metadata.py`

The GUI currently supports two control styles:

- Auto mode
  - runs the main end-to-end backend chain for a selected scan
- Step-by-step mode
  - runs individual backend stages for:
    - SLAM / pose recovery
    - wire extraction
    - fusion
  - opens the filtering tool for the filtering step
  - treats FLAI as an external/manual stage for now


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

When launched from the integrated dashboard, the pose-recovery results are
written into the selected scan folder under:

```text
assets/<scan_name>/processed/pose_recovery/
```

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

When launched from the integrated GUI, the backend writes into the selected
scan folder under:

```text
assets/<scan_name>/processed/
  pose_recovery/
  wires/
  fusion/
```


## Launch The Integrated GUI

From `ENGR-498-Project/`:

```powershell
python .\main.py
```

Recommended usage:

1. Create a scan from the dashboard.
2. Select the scan.
3. Use either:
   - Auto mode to run the main backend chain end to end, or
   - Step-by-step mode to run SLAM, wire extraction, or fusion individually.
4. Open the semantic viewer from the scan row.
5. Open the map from the scan row when Fusion objects or powerline overlays are available.

Current automation boundary:

- Automated from the GUI:
  - rosbag pose recovery
  - JPG export
  - TF CSV export
  - wire extraction
  - local YOLO inference / Colab bundle preparation
  - fusion
  - GPS georeferencing
  - Leaflet launch
- Still manual or external:
  - FLAI
  - Colab inference execution after a fallback bundle is prepared


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
