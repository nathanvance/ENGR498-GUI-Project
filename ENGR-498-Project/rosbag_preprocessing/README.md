# Rosbag Preprocessing

This folder contains the Docker/Compose project used to run the Linux ROS
preprocessing workflows from Windows.

Normal user expectation:

- clone the repo
- start Docker Desktop
- run `bash scripts/build_image_wsl.sh`
- after pulling updates, run `bash scripts/update_image_wsl.sh`
- use the GUI or the launcher scripts

You do **not** need local `ws_calib`, `ws_livox`, or `iridescence` folders on
the host machine just to build the image from this repo.

This stack provides:

- direct visual LiDAR calibration
- FAST-LIO rosbag preprocessing / SLAM
- TF sampling on image and GPS events

Calibration mode and post-processing mode are intentionally separate:

- calibration mode extracts camera intrinsics from `/camera/camera_info`,
  solves the LiDAR-camera extrinsic transform, and writes a calibration run
- post-processing mode runs FAST-LIO SLAM, JPG export, and TF sampling, then
  feeds those outputs into wire extraction, image inference, Fusion, and GPS
  georeferencing

These workflows can still be launched manually, and the Windows dashboard also
drives the pose-recovery workflow through:

- `ENGR-498-Project/testDashboard.py`
- `ENGR-498-Project/gui_pipeline.py`

That GUI path then hands the pose-recovery outputs forward into:

- MATLAB wire extraction
- YOLO inference
- fusion
- GPS georeferencing
- semantic viewer / map launch

The repo copy is designed to be portable at the source/configuration level:

- repo paths are relative
- Windows launchers derive the project root from their own location
- WSL runtime staging is driven by environment variables instead of one
  developer username
- outputs default to mounted repo folders under `outputs/`

The image is built from the staged ROS runtime stored in:

```text
rt/
```

That staged runtime includes the ROS workspaces and libraries needed by the
containerized calibration and rosbag preprocessing flows. The runtime root is
kept intentionally short so Windows clones are less likely to hit path-length
limits during checkout.


## Quick Start

### 1. Start Docker Desktop

Make sure:

- Docker Desktop is running
- WSL2 is installed

### 2. Build the image

From a WSL shell opened in this folder:

```bash
bash scripts/build_image_wsl.sh
```

For later updates after `git pull`, run:

```bash
bash scripts/update_image_wsl.sh
```

That wrapper rebuilds the same image and lets Docker reuse cached layers when
available, so repeat updates are usually much faster than the first install.

### 3. Run a workflow

Calibration:

```powershell
python .\launcher\run_calibration_workflow.py <dataset_path> --run-name test1_manual
```

Calibration requirements check:

```powershell
python .\launcher\run_calibration_workflow.py --check-only
```

Pose recovery / TF export:

```powershell
python .\launcher\run_transform_reading_workflow.py <bag_path> --image-topic /camera/image/compressed --gps-topic /fix
```


## Host Requirements

Required:

- Windows
- WSL2
- Docker Desktop
- Python on Windows for the launcher scripts

Recommended:

- Docker Desktop WSL integration enabled

The helper scripts and launchers fall back to `docker.exe` from WSL if the
Linux-side `docker` CLI is not available.

The recommended Windows-side Python environment for the repo is installed from
the project root with:

```powershell
python -m venv .venv
powershell -ExecutionPolicy Bypass -File ..\install_repo_python_env.ps1 `
  -Python ..\.venv\Scripts\python.exe
```

That repo-local `.venv` is the recommended setup because the installer writes
packages into whichever interpreter is passed with `-Python`.

Required for the calibration GUIs:

- WSLg support
- working hardware-backed OpenGL / GPU acceleration through WSLg

Not supported for calibration on Windows:

- software OpenGL rendering
- CPU-only fallback rendering

Not required on the host:

- ROS on Windows
- FAST-LIO on Windows


## Folder Layout

```text
rosbag_preprocessing/
  compose.yaml
  Dockerfile
  docker/
  launcher/
  scripts/
  rt/
    calib/
    livox/
    iri/
    lib/
    usr/
  outputs/
    calibration/
    pose_recovery/
  dist/
```

Important folders:

- `launcher/`
  Windows-side Python launchers.
- `scripts/`
  WSL helper scripts for syncing the runtime, building the image, and exporting
  the image archive.
- `rt/`
  Staged runtime artifacts copied from the working WSL environment before a
  Docker build.
- `outputs/`
  Persisted outputs written by the containerized workflows.


## Why The Staged Runtime Still Matters

This project still depends on staged runtime artifacts copied from a known-good
ROS/WSL setup. That staged runtime is now checked into the repo under `rt/`.
It includes the catkin `devel` spaces from:

- `ws_calib`
- `ws_livox`

Those `devel` spaces are not relocatable in a clean source-only sense, so they
are intentionally preserved as part of the runtime packaging flow. The repo
uses neutral, documented paths instead of hardcoding one developer's username
or old folder names to find them.

Normal users do not need any of those upstream workspaces locally.

Maintainers only:

- if you want to refresh the embedded runtime from a newer development machine,
  you can still do that
- the default maintainer-side staging root is now:

```text
~/Senior_Design_Docker_Image/
```

Expected maintainer refresh layout by default:

```text
~/Senior_Design_Docker_Image/
  ws_calib/
  ws_livox/
  iridescence/
  lib/
    libglfw_hint_shim.so
```

You can still override all of those locations with environment variables.


## Runtime Staging Inputs

This section only matters to maintainers who want to refresh the committed
runtime. Normal users can skip it.

When explicitly invoked, `scripts/sync_wsl_context.sh` copies the required
runtime files from a known-good WSL environment into `rt/`.

By default it looks under:

- `~/Senior_Design_Docker_Image/ws_calib`
- `~/Senior_Design_Docker_Image/ws_livox`
- `~/Senior_Design_Docker_Image/iridescence`
- `~/Senior_Design_Docker_Image/lib/libglfw_hint_shim.so`

You can override that layout with environment variables:

- `WSL_HOME_ROOT`
- `WS_CALIB_ROOT`
- `WS_LIVOX_ROOT`
- `IRIDESCENCE_ROOT`
- `GLFW_SHIM_PATH`
- `USR_LOCAL_PREFIX`
- `PORTABLE_ROS_RUNTIME_USER`

Example from a WSL shell opened in this folder:

```bash
bash scripts/sync_wsl_context.sh
```


## Build The Image

From a WSL shell opened in this folder:

```bash
bash scripts/build_image_wsl.sh
```

That script:

1. verifies the committed runtime exists in `rt/`
2. builds the Docker image with Compose

Important:

- the build helper creates repo-side folders like `outputs/` and `dist/`
- it does **not** require upstream WSL ROS folders on a normal user machine
- it builds directly from the runtime that ships in this repo

## Update The Image

After pulling repo updates, rebuild from a WSL shell opened in this folder:

```bash
git pull
bash scripts/update_image_wsl.sh
```

This uses the same Docker build flow as the first install. When the machine
still has its prior Docker layers cached, the update is usually much faster
than a fresh build.

Maintainer-only refresh flow:

```bash
bash scripts/build_image_wsl.sh --refresh-runtime
```

That path first reruns `scripts/sync_wsl_context.sh`, so it still requires the
maintainer-side WSL runtime folders described above.

Image tag:

```text
senior-design/portable-ros-stack:noetic
```


## Save Or Load The Image

Save from a WSL shell opened in this folder:

```bash
bash scripts/save_image_wsl.sh
```

Default archive output:

```text
dist/portable-ros-stack-noetic.tar
```

Load an existing archive from a WSL shell opened in this folder:

```bash
docker load -i ./dist/portable-ros-stack-noetic.tar
```

For another machine, either of these paths is supported:

1. run `bash scripts/build_image_wsl.sh` directly from the repo copy
2. load a previously exported archive with `docker load -i ...`

You no longer need local `ws_calib` / `ws_livox` folders just to build the
image from this repo.


## Outputs

By default the container writes outputs into the mounted repo folder:

```text
outputs/calibration/
outputs/pose_recovery/
```

Typical pose-recovery output:

```text
outputs/pose_recovery/<bag_stem>_<timestamp_pid>/
  image_timestamps.csv
  images/
    frame_000001.jpg
    frame_000002.jpg
  tf_camera_out.csv
  tf_gps_out.csv
  pcd/
    scans.pcd
  logs/
```

Typical calibration output:

```text
outputs/calibration/<run_name>/
outputs/calibration/<run_name>_raw_input/
```


## Windows Launchers

Main launcher:

- `launcher/run_pipeline.py`

Convenience wrappers:

- `launcher/run_calibration_workflow.py`
- `launcher/run_transform_reading_workflow.py`

The GUI uses the same launcher stack rather than duplicating Docker logic.
That keeps the dashboard path and the CLI path on the same backend contract.


## Run The Calibration Workflow

Before first use on a machine, run the requirements check:

```powershell
python .\launcher\run_calibration_workflow.py --check-only
```

That probe verifies:

- WSL is reachable
- Docker can launch the calibration container
- WSLg display plumbing is present
- the calibration container reports a hardware-backed OpenGL renderer

If the requirements check reports a software renderer such as `llvmpipe`, do
not proceed with calibration on that machine.

From this folder:

```powershell
python .\launcher\run_calibration_workflow.py <dataset_path> --run-name test1_manual
```

Optional preprocess-only checkpoint:

```powershell
python .\launcher\run_calibration_workflow.py <dataset_path> --run-name test1_manual --stop-after preprocess
```


## Run The Pose-Recovery Workflow

From this folder:

```powershell
python .\launcher\run_transform_reading_workflow.py <bag_path> --image-topic /camera/image/compressed --gps-topic /fix
```

This workflow writes the key downstream handoff file:

```text
outputs/pose_recovery/<run_name>/tf_gps_out.csv
```

When the integrated GUI launches rosbag preprocessing for a scan, the output root is
redirected into the scan folder itself:

```text
ENGR-498-Project/assets/<scan_name>/processed/pose_recovery/
```

That per-scan output root is what the downstream GUI-launched wire extraction
and Fusion stages consume.

That CSV is the direct input expected by the Fusion georeferencing stage.

It also saves every camera message received on the detected or overridden image
topic as a `.jpg` file in:

```text
outputs/pose_recovery/<run_name>/images/
```

and writes a Fusion-friendly image timestamp file:

```text
outputs/pose_recovery/<run_name>/image_timestamps.csv
```

Those two outputs are the direct inputs expected by:

```text
ENGR-498-Project/fusion/run_yolo_inference.py
```


## Compose Notes

The Compose service mounts:

- this folder to `/workspace`
- the Windows drive root visible in WSL
- WSLg runtime/display locations

Those mounts are environment-driven in `compose.yaml` so the repo does not have
to hardcode one developer's machine paths.

The container still uses standard Linux absolute paths internally for the
staged runtime and WSLg libraries. That is normal for Docker and distinct from
the user-specific hardcoded paths that were removed from the repo.


## Troubleshooting

If the launcher exits immediately:

- make sure Docker Desktop is running
- make sure WSL2 is installed
- confirm either:
  - `wsl bash -lc "docker --version"` works, or
  - `wsl bash -lc "docker.exe --version"` works

If the calibration GUIs are slow or unusable:

- confirm WSLg is active
- confirm OpenGL acceleration is working in WSL
- confirm the container has the WSLg and WSL library mounts available

If outputs are missing:

- check `outputs/calibration`
- check `outputs/pose_recovery`
- verify you did not override `--run-root` or `--output-root`
