# Rosbag Preprocessing

This folder contains the Docker/Compose project used to run the Linux ROS
preprocessing workflows from Windows:

- direct visual LiDAR calibration
- FAST-LIO pose recovery
- TF sampling on image and GPS events

These workflows can still be launched manually, but the experimental GUI branch
also drives the pose-recovery workflow from the Windows dashboard through:

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

The runtime packaging approach is unchanged: the image is still built from a
staged WSL runtime that includes the already-working catkin `devel` spaces.


## Host Requirements

Required:

- Windows
- WSL2
- Docker Desktop with WSL integration enabled
- Python on Windows for the launcher scripts

The recommended Windows-side Python environment for the repo is installed from
the project root with:

```powershell
powershell -ExecutionPolicy Bypass -File ..\install_repo_python_env.ps1 `
  -Python C:\path\to\python.exe
```

Required for the calibration GUIs:

- WSLg support
- working OpenGL / GPU acceleration through WSLg

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
  context/
    runtime/
      home/
      usr_local/
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
- `context/runtime/`
  Staged runtime artifacts copied from the working WSL environment before a
  Docker build.
- `outputs/`
  Persisted outputs written by the containerized workflows.


## Why The Staged Runtime Still Matters

This project still depends on staged runtime artifacts copied from a known-good
WSL setup. That staging process includes the catkin `devel` spaces from:

- `ws_calib`
- `ws_livox`

Those `devel` spaces are not relocatable in a clean source-only sense, so they
are intentionally preserved as part of the runtime packaging flow. The portable
change in this branch is that the repo no longer hardcodes one developer's
username or old folder names to find them.

Do not delete the source WSL `devel` folders if you still plan to rebuild the
Docker image from the working WSL environment.


## Runtime Staging Inputs

Before building the image, `scripts/sync_wsl_context.sh` copies the required
runtime files into `context/runtime/`.

By default it assumes the common WSL layout:

- `~/ws_calib`
- `~/ws_livox`
- `~/iridescence`
- `~/lib/libglfw_hint_shim.so`

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

1. syncs the staged runtime into `context/runtime/`
2. builds the Docker image with Compose

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

When the integrated GUI launches pose recovery for a scan, the output root is
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
- make sure WSL integration is enabled
- confirm `wsl bash -lc "docker --version"` works

If the calibration GUIs are slow or unusable:

- confirm WSLg is active
- confirm OpenGL acceleration is working in WSL
- confirm the container has the WSLg and WSL library mounts available

If outputs are missing:

- check `outputs/calibration`
- check `outputs/pose_recovery`
- verify you did not override `--run-root` or `--output-root`
