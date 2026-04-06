# Backend Software Description

## Quick Overview

At a high level, the backend starts with one or more rosbags and turns them
into a georeferenced, semantically labeled utility-scene dataset. The
`rosbag_preprocessing` stack replays the bag, runs FAST-LIO to build the SLAM
map, exports JPG camera frames, and samples the LiDAR pose whenever image and
GPS messages arrive so later stages have exact time-aligned transforms. Those
JPGs then go through YOLO segmentation either on a local NVIDIA GPU or through
the Colab fallback, producing per-image instance masks and metadata. The
`fusion` stack uses camera intrinsics, LiDAR-camera extrinsics, time-matched
LiDAR poses, and the segmentation masks to project semantic labels into the
global point cloud, filter them, segment object instances such as poles,
crossarms, transformers, and pedestals, and compute derived metrics like pole
spacing. If GPS is available, the pipeline then fits a local-to-global
alignment from `tf_gps_out.csv`, writes GPS coordinates back into fused objects
and powerline overlays, and leaves behind a set of portable artifacts such as
`scans.pcd`, `fused_objects.json`, `fused_semantic_map.ply`,
`pole_neighbor_distances.json`, and georeferenced JSON outputs for mapping and
downstream visualization.

## Scope

This document explains the backend data-processing pipeline implemented in this
repository. It intentionally excludes the GUI/dashboard visualization layer and
focuses on how the software:

1. preprocesses ROS bags inside Docker and WSL-backed ROS workspaces,
2. recovers LiDAR poses and exports image/GPS-aligned artifacts,
3. runs YOLO segmentation inference locally or through a Colab fallback,
4. projects semantic masks into a SLAM point cloud,
5. segments object instances and computes pole spacing,
6. georeferences fused objects and powerline overlays,
7. prepares outputs for downstream viewers and mapping.

The primary code paths covered here live in:

- `rosbag_preprocessing/`
- `fusion/`
- `project_paths.py`
- `install_repo_python_env.ps1`

This document describes the backend as it exists on the
`fusion-portability-integration` branch.

## System Overview

At a high level, the backend is split into four runtime domains:

1. Windows Python orchestration
   - Launches Docker workflows.
   - Runs Fusion, GPS georeferencing, and local YOLO inference.
   - Builds optional native Windows DLL accelerators.

2. Dockerized ROS preprocessing
   - Wraps FAST-LIO and direct visual LiDAR calibration workflows.
   - Writes portable run outputs into mounted repo folders.
   - Samples `/tf` when image and GPS messages arrive during bag replay.

3. Optional Google Colab inference
   - Used as a fallback if no suitable local NVIDIA GPU is available.
   - Consumes a prepared `colab_bundle`.
   - Exports Fusion-compatible mask and metadata files.

4. Native C++ hot paths on Windows
   - Accelerate per-point projection into masks.
   - Accelerate bulk GPS similarity transforms.

The pipeline is not a single monolithic script. It is a staged system with
branch points, file contracts, and output handoffs between domains.

## Top-Level Backend Entry Points

### Project root utilities

| File | Purpose |
| --- | --- |
| `install_repo_python_env.ps1` | Installs the Windows-side Python environment for the repo. |
| `requirements-repo-python.txt` | Curated Windows/Python dependency set for Fusion, inference, and repo-side utilities. |
| `requirements-repo-python-lock.txt` | Frozen version of the repo-side Python environment. |
| `requirements-repo-python-matlab.txt` | Optional MATLAB bridge dependency set. |
| `requirements-repo-python-matlab-lock.txt` | Frozen MATLAB bridge dependency set. |
| `project_paths.py` | Central relative-path helper for repo-root resources. |

### Backend subtrees

| Folder | Purpose |
| --- | --- |
| `rosbag_preprocessing/` | Dockerized ROS pipeline for calibration and pose recovery from bags. |
| `fusion/` | Semantic fusion, local/Colab inference preparation, GPS alignment, and map export. |

## Architecture Decisions

### 1. Split the pipeline by runtime domain

The repo deliberately does not force all stages into one runtime:

- ROS and FAST-LIO stay inside Docker because they depend on Linux, ROS Noetic,
  catkin workspaces, and GUI/OpenGL behavior best handled through WSL.
- Fusion and inference orchestration stay on the Windows Python side because
  they integrate with local files, Open3D, PyTorch, and later GUI code.
- Colab is optional because inference generally runs well on a local RTX-class
  GPU, but Colab remains a fallback for weaker machines or cloud-backed runs.

### 2. Use mounted output roots for portability

Earlier versions wrote outputs to container-internal `/home/...` locations. The
current design writes user-facing outputs into mounted repo folders:

- `rosbag_preprocessing/outputs/calibration/`
- `rosbag_preprocessing/outputs/pose_recovery/`

This allows downstream Windows-side scripts to consume files without guessing
container-internal paths.

### 3. Preserve staged catkin `devel` spaces in Docker

The Docker image is runtime-oriented, not a clean source build. The repo stages
known-good WSL runtime artifacts into the image context. This is why internal
container paths such as `/home/portable/ws_calib` and `/home/portable/ws_livox`
still exist even though user-facing paths were made relative and portable.

This is a deliberate tradeoff:

- Pros: fast image rebuilds, preserves known-good ROS behavior.
- Cons: container internals are not fully relocatable.

### 4. Treat every stage as a file-contract boundary

Each pipeline stage produces explicit files consumed by the next stage. This is
the primary architectural pattern used throughout the backend:

- pose recovery -> `images/`, `image_timestamps.csv`, `tf_camera_out.csv`, `tf_gps_out.csv`, `pcd/scans.pcd`
- inference -> `masks_npz/`, `meta_json/`, optional `pred_images/`
- fusion -> `fused_objects.json`, `fused_semantic_map.ply`, `fused_semantic_labels.npz`, `pole_neighbor_distances.json`
- GPS georeferencing -> `gps_alignment.json`, georeferenced JSONs, aligned CSV

This makes debugging easier and allows stages to be rerun independently.

## Repository File Catalog

### Shared path helper

#### `project_paths.py`

This module defines repo-relative roots:

```python
PROJECT_ROOT = Path(__file__).resolve().parent
ASSETS_DIR = PROJECT_ROOT / "assets"
FUSION_DIR = PROJECT_ROOT / "fusion"
ROSBAG_PREPROCESSING_DIR = PROJECT_ROOT / "rosbag_preprocessing"
OUTPUTS_DIR = PROJECT_ROOT / "outputs"
```

Use this style rather than hardcoded machine paths. It is the convention the
repo follows for Windows-side scripts.

### `rosbag_preprocessing/` catalog

| File | Purpose |
| --- | --- |
| `Dockerfile` | Builds the runtime container on top of `osrf/ros:noetic-desktop-full-focal`. |
| `compose.yaml` | Defines the `portable-ros-stack` service, mounted outputs, and WSLg/GPU passthrough. |
| `docker/ros_entrypoint.sh` | Sources ROS and staged catkin workspaces inside the container. |
| `docker/run_pipeline.sh` | Dispatches container runs into `calibration`, `pose-recovery`, or plain shell mode. |
| `launcher/run_pipeline.py` | Windows entrypoint that calls Docker through WSL and normalizes paths. |
| `launcher/run_calibration_workflow.py` | Thin wrapper for calibration mode. |
| `launcher/run_transform_reading_workflow.py` | Thin wrapper for pose-recovery mode. |
| `scripts/sync_wsl_context.sh` | Copies staged WSL runtime artifacts into the Docker build context and patches workflow scripts. |
| `scripts/build_image_wsl.sh` | Rebuilds the Docker image from WSL. |
| `scripts/save_image_wsl.sh` | Utility helper for WSL-side image management. |
| `overrides/ws_livox/scripts/tf_sample_camera_gps.py` | Samples `/tf` on every image and GPS event and exports JPGs and CSVs. |
| `overrides/ws_livox/scripts/run_pose_recovery_camera_gps.sh` | End-to-end pose-recovery orchestration around FAST-LIO and the sampler. |
| `outputs/pose_recovery/.gitkeep` | Placeholder so the output folder exists in the repo. |
| `outputs/calibration/.gitkeep` | Placeholder so the output folder exists in the repo. |
| `dist/.gitkeep` | Placeholder for distributable image exports. |
| `imgui.ini` | GUI state file used by some staged applications inside the runtime. |
| `README.md` | User-facing setup and usage instructions for the preprocessing stack. |

### `fusion/` catalog

| File | Purpose |
| --- | --- |
| `fuse_masks_to_slam.py` | Core semantic fusion engine. |
| `georeference_from_tf_gps.py` | Fits a local-to-GPS transform and writes GPS into object/powerline outputs. |
| `run_yolo_inference.py` | Local-or-Colab inference dispatcher for JPGs exported from pose recovery. |
| `yolo_inference_common.py` | Shared helpers for inference export, bundling, timestamps, and output validation. |
| `colab/run_inference_colab.py` | Colab-side runner that mirrors local inference export behavior. |
| `import_colab_inference_results.py` | Copies completed Colab outputs back into a local Fusion run folder. |
| `export_powerlines_to_leaflet.py` | Converts generated wire outputs into a Leaflet-friendly JSON overlay. |
| `launch_leaflet_viewer.ps1` | Serves the Leaflet viewer locally. |
| `leaflet_viewer/` | Browser-side map viewer assets. |
| `native/pointcloud_accel.cpp` | C++ projection kernel for point-to-mask assignment. |
| `native/pointcloud_accel.py` | Python wrapper for the point-cloud accelerator DLL. |
| `native/build_accel.py` | Builds `pointcloud_accel.dll` with Visual Studio tools. |
| `native/geospatial_accel.cpp` | C++ similarity-transform kernel for bulk GPS transforms. |
| `native/geospatial_accel.py` | Python wrapper for the geospatial accelerator DLL. |
| `native/build_geospatial_accel.py` | Builds `geospatial_accel.dll` with Visual Studio tools. |
| `sample_intrinsics.json` | Example camera intrinsics input. |
| `sample_extrinsics.json` | Example extrinsics input. |
| `sample_pose.csv` | Example pose CSV input. |
| `sample_image_timestamps.csv` | Example image timestamp CSV input. |
| `calibration_test/` | Saved example outputs and synthetic validation inputs. |
| `README.md` / `README.txt` | Focused Fusion usage docs. |
| `ENGR498_VIEWER_INTEGRATION_GUIDE.md` | Viewer integration notes for the ENGR498 codebase. |

## Runtime Domains in Detail

## 1. Repo Python environment setup

### Entry point

- `install_repo_python_env.ps1`

### What it does

This script installs the Windows-side Python environment used by:

- Fusion scripts,
- local YOLO inference,
- Open3D visualization,
- viewer-related Python utilities.

Key behaviors:

- accepts `-Python` so the caller can target an existing venv,
- supports `-UseLockFile` for exact installs,
- supports `-IncludeMatlabEngine` for optional MATLAB bridge setup.

Example:

```powershell
powershell -ExecutionPolicy Bypass -File .\install_repo_python_env.ps1 `
  -Python .\SeniorDesignProject\Scripts\python.exe `
  -UseLockFile
```

## 2. Dockerized ROS preprocessing domain

This domain handles two branches:

1. calibration
2. pose recovery / transform reading

Both are launched from Windows Python but executed inside Docker through WSL.

### 2.1 Windows launcher layer

#### `rosbag_preprocessing/launcher/run_pipeline.py`

This is the main Windows-to-WSL-to-Docker bridge.

Responsibilities:

- derive the project root from `__file__`,
- convert Windows paths like `C:\...` into WSL/container-visible `/mnt/c/...`,
- call:
  - `docker compose run -T --rm portable-ros-stack ...`
- dispatch the container mode:
  - `calibration`
  - `pose-recovery`
  - `bash`

Important function flow:

```python
to_wsl_path() -> normalize_workflow_args() -> run_wsl()
```

#### Thin launchers

- `run_calibration_workflow.py`
- `run_transform_reading_workflow.py`

These do not implement logic themselves. They forward arguments into
`run_pipeline.py` with the correct mode name.

### 2.2 Container definition layer

#### `rosbag_preprocessing/Dockerfile`

The image is built from:

```dockerfile
FROM osrf/ros:noetic-desktop-full-focal
```

It installs runtime dependencies for:

- ROS Noetic,
- OpenGL/X11/WSLg interop,
- image transport and CV bridge,
- bag replay,
- FAST-LIO runtime support.

It then copies a staged runtime into:

- `/home/portable`
- `/usr/local/lib`
- `/usr/local/share`

This staged runtime is generated by `scripts/sync_wsl_context.sh`.

#### `rosbag_preprocessing/compose.yaml`

This file makes the container behave like a portable ROS workstation.

Key decisions:

- `network_mode: host` for ROS networking simplicity,
- `gpus: all` for GUI/OpenGL acceleration,
- bind-mount the repo to `/workspace`,
- bind-mount the Windows drive root,
- bind-mount WSLg/X11 and runtime sockets,
- define mounted output roots:
  - `/workspace/outputs/calibration`
  - `/workspace/outputs/pose_recovery`

### 2.3 Container startup layer

#### `docker/ros_entrypoint.sh`

This script prepares the runtime inside the container:

1. source `/opt/ros/noetic/setup.bash`
2. export `HOME` and `PORTABLE_ROS_HOME`
3. prepend staged library paths to `LD_LIBRARY_PATH`
4. source staged catkin workspaces:
   - `/home/portable/ws_calib/devel/setup.bash`
   - `/home/portable/ws_livox/devel/setup.bash`
5. prepend staged script folders to `PATH`
6. exec the requested command

This is the reason the container can run staged workflow scripts as if it were
the original WSL environment.

#### `docker/run_pipeline.sh`

This script dispatches container mode:

- `calibration`
- `pose-recovery`
- `bash`

It also injects mounted output roots if the caller did not explicitly set them.

Behavior summary:

- `calibration` -> append `--run-root /workspace/outputs/calibration`
- `pose-recovery` -> append `--output-root /workspace/outputs/pose_recovery`

### 2.4 Runtime staging layer

#### `scripts/sync_wsl_context.sh`

This is a critical architecture file.

It copies runtime artifacts from the known-good WSL setup into the Docker build
context, including:

- `ws_calib/devel`
- `ws_livox/devel`
- selected source trees from:
  - `ws_calib/src/direct_visual_lidar_calibration`
  - `ws_livox/src/FAST_LIO`
  - `ws_livox/src/livox_ros_driver`
- staged scripts from:
  - `~/ws_calib/scripts`
  - `~/ws_livox/scripts`
- local runtime libs:
  - `libiridescence.so`
  - `libglfw_hint_shim.so`
  - GTSAM, Ceres, and related `/usr/local/lib` artifacts

It also patches staged shell scripts so they:

- source ROS safely in-container,
- write outputs to mounted repo folders,
- avoid forcing the software cursor,
- handle container-visible dataset paths.

This is the mechanism that preserves the working WSL ROS environment while
keeping Docker rebuilds fast.

## 3. Calibration branch

### User-facing entry

- `python rosbag_preprocessing/launcher/run_calibration_workflow.py <dataset>`

### Backend behavior

The repo-authored code does not reimplement the internals of direct visual
LiDAR calibration. Instead, it orchestrates the staged calibration workflow
inside the container.

What our backend is responsible for:

1. launch the Docker container,
2. source the staged `ws_calib` workspace,
3. route outputs to `rosbag_preprocessing/outputs/calibration`,
4. preserve the OpenGL shim and GUI runtime assumptions that made the native
   WSL workflow usable.

What the staged calibration workflow does conceptually:

1. ensure `roscore` is running,
2. normalize bag image topics if needed,
3. preprocess one or more bags in a dataset directory,
4. invoke manual initial-guess selection,
5. run nonlinear calibration refinement,
6. launch the calibration result viewer.

This branch is important because the fusion branch depends on the resulting
camera intrinsics and LiDAR-to-camera extrinsics.

## 4. Pose recovery / transform reading branch

### User-facing entry

- `python rosbag_preprocessing/launcher/run_transform_reading_workflow.py <bag>`

### Main orchestration file

- `rosbag_preprocessing/overrides/ws_livox/scripts/run_pose_recovery_camera_gps.sh`

### Purpose

This script replays a ROS bag, runs FAST-LIO, and samples the LiDAR pose every
time an image or GPS message is observed.

### Step-by-step execution

1. Resolve the bag path.
2. Create a run folder under:
   - `rosbag_preprocessing/outputs/pose_recovery/<bag_stem>_<timestamp_pid>/`
3. Create output subfolders:
   - `logs/`
   - `pcd/`
   - `images/`
4. Detect image and GPS topics from the bag unless the user overrides them.
5. Start a clean ROS master.
6. Enable simulated time for replay.
7. Launch FAST-LIO.
8. Launch `tf_sample_camera_gps.py`.
9. Replay the bag paused, then auto-unpause with `expect`.
10. Wait for the sampler to finish.
11. Copy the final FAST-LIO map to:
    - `pcd/scans.pcd`

### Outputs

Each run produces:

- `images/frame_000001.jpg`, ...
- `image_timestamps.csv`
- `tf_camera_out.csv`
- `tf_gps_out.csv`
- `pcd/scans.pcd`
- `logs/*.log`

### Sampler internals

#### `tf_sample_camera_gps.py`

This script is event-driven. It does not sample `/tf` on a timer. It samples
`/tf` when semantic downstream consumers will actually need it:

- on image messages,
- on GPS messages.

Important classes and responsibilities:

- `ClockMonitor`
  - Tracks `/clock` during simulated bag replay.
- `TFStampMonitor`
  - Tracks `/tf` availability and stall state.
- `ImageExportWriter`
  - Converts incoming `CompressedImage` or `Image` messages into JPG files and
    writes `image_timestamps.csv`.
- `TimebaseState`
  - Selects whether replay progress is tracked by `/clock` or TF stamps.

Key output schema decisions:

#### `image_timestamps.csv`

```csv
filename,t_query_sec,t_in_sec
frame_000001.jpg,1771469608.123456,0.000000
```

#### `tf_camera_out.csv`

```csv
t_in_sec,t_query_sec,x,y,z,qx,qy,qz,qw,status
```

#### `tf_gps_out.csv`

```csv
t_in_sec,t_query_sec,x,y,z,qx,qy,qz,qw,status,latitude_deg,longitude_deg,altitude_m,fix_status,service,cov_xx_m2,cov_yy_m2,cov_zz_m2,covariance_type
```

### Algorithm detail inside the sampler

For every event:

1. infer its timestamp from the ROS header,
2. initialize the time base the first time an event is processed,
3. wait until replay time reaches the event timestamp,
4. query TF at that exact timestamp:
   - `target = camera_init`
   - `source = body`
5. write a CSV row with either:
   - the pose and `OK`,
   - or a status explaining why the sample failed

This produces pose data aligned to the actual image and GPS message times, not
to arbitrary replay intervals.

## 5. YOLO inference domain

The inference domain begins from the JPGs exported by pose recovery.

### Required input contract

`run_yolo_inference.py` expects a pose-recovery run folder containing:

- `images/`
- `image_timestamps.csv`

This contract is enforced by `yolo_inference_common.resolve_pose_recovery_inputs`.

### Main dispatcher

#### `fusion/run_yolo_inference.py`

This file implements three runtime modes:

- `auto`
- `local`
- `colab`

### Runtime branch behavior

#### `--runtime local`

Always run inference locally on a selected CUDA device.

#### `--runtime auto`

1. probe local runtime:
   - is `torch` installed?
   - is `ultralytics` installed?
   - is CUDA available?
   - is the selected device modern enough?
2. if yes -> run local inference
3. if not -> prepare a `colab_bundle`

#### `--runtime colab`

Always prepare a Colab bundle without attempting local inference.

### Local GPU probing

`probe_local_runtime()` checks:

- CUDA availability in PyTorch,
- device existence,
- capability major version,
- whether the GPU is modern enough for the intended workflow.

### Local inference path

`run_local_inference()`:

1. loads YOLO weights,
2. runs `model.predict()` on the exported JPG directory,
3. uses `export_ultralytics_results()` to write Fusion-compatible outputs,
4. writes `inference_manifest.json`

### Output contract for Fusion

For every image `frame_000001.jpg`, inference must produce:

- `masks_npz/frame_000001_masks.npz`
- `meta_json/frame_000001_meta.json`

The `.npz` contains:

- `masks`: `(N, H, W)` binary masks
- `cls`: class IDs
- `conf`: confidences

The metadata JSON contains:

- `source_image`
- `orig_shape`
- `names`
- `detections[]`

Each detection includes:

- `instance_index`
- `class_id`
- `class_name`
- `confidence`
- `box_xyxy`

### Shared export helpers

#### `fusion/yolo_inference_common.py`

Key responsibilities:

- validate pose-recovery run inputs,
- manage output directories,
- copy inputs into a Colab bundle,
- zip the bundle,
- find default YOLO weight candidates,
- export Ultralytics results into the repo contract.

### Colab fallback

#### `fusion/colab/run_inference_colab.py`

This script is designed to run inside Google Colab on a prepared `colab_bundle`.

Responsibilities:

1. optionally mount Google Drive,
2. load `colab_inference_config.json`,
3. resolve weights from:
   - explicit arg,
   - bundled weight file,
   - configured Drive path,
4. ensure CUDA is available,
5. report whether the preferred GPU name matched the actual assignment,
6. run YOLO inference,
7. export results in the same format as local inference

Important note: the script can prefer or report an A100, but it cannot force
Colab to assign one. It uses whatever CUDA device Colab actually provides.

### Colab bundle contents

Prepared by `run_yolo_inference.py`, the bundle includes:

- `images/`
- `image_timestamps.csv`
- `colab_inference_config.json`
- `run_inference_colab.py`
- `yolo_inference_common.py`
- optionally `best.pt`

After Colab inference, the same bundle root contains:

- `pred_images/`
- `masks_npz/`
- `meta_json/`
- `combined_class_masks/`

### Importing Colab results back locally

#### `fusion/import_colab_inference_results.py`

This script validates that the Colab results cover the full image set, then
copies them into the local Fusion run directory and writes an
`inference_manifest.json`.

## 6. Semantic fusion domain

### Main entry point

- `fusion/fuse_masks_to_slam.py`

### Purpose

This script projects per-image YOLO segmentation masks into a global SLAM point
cloud using:

- camera intrinsics,
- LiDAR-to-camera extrinsics,
- time-aligned LiDAR poses,
- per-image timestamps.

### Inputs

- `--intrinsics-json`
- `--extrinsics-json`
- `--pose-csv`
- `--image-timestamps-csv`
- `--point-cloud`
- `--mask-dir`
- `--meta-dir`

### Core data classes

- `CameraIntrinsics`
- `PoseRecord`
- `FrameRecord`

These create typed boundaries between raw file parsing and later computation.

### Supported classes

Allowed by default:

- `pole`
- `crossarm`
- `transformer`
- `pedestal`
- `pedestal_box`
- `pedestal box`

Rejected by default:

- wire/powerline synonyms

Pedestal support was explicitly enabled so pedestal-like classes can pass
through the same fusion path as poles, crossarms, and transformers.

### Detailed algorithm

#### Step 1: parse calibration and timing inputs

Functions:

- `load_intrinsics()`
- `load_extrinsics()`
- `load_pose_records()`
- `load_image_timestamps()`
- `build_frame_records()`

Behavior:

- supports multiple intrinsics JSON layouts,
- supports multiple extrinsics JSON layouts,
- filters pose rows by `status`,
- sorts poses and frames by time,
- matches masks and metadata by image stem.

#### Step 2: nearest-pose matching

Function:

- `match_nearest_pose_indices()`

Behavior:

- pairs each frame timestamp with the nearest pose timestamp,
- uses nearest-neighbor matching in time,
- supports a global time offset with `--time-offset-sec`.

#### Step 3: point projection into masks

This is the hottest loop in the fusion backend.

Functions:

- Python wrapper: `native.pointcloud_accel.project_assign_best_detection()`
- C++ kernel: `native/pointcloud_accel.cpp`

Algorithm in the native kernel:

1. transform map point -> LiDAR frame using `map_to_lidar`,
2. transform LiDAR point -> camera frame using `lidar_to_cam`,
3. reject points behind the camera,
4. normalize by depth,
5. apply plumb-bob-style distortion if present,
6. project to integer pixel coordinates,
7. test that pixel against every allowed instance mask,
8. keep the highest-confidence allowed detection for that point

This produces, for every map point:

- best detection index
- best detection confidence

#### Step 4: dense-cluster filtering per detection

Function:

- `keep_largest_dense_cluster()`

Purpose:

- suppress sparse false-positive fragments by keeping only the densest cluster
  for each detection’s projected points.

Implementation:

- adaptive epsilon from k-nearest-neighbor distances,
- Open3D `cluster_dbscan`,
- keep only the largest dense component.

#### Step 5: voting across frames

Global arrays track:

- `best_conf`
- `vote_count`
- `winner_class`

Behavior:

- each frame contributes votes to the points it labels,
- the best-confidence class wins per point,
- optional `--min-vote-to-keep` removes low-support assignments after all frames.

#### Step 6: class-wise instance segmentation

Function:

- `segment_class_instances()`

Behavior:

- collect final winning points for one class,
- estimate clustering epsilon adaptively,
- cluster in:
  - XY for poles,
  - full 3D for other classes,
- apply Open3D statistical outlier removal per cluster,
- compute:
  - centroid,
  - AABB,
  - mean confidence,
  - point count

#### Step 7: fragment merging

Two post-clustering merge passes correct common utility-scene artifacts.

##### Pole merge

Function:

- `merge_pole_fragments()`

Why:

- poles are vertical objects and frequently get split into stacked or partial
  fragments.

How:

- merge pole instances whose XY boxes overlap.

##### Transformer merge

Function:

- `merge_transformer_fragments()`

Why:

- transformers can be split into multiple small clusters near the same pole.

How:

- find the nearest supporting pole for each transformer fragment,
- only compare fragments sharing the same support pole,
- merge if they overlap or are close in XY.

#### Step 8: final object naming

Function:

- `assign_instance_names()`

Naming convention:

- `pole_01`
- `crossarm_01`
- `transformer_01`
- `pedestal_01`

#### Step 9: pole spacing computation

Functions:

- `compute_pole_neighbor_distances()`
- `build_pole_distance_geometry()`

Algorithm:

1. take final pole centroids,
2. compute all pairwise horizontal distances,
3. reject links outside:
   - a minimum spacing,
   - an adaptive maximum spacing,
4. keep a sparse graph by limiting per-pole degree,
5. write edge metadata including:
   - horizontal distance,
   - delta Z,
   - 3D distance,
   - midpoint

#### Step 10: final outputs

Generated files:

- `fused_semantic_map.ply`
- `fused_semantic_labels.npz`
- `fused_objects.json`
- `pole_neighbor_distances.json`

Important schemas:

##### `fused_objects.json`

Each object contains:

- `object_name`
- `class_name`
- `instance_number`
- `confidence_score`
- `num_points`
- `centroid_map_xyz`
- `bbox_aabb_min_xyz`
- `bbox_aabb_max_xyz`
- `class_color_rgb`
- `gps`

##### `fused_semantic_labels.npz`

Contains per-point label arrays such as:

- `winner_class`
- `final_class_index`
- `final_instance_index`
- `best_conf`
- `vote_count`

## 7. Native acceleration layer

### `fusion/native/pointcloud_accel.cpp`

Role:

- accelerates projection and best-mask assignment.

Design:

- plain exported C function,
- OpenMP parallel loop when available,
- distortion model applied in C++ for speed,
- called through `ctypes`.

### `fusion/native/geospatial_accel.cpp`

Role:

- accelerates bulk SE(2)/similarity-style transforms for GPS application.

Design:

- exported C function,
- simple per-point transform:
  - scale
  - yaw rotation in XY
  - translation in ENU

### Python wrappers

- `native/pointcloud_accel.py`
- `native/geospatial_accel.py`

Common behavior:

- locate or build the DLL,
- declare function signatures via `ctypes`,
- ensure contiguous NumPy arrays,
- expose Python-friendly functions.

### Build scripts

- `native/build_accel.py`
- `native/build_geospatial_accel.py`

Both:

- locate Visual Studio Build Tools via `vswhere.exe`,
- generate a temporary `.bat`,
- compile with:
  - `/LD`
  - `/O2`
  - `/std:c++17`
  - `/openmp`

## 8. GPS georeferencing domain

### Main entry point

- `fusion/georeference_from_tf_gps.py`

### Purpose

This script fits a transform from the local SLAM frame into GPS space using the
`tf_gps_out.csv` samples generated during pose recovery.

### Input assumptions

`tf_gps_out.csv` contains:

- LiDAR pose in the local map frame at each GPS event,
- GPS latitude/longitude/altitude,
- GPS quality fields from `sensor_msgs/NavSatFix`.

### Detailed algorithm

#### Step 1: load and filter samples

Functions:

- `load_tf_gps_records()`
- `filter_records()`
- `compute_weights()`

Behavior:

- skip rows whose TF status is not `OK`,
- skip rows without GPS,
- optionally reject rows by:
  - `fix_status`
  - horizontal covariance
- compute per-row weights from covariance when available.

#### Step 2: convert GPS to ENU

Functions:

- `geodetic_to_ecef()`
- `ecef_to_geodetic()`
- `ecef_to_enu()`
- `enu_to_ecef()`
- `enu_to_geodetic()`

Model:

- WGS84 ellipsoid,
- first GPS record becomes the ENU reference origin.

#### Step 3: convert LiDAR pose samples into local GPS antenna positions

Function:

- `local_gps_positions()`

Behavior:

- takes each LiDAR pose from `tf_gps_out.csv`,
- applies the measured GPS-to-LiDAR lever arm in the body frame,
- computes the local-frame GPS antenna position corresponding to each GPS fix.

#### Step 4: fit the local-to-GPS alignment

Functions:

- `weighted_rigid_2d()`
- `fit_alignment()`

Model:

- weighted SE(2) in horizontal plane,
- plus independent Z translation,
- optional uniform XY scale if `--allow-scale` is enabled.

Robustness:

- iterative outlier rejection over XY residuals,
- threshold based on median/MAD with a minimum floor.

The final alignment result contains:

- `scale`
- `yaw_rad`
- `translation_enu_m`
- reference LLH/ECEF
- inlier mask
- residuals
- RMSE metrics

#### Step 5: apply the transform

Functions:

- `apply_similarity_numpy()`
- `apply_similarity()`
- `points_to_gps()`

Branch:

- native mode on/auto -> use `geospatial_accel.dll` when available,
- otherwise -> NumPy fallback.

#### Step 6: georeference downstream JSONs

Functions:

- `georeference_objects_json()`
- `georeference_powerlines_json()`

Behavior:

- write `gps` into every fused object,
- write `gps` and `polyline_gps` into every powerline,
- append a `gps_alignment` provenance block.

#### Step 7: write reports

Generated files:

- `gps_alignment.json`
- `tf_gps_georeferenced.csv`
- optionally:
  - `fused_objects_georeferenced.json`
  - `eng498_powerlines_overlay_georeferenced.json`

## 9. Powerline overlay export

### Main entry point

- `fusion/export_powerlines_to_leaflet.py`

### Purpose

This script does not compute wires. It adapts generated ENGR498 powerline
artifacts into a portable JSON overlay usable by the mapping layer.

Inputs:

- `wires_points.npz`
- optional `wire_info.json`
- optional `ground_points.npz`

Behavior:

1. load wire point sets,
2. order points along the dominant wire direction,
3. resample long polylines to a manageable size,
4. compute centroid and basic geometry metrics,
5. emit `powerlines[]` records with:
   - `centroid_map_xyz`
   - `polyline_map_xyz`
   - `fit_metadata`
   - placeholder GPS fields

This is intentionally an adapter layer, not a replacement for the wire
extraction algorithm.

## 10. Branches in the overall backend

The backend has several important branch points.

### Branch A: Calibration vs pose recovery

- Calibration branch:
  - used when deriving camera intrinsics / LiDAR-camera extrinsics.
- Pose recovery branch:
  - used when extracting `scans.pcd`, `tf_camera_out.csv`, `tf_gps_out.csv`,
    JPGs, and timestamps for a run.

### Branch B: Local inference vs Colab inference

- Local:
  - preferred when a suitable NVIDIA CUDA GPU exists.
- Colab:
  - used when no suitable local GPU is available or when explicitly requested.

### Branch C: GPS available vs GPS unavailable

- GPS unavailable:
  - Fusion still produces local-frame object outputs.
- GPS available:
  - `georeference_from_tf_gps.py` adds real-world coordinates.

### Branch D: Native accelerator on/off

- native `auto`
  - use DLL if present, otherwise fall back silently.
- native `on`
  - require DLL and fail loudly if unavailable.
- native `off`
  - force pure Python/NumPy.

## 11. Naming and file conventions

### Image naming

Exported pose-recovery frames use:

- `frame_000001.jpg`
- `frame_000002.jpg`

### Inference output naming

For image `frame_000001.jpg`, inference must write:

- `frame_000001_masks.npz`
- `frame_000001_meta.json`

### Object naming

Final objects use:

- `<class>_<two-digit instance>`

Examples:

- `pole_01`
- `transformer_02`
- `pedestal_01`

### Output-root convention

User-facing generated outputs should live under mounted repo folders, not
container-internal folders.

Primary output roots:

- `rosbag_preprocessing/outputs/calibration/`
- `rosbag_preprocessing/outputs/pose_recovery/`
- Fusion run output folder chosen by the caller

## 12. Example end-to-end backend flow

### Step A: preprocess a bag

```powershell
python .\rosbag_preprocessing\launcher\run_transform_reading_workflow.py `
  .\data\movingtest1.bag `
  --image-topic /camera/image/compressed `
  --gps-topic /fix
```

### Step B: run local inference or prepare Colab bundle

```powershell
python .\fusion\run_yolo_inference.py `
  --pose-recovery-run-dir .\rosbag_preprocessing\outputs\pose_recovery\<run_name> `
  --runtime auto `
  --weights ..\Colab\best.pt
```

### Step C: fuse masks into the SLAM cloud

```powershell
python .\fusion\fuse_masks_to_slam.py `
  --intrinsics-json .\fusion\sample_intrinsics.json `
  --extrinsics-json .\fusion\sample_extrinsics.json `
  --pose-csv .\rosbag_preprocessing\outputs\pose_recovery\<run_name>\tf_camera_out.csv `
  --image-timestamps-csv .\rosbag_preprocessing\outputs\pose_recovery\<run_name>\image_timestamps.csv `
  --point-cloud .\rosbag_preprocessing\outputs\pose_recovery\<run_name>\pcd\scans.pcd `
  --mask-dir .\rosbag_preprocessing\outputs\pose_recovery\<run_name>\yolo_inference\masks_npz `
  --meta-dir .\rosbag_preprocessing\outputs\pose_recovery\<run_name>\yolo_inference\meta_json `
  --output-dir .\fusion\outputs\<fusion_run>
```

### Step D: georeference the results

```powershell
python .\fusion\georeference_from_tf_gps.py `
  --tf-gps-csv .\rosbag_preprocessing\outputs\pose_recovery\<run_name>\tf_gps_out.csv `
  --objects-json .\fusion\outputs\<fusion_run>\fused_objects.json `
  --output-dir .\fusion\outputs\<fusion_run>\gps
```

## 13. Known limitations and future work

- The Docker image is externally portable but internally depends on staged
  catkin `devel` spaces and fixed `/home/portable/...` paths.
- Calibration internals remain delegated to the staged direct visual LiDAR
  calibration workflow rather than repo-authored source.
- Colab fallback is implemented as a file-contract workflow, not yet as a
  one-click GUI action.
- Powerline extraction itself remains owned by the existing ENGR498 pipeline;
  this repo currently adapts its outputs rather than replacing it.
- A full top-level orchestrator that chains:
  - pose recovery,
  - inference,
  - fusion,
  - georeferencing,
  - final export
  is still a logical next step.

## 14. Summary

The backend is a staged robotics/data-fusion pipeline with three major design
principles:

1. keep ROS-heavy preprocessing in Docker and WSL-backed Linux,
2. keep semantic fusion and georeferencing in Windows-side Python,
3. connect stages through explicit file contracts rather than hidden state.

The result is a pipeline that can:

- extract a SLAM map and time-aligned transforms from rosbags,
- export per-frame JPGs for inference,
- run local YOLO segmentation or prepare a Colab fallback,
- fuse semantic masks into a 3D point cloud,
- segment infrastructure instances,
- compute pole spacing,
- assign GPS coordinates to fused assets and powerline overlays.

That backend is the foundation the future GUI layer should call, not replace.
