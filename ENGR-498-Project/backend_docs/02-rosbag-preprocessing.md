# ROS Bag Preprocessing

[Previous: System Overview](01-system-overview.md) | [Back to index](README.md) | [Next: Inference](03-inference.md)

## Purpose

This stage turns a rosbag into:

- a FAST-LIO SLAM map,
- exported JPG frames,
- timestamp-aligned LiDAR poses for images,
- timestamp-aligned LiDAR poses for GPS fixes.

It lives in:

- `rosbag_preprocessing/`

## Primary Files

| File | Purpose |
| --- | --- |
| `launcher/run_pipeline.py` | Windows-to-WSL-to-Docker bridge |
| `launcher/run_calibration_workflow.py` | Thin wrapper for calibration mode |
| `launcher/run_transform_reading_workflow.py` | Thin wrapper for pose recovery mode |
| `docker/run_pipeline.sh` | Container-side mode dispatcher |
| `docker/ros_entrypoint.sh` | Sources ROS and staged catkin workspaces |
| `Dockerfile` | Runtime image |
| `compose.yaml` | Service definition, GPU/WSLg mounts, output mounts |
| `scripts/sync_wsl_context.sh` | Stages WSL runtime artifacts into the build context |
| `overrides/ws_livox/scripts/run_pose_recovery_camera_gps.sh` | End-to-end pose recovery workflow |
| `overrides/ws_livox/scripts/tf_sample_camera_gps.py` | Event-driven TF/image/GPS sampler **and** dense trajectory sampler |
| `overrides/ws_livox/scripts/sanitize_pose_recovery_outputs.py` | Post-run sanitization for GPS and dense trajectory CSVs |

## Calibration Branch

The calibration workflow is orchestrated from this stack, but the detailed
calibration internals remain part of the staged direct visual LiDAR
calibration runtime. The repo-side role is to:

- launch the workflow reproducibly,
- route outputs to mounted folders,
- preserve the working GUI/OpenGL runtime assumptions.

Use this branch when you need:

- camera intrinsics,
- LiDAR-to-camera extrinsics.

## Pose Recovery Branch

The pose recovery branch is used for backend processing runs that feed Fusion.

### Inputs

- one ROS bag
- LiDAR + IMU topics for FAST-LIO
- image topic, usually `/camera/image/compressed`
- GPS topic, usually `/fix`

### Outputs

- `images/`
- `image_timestamps.csv`
- `tf_camera_out.csv`
- `tf_gps_out.csv`
- `tf_dense_trajectory.csv`
- `pcd/scans.pcd`
- run logs

## Pose Recovery Flow

1. Resolve the bag path.
2. Create a unique run directory under `outputs/pose_recovery/`.
3. Detect image and GPS topics or apply manual overrides.
4. Start a clean ROS master.
5. Enable simulated time.
6. Start FAST-LIO.
7. Start `tf_sample_camera_gps.py` (launches both the event sampler and the dense trajectory sampler).
8. Replay the bag paused, then auto-unpause.
9. Wait for sampling to finish.
10. Copy the final FAST-LIO PCD into the run directory.
11. Run `sanitize_pose_recovery_outputs.py` on both `tf_gps_out.csv` and `tf_dense_trajectory.csv`.

## TF / Image / GPS Sampling Model

`tf_sample_camera_gps.py` now runs **two sampling paths in parallel**:

### Event-driven path (unchanged)

Samples `/tf` at the moment each image or GPS message arrives. This is the original
design. It produces:

- `tf_camera_out.csv` — one row per camera frame
- `tf_gps_out.csv` — one row per GPS fix (with GPS coordinates appended); in the
  explicit GPS-optional developer mode this file may sanitize down to just the header

These files are used by:
- `fuse_masks_to_slam.py` when time-offset is disabled (nearest-pose matching)
- `georeference_from_tf_gps.py` for GPS alignment when usable GPS rows exist

### Dense trajectory path (new)

`DenseTrajectorySampler` runs in a background daemon thread. It is **timer-driven**,
not event-driven. It samples `/tf` every 10 ms (default) from the first TF stamp
through to the end of the bag. This produces:

- `tf_dense_trajectory.csv` — one row per 10 ms interval over the full bag

This file is used by `fuse_masks_to_slam.py` when time-offset is **enabled** and
the file is present. It allows accurate interpolation at arbitrary shifted
timestamps because the dense grid captures `tf2_ros`'s own internal inter-message
interpolation and stores it compactly.

`timestamp_sec` is exported in the same unix/header time domain used by
`image_timestamps.csv:t_query_sec` and `tf_camera_out.csv:t_query_sec`. TF
lookups still happen in raw TF time internally; only the exported dense CSV
timestamps are converted.

The sampling lag (staying slightly behind the bag clock) prevents `ExtrapolationException`
by ensuring each queried timestamp is already in the `tf2_ros` buffer.

After sampling, `sanitize_pose_recovery_outputs.py` cleans the dense CSV:
- removes non-OK rows (lookup failures, extrapolations)
- removes rows with invalid quaternion norm
- sorts by timestamp and deduplicates

### Important classes

| Class | Role |
| --- | --- |
| `ClockMonitor` | Tracks `/clock` messages for sim-time synchronization |
| `TFStampMonitor` | Tracks first/last `/tf` stamp; used by both sampling paths |
| `ImageExportWriter` | Async JPEG export thread |
| `TimebaseState` | Decides whether sim-time or TF-stamp is used as the time reference |
| `DenseTrajectorySampler` | Background thread that produces `tf_dense_trajectory.csv` |

### Output conventions

- image filenames are sequential: `frame_000001.jpg`, `frame_000002.jpg`, ...
- timestamps are written to `image_timestamps.csv`
- all pose CSVs use XYZW quaternion order
- `tf_dense_trajectory.csv` uses `timestamp_sec` (not `t_query_sec`) as its
  primary column to distinguish it from the event-driven files, while staying in
  the same unix/header time domain as the event-driven `t_query_sec` fields

## Why This Stage Matters

Everything downstream depends on these outputs:

- inference consumes `images/`
- fusion (no-offset mode) consumes:
  - `image_timestamps.csv`
  - `tf_camera_out.csv`
  - `pcd/scans.pcd`
- fusion (time-offset mode) additionally consumes:
  - `tf_dense_trajectory.csv` (preferred)
  - or `tf_camera_out.csv` as sparse interpolation fallback
- georeferencing consumes `tf_gps_out.csv`

If this stage is wrong, every later stage is misaligned.

## Where To Change Behavior

| Goal | File to change |
| --- | --- |
| Change Windows-to-Docker launch behavior | `launcher/run_pipeline.py` |
| Change container output-root injection | `docker/run_pipeline.sh` |
| Change Docker environment/mounts | `compose.yaml` |
| Change runtime staging behavior | `scripts/sync_wsl_context.sh` |
| Change pose recovery orchestration | `overrides/ws_livox/scripts/run_pose_recovery_camera_gps.sh` |
| Change how image/GPS events are sampled | `overrides/ws_livox/scripts/tf_sample_camera_gps.py` |

[Previous: System Overview](01-system-overview.md) | [Back to index](README.md) | [Next: Inference](03-inference.md)
