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
| `overrides/ws_livox/scripts/tf_sample_camera_gps.py` | Event-driven TF/image/GPS sampler |

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
- `pcd/scans.pcd`
- run logs

## Pose Recovery Flow

1. Resolve the bag path.
2. Create a unique run directory under `outputs/pose_recovery/`.
3. Detect image and GPS topics or apply manual overrides.
4. Start a clean ROS master.
5. Enable simulated time.
6. Start FAST-LIO.
7. Start `tf_sample_camera_gps.py`.
8. Replay the bag paused, then auto-unpause.
9. Wait for sampling to finish.
10. Copy the final FAST-LIO PCD into the run directory.

## TF / Image / GPS Sampling Model

`tf_sample_camera_gps.py` is event-driven, not timer-driven.

It samples `/tf`:

- when an image message arrives,
- when a GPS message arrives.

That is a key design choice. It means downstream stages receive pose samples at
the exact times they need rather than at an arbitrary fixed rate.

### Important classes

- `ClockMonitor`
- `TFStampMonitor`
- `ImageExportWriter`
- `TimebaseState`

### Output conventions

- image filenames are sequential:
  - `frame_000001.jpg`
  - `frame_000002.jpg`
- timestamps are written to `image_timestamps.csv`
- pose rows use XYZW quaternion order

## Why This Stage Matters

Everything downstream depends on these outputs:

- inference consumes `images/`
- fusion consumes:
  - `image_timestamps.csv`
  - `tf_camera_out.csv`
  - `pcd/scans.pcd`
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
