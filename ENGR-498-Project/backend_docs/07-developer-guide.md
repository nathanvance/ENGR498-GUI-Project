# Developer Guide

[Previous: File Contracts](06-file-contracts.md) | [Back to index](README.md) | [Next: Validation And Troubleshooting](08-validation-and-troubleshooting.md)

## Where To Change Behavior

| Goal | File to change |
| --- | --- |
| Change scan metadata normalization / file resolution | `scan_metadata.py` |
| Change dashboard-to-backend orchestration | `gui_pipeline.py` |
| Change the integrated dashboard window | `testDashboard.py` |
| Change how the step-by-step dashboard launches stages | `views/lidar_dashboard_stepbystep.py` |
| Change the main GUI entrypoint | `main.py` |
| Change combined wire + Fusion semantic rendering | `Matlab_ExtractPowerLine/testSemanticLidarViewer.py` |
| Change MATLAB wire extraction wrapper | `Matlab_ExtractPowerLine/testMatlab.py` |
| Change Windows-to-Docker launch behavior | `rosbag_preprocessing/launcher/run_pipeline.py` |
| Change output-root injection | `rosbag_preprocessing/docker/run_pipeline.sh` |
| Change runtime staging behavior | `rosbag_preprocessing/scripts/sync_wsl_context.sh` |
| Change image/GPS TF sampling or dense trajectory sampling | `rosbag_preprocessing/overrides/ws_livox/scripts/tf_sample_camera_gps.py` |
| Change pose recovery orchestration | `rosbag_preprocessing/overrides/ws_livox/scripts/run_pose_recovery_camera_gps.sh` |
| Change dense trajectory sanitization | `rosbag_preprocessing/overrides/ws_livox/scripts/sanitize_pose_recovery_outputs.py` |
| Change local-vs-Colab inference logic | `fusion/run_yolo_inference.py` |
| Change inference bundle/export helpers | `fusion/yolo_inference_common.py` |
| Change Colab runtime behavior | `fusion/colab/run_inference_colab.py` |
| Change fusion class filtering | `fusion/fuse_masks_to_slam.py` |
| Change clustering / merge behavior | `fusion/fuse_masks_to_slam.py` |
| Change dense trajectory interpolation (SLERP/lerp) | `query_dense_trajectory()` in `fusion/fuse_masks_to_slam.py` |
| Change quaternion sign continuity enforcement | `_enforce_quaternion_sign_continuity()` in `fusion/fuse_masks_to_slam.py` |
| Change dense trajectory loading/validation | `load_dense_trajectory()` in `fusion/fuse_masks_to_slam.py` |
| Change projection kernel | `fusion/native/pointcloud_accel.cpp` |
| Change GPS fit model | `fusion/georeference_from_tf_gps.py` |
| Change geospatial kernel | `fusion/native/geospatial_accel.cpp` |
| Change powerline export schema | `fusion/export_powerlines_to_leaflet.py` |

## Common Developer Workflows

### Add a new object class

1. Make sure the segmentation model emits the class name you want.
2. Add the class to the allow-list or pass it through CLI options.
3. Add a stable color if needed.
4. Decide whether the class needs custom clustering rules.
5. Validate that the class shows up in `fused_objects.json`.

### Change clustering behavior

Start in:

- `segment_class_instances()`
- `keep_largest_dense_cluster()`
- merge helpers inside `fuse_masks_to_slam.py`

Tune:

- `--eps-factor`
- `--min-cluster-points`
- `--stat-nb-neighbors`
- `--stat-std-ratio`

### Change GPS alignment behavior

Start in:

- `weighted_rigid_2d()`
- `fit_alignment()`
- `filter_records()`

### Change how outputs are exported

Start in:

- `yolo_inference_common.py`
- `fuse_masks_to_slam.py`
- `georeference_from_tf_gps.py`

### Change how the GUI runs the backend

Start in:

- `testDashboard.py`
- `gui_pipeline.py`
- `scan_metadata.py`

This is where the current experimental branch decides:

- how scan folders are structured,
- where rosbag-preprocessing outputs are redirected,
- how wire extraction is launched and where it writes outputs,
- when the GUI prompts for calibration JSON files,
- how the dashboard reacts to a Colab fallback,
- how map launch and viewer launch resolve per-scan artifacts.

## Assumptions And Invariants

- Rosbag preprocessing must produce the input images used for inference.
- Inference must not rename image stems.
- Masks must be exported at the original image resolution.
- The Fusion stage expects pose timestamps and image timestamps to be comparable.
- GPS alignment assumes local geometry is rigid enough for weighted SE(2)+Z.
- The Docker ROS runtime depends on staged catkin `devel` spaces.
- `tf_dense_trajectory.csv` must come from the same preprocessing run as
  `tf_camera_out.csv`; the GUI enforces this automatically.
- The dense trajectory must be sanitized before Fusion reads it; the shell
  script does this automatically after each run.

## Dense Trajectory Developer Notes

The dense trajectory feature adds a continuous-time LiDAR pose record to every
preprocessing run. These notes cover the internal design decisions.

### Why a background thread rather than post-processing

The dense sampler runs inside the same `tf_sample_camera_gps.py` process as the
event-driven sampler. This is important because both need access to the same
`tf2_ros.Buffer` object, which only receives messages while the ROS node is
alive. Extracting `/tf` densely after the fact would require replaying the bag
again.

### Why a 50–100 ms lag behind the bag clock

`tf2_ros.Buffer.lookup_transform` raises `ExtrapolationException` if the
requested timestamp is ahead of the latest received TF message. The sampler
deliberately stays slightly behind the live bag clock (one interval ahead of
confirmed TF stamps) to avoid this. The lag is bounded by `interval_sec` and is
not visible in the output because each row records the actual query timestamp.

### Quaternion sign continuity

SLERP interpolation is only correct for rotations on the short arc. When a
quaternion trajectory crosses a hemisphere boundary, consecutive `q[i-1]` and
`q[i]` may have `dot < 0`, meaning SLERP would interpolate the long way around.
The loader enforces sign continuity with a single O(N) pass before the
`DenseTrajectory` object is returned. Individual SLERP calls then never need
per-call sign correction.

### Why quaternion norm_sq must be in [0.5, 2.0]

This is a loose gate that catches clearly broken quaternions (all-zeros, NaN
propagation, wild numerical drift) without rejecting quaternions that are merely
slightly un-normalized due to floating-point arithmetic. Accepted quaternions
are renormalized to unit length before being stored in the `DenseTrajectory`.

### Running the unit tests

```powershell
python -m pytest fusion\tests\test_dense_trajectory.py -v
```

Tests cover: sign continuity enforcement, SLERP edge cases, `query_dense_trajectory`
boundary conditions, `load_dense_trajectory` filtering/sorting/normalization,
and full load→query round-trips.

## Known Coupling Points

- Changing image orientation in inference without rotating masks back will
  silently break projection.
- Changing class labels in the model without updating Fusion allow/reject rules
  will silently drop detections.
- Changing CSV schemas without updating downstream loaders will break later
  stages.
- Changing scan metadata keys without updating `scan_metadata.py` will break
  the GUI integration layer.
- Changing viewer overlay JSON formats without updating
  `Matlab_ExtractPowerLine/testSemanticLidarViewer.py` and
  `semantic_overlay_loader.py` will break combined wire + Fusion viewing.
- Changing the `tf_dense_trajectory.csv` schema (column names, column order,
  status string values) without updating `load_dense_trajectory()` and
  `sanitize_dense_trajectory_csv()` will break the dense interpolation path.
- Changing the `DenseTrajectory` diagnostics schema returned by
  `query_dense_trajectory()` without updating the `used_frames` JSON writer
  will break backward compatibility of the Fusion output JSON.

## Tuning Notes

### Fusion

- Increase `--min-vote-to-keep` to suppress one-frame noise.
- Lower `--eps-factor` to split clusters more aggressively.
- Raise `--min-cluster-points` to reject tiny fragments.

### GPS alignment

- Raise `--min-fix-status` to require stronger fixes.
- Lower `--max-horizontal-cov-m2` to reject poor GPS.
- Tighten `--outlier-threshold-m` if the fit is accepting obvious outliers.

## Suggested Code Reading Order

1. `rosbag_preprocessing/launcher/run_pipeline.py`
2. `rosbag_preprocessing/overrides/ws_livox/scripts/run_pose_recovery_camera_gps.sh`
3. `rosbag_preprocessing/overrides/ws_livox/scripts/tf_sample_camera_gps.py`
   — focus on `DenseTrajectorySampler` for the dense trajectory feature
4. `rosbag_preprocessing/overrides/ws_livox/scripts/sanitize_pose_recovery_outputs.py`
   — focus on `sanitize_dense_trajectory_csv()` 
5. `fusion/run_yolo_inference.py`
6. `fusion/yolo_inference_common.py`
7. `fusion/fuse_masks_to_slam.py`
   — focus on `load_dense_trajectory()`, `_enforce_quaternion_sign_continuity()`,
     `query_dense_trajectory()`, and the three-path dispatch in the main loop
8. `fusion/tests/test_dense_trajectory.py`
   — unit tests that document expected behavior of the dense interpolation API
9. `fusion/georeference_from_tf_gps.py`

[Previous: File Contracts](06-file-contracts.md) | [Back to index](README.md) | [Next: Validation And Troubleshooting](08-validation-and-troubleshooting.md)
