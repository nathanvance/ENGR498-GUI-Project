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
| Change image/GPS TF sampling | `rosbag_preprocessing/overrides/ws_livox/scripts/tf_sample_camera_gps.py` |
| Change pose recovery orchestration | `rosbag_preprocessing/overrides/ws_livox/scripts/run_pose_recovery_camera_gps.sh` |
| Change local-vs-Colab inference logic | `fusion/run_yolo_inference.py` |
| Change inference bundle/export helpers | `fusion/yolo_inference_common.py` |
| Change Colab runtime behavior | `fusion/colab/run_inference_colab.py` |
| Change fusion class filtering | `fusion/fuse_masks_to_slam.py` |
| Change clustering / merge behavior | `fusion/fuse_masks_to_slam.py` |
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
4. `fusion/run_yolo_inference.py`
5. `fusion/yolo_inference_common.py`
6. `fusion/fuse_masks_to_slam.py`
7. `fusion/georeference_from_tf_gps.py`

[Previous: File Contracts](06-file-contracts.md) | [Back to index](README.md) | [Next: Validation And Troubleshooting](08-validation-and-troubleshooting.md)
