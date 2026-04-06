# Validation And Troubleshooting

[Previous: Developer Guide](07-developer-guide.md) | [Back to index](README.md) | [Next: Command Cookbook And Glossary](09-command-cookbook-and-glossary.md)

## Validation Strategy

Validate the pipeline stage-by-stage instead of only at the end.

### Pose recovery validation

Check:

- `images/` exists and contains frames
- `image_timestamps.csv` row count matches JPG count
- `tf_camera_out.csv` has valid `OK` rows
- `tf_gps_out.csv` has valid `OK` rows when GPS is present
- `pcd/scans.pcd` exists and is non-empty

### Inference validation

Check:

- every image stem has:
  - one `*_masks.npz`
  - one `*_meta.json`
- `meta_json` class names are what Fusion expects
- masks are non-empty for expected objects

### Fusion validation

Check:

- `fused_objects.json` contains expected classes
- `fused_semantic_map.ply` is non-empty
- `fused_semantic_labels.npz` contains the expected arrays
- `pole_neighbor_distances.json` contains plausible pole spacing

### GPS validation

Check:

- `gps_alignment.json` has a reasonable inlier count
- `rmse_xy_m` and `rmse_z_m` are plausible
- georeferenced object positions are not obviously flipped or rotated

## Common Failure Modes

| Symptom | Likely cause | First place to inspect |
| --- | --- | --- |
| Docker workflow exits immediately | path conversion or container startup issue | `launcher/run_pipeline.py`, `compose.yaml` |
| No `tf_camera_out.csv` rows | image topic not detected or replay stalled | `run_pose_recovery_camera_gps.sh`, `tf_sample_camera_gps.py` |
| No `tf_gps_out.csv` rows | GPS topic not detected or bag has no `NavSatFix` messages | `run_pose_recovery_camera_gps.sh`, `tf_sample_camera_gps.py` |
| Inference outputs incomplete | local runtime failure or incomplete Colab export | `run_yolo_inference.py`, `import_colab_inference_results.py` |
| Fusion says “no allowed detections” | class-name mismatch | `fuse_masks_to_slam.py`, inference metadata |
| Fusion says “no map points landed inside allowed masks” | calibration/timing/image-orientation mismatch | calibration files, timestamps, masks |
| Objects are over-segmented | clustering thresholds too strict | `segment_class_instances()`, merge helpers |
| GPS alignment looks wrong | bad fixes, wrong lever arm, or frame mismatch | `georeference_from_tf_gps.py`, `gps_alignment.json` |

## Stage-Specific Troubleshooting

### Docker / ROS preprocessing

1. Inspect `logs/roscore.log`.
2. Inspect `logs/rosbag.log`.
3. Inspect `logs/fastlio.log`.
4. Inspect `logs/sampler.log`.
5. Retry with explicit topic overrides if auto-detection failed.

### Inference

1. Confirm the weights file exists.
2. Confirm the model is a segmentation model.
3. Confirm CUDA is visible for local runs.
4. Confirm every JPG stem is represented in the output.

### Fusion

1. Open one `*_meta.json`.
2. Confirm the class names and instance indices look correct.
3. Confirm the pose CSV time column is the intended one.
4. Confirm masks were not rotated relative to the input images.

### GPS alignment

1. Open `gps_alignment.json`.
2. Check:
   - `num_samples_total`
   - `num_samples_inliers`
   - `rmse_xy_m`
   - `rmse_z_m`
3. If inlier count is low, suspect:
   - bad GPS
   - wrong lever arm
   - wrong frame assumption

[Previous: Developer Guide](07-developer-guide.md) | [Back to index](README.md) | [Next: Command Cookbook And Glossary](09-command-cookbook-and-glossary.md)
