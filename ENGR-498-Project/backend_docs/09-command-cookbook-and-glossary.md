# Command Cookbook And Glossary

[Previous: Validation And Troubleshooting](08-validation-and-troubleshooting.md) | [Back to index](README.md)

## Command Cookbook

### Install repo Python dependencies

```powershell
powershell -ExecutionPolicy Bypass -File .\install_repo_python_env.ps1 `
  -Python C:\path\to\python.exe
```

### Build the ROS Docker image

```powershell
wsl bash -lc "cd /mnt/c/path/to/ENGR-498-Project/rosbag_preprocessing && bash scripts/build_image_wsl.sh"
```

### Run pose recovery

```powershell
python .\rosbag_preprocessing\launcher\run_transform_reading_workflow.py `
  .\data\movingtest1.bag `
  --image-topic /camera/image/compressed `
  --gps-topic /fix
```

### Run local inference

```powershell
python .\fusion\run_yolo_inference.py `
  --pose-recovery-run-dir .\rosbag_preprocessing\outputs\pose_recovery\<run_name> `
  --runtime local `
  --local-device 0 `
  --weights ..\Colab\best.pt
```

### Prepare a Colab bundle

```powershell
python .\fusion\run_yolo_inference.py `
  --pose-recovery-run-dir .\rosbag_preprocessing\outputs\pose_recovery\<run_name> `
  --runtime colab `
  --weights ..\Colab\best.pt
```

### Run fusion

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

### Run GPS georeferencing

```powershell
python .\fusion\georeference_from_tf_gps.py `
  --tf-gps-csv .\rosbag_preprocessing\outputs\pose_recovery\<run_name>\tf_gps_out.csv `
  --objects-json .\fusion\outputs\<fusion_run>\fused_objects.json `
  --output-dir .\fusion\outputs\<fusion_run>\gps
```

## Example Directory Trees

### Pose recovery

```text
rosbag_preprocessing/outputs/pose_recovery/<run_name>/
├── image_timestamps.csv
├── tf_camera_out.csv
├── tf_gps_out.csv
├── images/
├── logs/
├── pcd/
└── yolo_inference/
```

### Fusion

```text
fusion/outputs/<fusion_run>/
├── fused_semantic_map.ply
├── fused_semantic_labels.npz
├── fused_objects.json
├── pole_neighbor_distances.json
└── gps/
```

## Parameter Highlights

### Fusion

- `--min-vote-to-keep`
- `--eps-factor`
- `--min-cluster-points`
- `--stat-std-ratio`
- `--min-pole-spacing-m`

### GPS

- `--min-fix-status`
- `--max-horizontal-cov-m2`
- `--outlier-threshold-m`
- `--allow-scale`

## Glossary

| Term | Meaning |
| --- | --- |
| SLAM map | Global point cloud built by FAST-LIO |
| local frame | The internal map frame produced by SLAM |
| ENU | East-North-Up local tangent geographic frame |
| lever arm | The measured spatial offset between the GPS antenna and LiDAR |
| dense-cluster cleanup | Detection-level filtering that keeps only the densest projected point component |
| instance segmentation | Splitting one semantic class into separate physical objects |
| frame match | Nearest-time pairing between an image timestamp and a LiDAR pose |

[Previous: Validation And Troubleshooting](08-validation-and-troubleshooting.md) | [Back to index](README.md)
