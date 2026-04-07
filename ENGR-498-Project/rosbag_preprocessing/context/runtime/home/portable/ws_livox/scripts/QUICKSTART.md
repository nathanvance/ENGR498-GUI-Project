# ws_livox Scripts Quickstart

This is the short version of the pose-recovery workflow in `~/ws_livox/scripts`.

For the full reference, see:

- `~/ws_livox/scripts/README.md`

## Main Scripts

- `tf_sample_csv.py`
  - Old workflow
  - Samples `/tf` from timestamps stored in an input CSV
- `run_pose_recovery.sh`
  - Old automation wrapper
- `tf_sample_camera_gps.py`
  - New workflow
  - Samples `/tf` whenever an image message or GPS message is seen
- `run_pose_recovery_camera_gps.sh`
  - New automation wrapper

## Most Common Workflow

Use this when your bag contains:

- Livox LiDAR
- IMU
- image topic
- GPS `NavSatFix` topic

Run:

```bash
~/ws_livox/scripts/run_pose_recovery_camera_gps.sh ~/ws_livox/bags/YOUR_BAG.bag
```

If topic auto-detection picks the wrong topics, run:

```bash
~/ws_livox/scripts/run_pose_recovery_camera_gps.sh \
  ~/ws_livox/bags/YOUR_BAG.bag \
  --image-topic /image/compressed \
  --gps-topic /fix
```

## Output Location

Each run creates a folder like:

```text
~/ws_livox/pose_recovery_outputs/<bag_name>_<timestamp_pid>/
```

Inside that folder:

```text
tf_camera_out.csv
tf_gps_out.csv
pcd/scans.pcd
logs/
```

## Output CSVs

### `tf_camera_out.csv`

```csv
t_in_sec,t_query_sec,x,y,z,qx,qy,qz,qw,status
```

### `tf_gps_out.csv`

```csv
t_in_sec,t_query_sec,x,y,z,qx,qy,qz,qw,status,latitude_deg,longitude_deg,altitude_m,fix_status,service,cov_xx_m2,cov_yy_m2,cov_zz_m2,covariance_type
```

## Common Overrides

Custom output root:

```bash
~/ws_livox/scripts/run_pose_recovery_camera_gps.sh \
  ~/ws_livox/bags/YOUR_BAG.bag \
  --output-root ~/ws_livox/custom_pose_outputs
```

Rosbag remaps:

```bash
~/ws_livox/scripts/run_pose_recovery_camera_gps.sh \
  ~/ws_livox/bags/YOUR_BAG.bag \
  --remap /livox/lidar:=/livox/lidar_points \
  --remap /livox/imu:=/imu/data
```

## Old CSV-Driven Workflow

If you already have a timestamp CSV:

```bash
~/ws_livox/scripts/run_pose_recovery.sh \
  ~/ws_livox/bags/movingtest1.bag \
  ~/ws_livox/scripts/test_times.csv \
  ~/ws_livox/scripts/tf_out.csv
```

## Direct Sampler Usage

If ROS is already running and you only want the new sampler:

```bash
~/ws_livox/scripts/tf_sample_camera_gps.py \
  --camera-out-csv ~/ws_livox/scripts/tf_camera_out.csv \
  --gps-out-csv ~/ws_livox/scripts/tf_gps_out.csv \
  --image-topic /image/compressed \
  --gps-topic /fix
```

## Prerequisites

- ROS Noetic installed
- `~/ws_livox` built
- FAST-LIO available in the workspace
- `expect` installed

Install `expect` if needed:

```bash
sudo apt-get install -y expect
```

## Troubleshooting

- If `tf_camera_out.csv` is empty:
  - verify the bag actually contains an image topic
  - rerun with `--image-topic`
- If `tf_gps_out.csv` is empty:
  - verify the bag contains a `sensor_msgs/NavSatFix` topic
  - rerun with `--gps-topic`
- If `pcd/scans.pcd` is missing:
  - check `logs/fastlio.log`
  - check that FAST-LIO finished cleanly
- If statuses are not `OK`:
  - inspect `logs/sampler.log`
  - inspect replay timing and TF availability
