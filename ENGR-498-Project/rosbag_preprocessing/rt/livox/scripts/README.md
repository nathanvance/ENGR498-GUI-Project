# ws_livox Scripts README

This document describes the scripts in `~/ws_livox/scripts`, with emphasis on the pose-recovery workflow used to:

- replay a rosbag through FAST-LIO
- recover LiDAR/body poses from `/tf`
- export those poses to CSV
- save the resulting SLAM point cloud as a `.pcd`
- capture LiDAR pose at image timestamps
- capture LiDAR pose at GPS timestamps

The folder now contains two related workflows:

1. The original timestamp-from-CSV workflow
2. The newer event-driven camera/GPS workflow

## Folder Contents

The main files in `~/ws_livox/scripts` are:

- `tf_sample_csv.py`
  - Original sampler
  - Reads timestamps from an input CSV and queries `/tf` at those times
- `run_pose_recovery.sh`
  - Original automation wrapper for `tf_sample_csv.py`
- `tf_sample_camera_gps.py`
  - New event-driven sampler
  - Samples `/tf` whenever an image message or GPS message is seen during rosbag replay
- `run_pose_recovery_camera_gps.sh`
  - New automation wrapper for `tf_sample_camera_gps.py`
- `tf_out.csv`
  - Example/output from the original workflow
- `test_times.csv`
  - Example input for the original workflow
- `run_logs/`
  - Historical logs from prior runs

## Coordinate Frames

Both samplers use the existing FAST-LIO TF tree:

- target frame: `camera_init`
- source frame: `body`

That means the scripts query:

- pose of `body`
- expressed in `camera_init`

In practice, this is the LiDAR/body pose in the FAST-LIO SLAM/world frame. This is the same convention already used by the existing `tf_out.csv` workflow and is the pose representation expected by the downstream fusion pipeline.

## Time Behavior

These scripts are written to support rosbag replay with simulated time.

### During rosbag replay

- `rosbag play --clock` publishes `/clock`
- `/use_sim_time` is enabled
- the samplers use `/clock` and/or TF timestamps to determine when it is valid to query a transform

### During live acquisition

- the actual system timebase should come from real message timestamps
- simulated time is not required
- the new event-driven sampler is designed so that the timestamp used for each sample comes from the incoming message header

In other words:

- `/clock` matters for replay
- message header timestamps matter for the actual recorded events

## Workflow 1: Original CSV-Driven Pose Sampling

### Purpose

The original workflow exists for cases where you already have a list of timestamps and want to sample the LiDAR/body pose from `/tf` at those times.

### Files

- `tf_sample_csv.py`
- `run_pose_recovery.sh`

### Input CSV formats supported by `tf_sample_csv.py`

The original sampler supports:

1. Absolute timestamps using:
   - `secs`
   - `nsecs`
2. Relative timestamps using:
   - `t`
3. Relative timestamps using:
   - `stamp`

### Output CSV format

The output columns are:

```csv
t_in_sec,t_query_sec,x,y,z,qx,qy,qz,qw,status
```

Field meaning:

- `t_in_sec`
  - original input time from the CSV
  - for relative inputs, this is the relative time
- `t_query_sec`
  - final absolute timestamp actually queried in TF
- `x,y,z`
  - translation of `body` in `camera_init`
- `qx,qy,qz,qw`
  - quaternion rotation of `body` in `camera_init`
- `status`
  - `OK` if lookup succeeded
  - otherwise an error/status code such as:
    - `NO_TF_AVAILABLE`
    - `EXTRAPOLATION`
    - `BAG_STOPPED`
    - `TIME_NOT_REACHED:...`

### Original runner behavior

`run_pose_recovery.sh` does the following:

1. Starts `roscore`
2. Enables simulated time
3. Launches FAST-LIO
4. Starts rosbag replay in paused mode
5. Unpauses the bag
6. Runs `tf_sample_csv.py`
7. Waits for the sampler to finish
8. Stops rosbag replay

This workflow is still useful when a separate timestamp list already exists.

## Workflow 2: Event-Driven Camera and GPS Pose Sampling

### Purpose

The new workflow was added for bags that include:

- compressed image messages or raw image messages
- GPS messages, specifically `sensor_msgs/NavSatFix`

Instead of reading timestamps from a prebuilt CSV, the new sampler listens to the actual bag topics and records a LiDAR/body pose sample whenever an event occurs.

This is useful for:

- creating `tf_camera_out.csv` from camera/image events
- creating `tf_gps_out.csv` from GPS fix events
- aligning SLAM outputs with future semantic fusion and mapping pipelines

### Files

- `tf_sample_camera_gps.py`
- `run_pose_recovery_camera_gps.sh`

## `tf_sample_camera_gps.py`

### What it does

The sampler subscribes to:

- `/tf`
- `/clock` when replay is using simulated time
- one image topic
- one GPS topic

It then:

1. observes incoming image messages
2. observes incoming GPS messages
3. reads the message header timestamp
4. waits until replay time/TF has reached that timestamp
5. queries `/tf` for the transform from `camera_init` to `body`
6. writes the result to the correct CSV

### Auto-detected topics

The script can auto-detect:

- an image topic from:
  - `sensor_msgs/CompressedImage`
  - `sensor_msgs/Image`
- a GPS topic from:
  - `sensor_msgs/NavSatFix`

If multiple candidates exist, it scores them heuristically. For example:

- image topics containing `compressed`, `compress`, `image`, or `camera` are preferred
- GPS topics containing `/fix`, `gps`, `gnss`, or `navsat` are preferred

### Manual topic override

You can override auto-detection with:

- `--image-topic`
- `--gps-topic`

This is recommended if:

- the bag has multiple camera topics
- the bag has multiple GPS topics
- the naming convention is unusual

### Output files

The new sampler writes two CSV files:

- `tf_camera_out.csv`
- `tf_gps_out.csv`

### `tf_camera_out.csv` format

```csv
t_in_sec,t_query_sec,x,y,z,qx,qy,qz,qw,status
```

This is intentionally kept aligned with the legacy `tf_out.csv` format.

Field meaning:

- `t_in_sec`
  - relative time since the first image event observed by the sampler
- `t_query_sec`
  - exact message header timestamp used to query TF
- `x,y,z`
  - LiDAR/body translation in `camera_init`
- `qx,qy,qz,qw`
  - LiDAR/body orientation in `camera_init`
- `status`
  - TF lookup result

### `tf_gps_out.csv` format

```csv
t_in_sec,t_query_sec,x,y,z,qx,qy,qz,qw,status,latitude_deg,longitude_deg,altitude_m,fix_status,service,cov_xx_m2,cov_yy_m2,cov_zz_m2,covariance_type
```

Field meaning:

- `t_in_sec`
  - relative time since the first GPS event observed by the sampler
- `t_query_sec`
  - GPS message header timestamp used to query TF
- `x,y,z`
  - LiDAR/body translation in `camera_init`
- `qx,qy,qz,qw`
  - LiDAR/body orientation in `camera_init`
- `status`
  - TF lookup result
- `latitude_deg`
  - GPS latitude from `NavSatFix`
- `longitude_deg`
  - GPS longitude from `NavSatFix`
- `altitude_m`
  - GPS altitude from `NavSatFix`
- `fix_status`
  - `NavSatStatus.status`
- `service`
  - `NavSatStatus.service`
- `cov_xx_m2`
  - East-East variance from `position_covariance[0]`
- `cov_yy_m2`
  - North-North variance from `position_covariance[4]`
- `cov_zz_m2`
  - Up-Up variance from `position_covariance[8]`
- `covariance_type`
  - `NavSatFix.position_covariance_type`

### Why GPS covariance is included

These covariance terms are useful later when georeferencing the SLAM map to the Earth frame because they allow low-quality fixes to be:

- rejected
- down-weighted
- debugged more easily

### Internal timing logic

The script intentionally contains more logic than a minimal subscriber because ROS bag replay timing can be messy. It includes:

- `/clock` monitoring
- TF stamp monitoring
- stall detection
- TF retry logic
- queue-based event processing

This helps avoid failures when:

- TF has not been published yet
- the bag is paused
- the bag stops before a transform is available
- the replay timebase is different from the TF header timebase

### Exit behavior

The sampler exits when:

- ROS shuts down
- replay stalls and no events remain to process
- both image and GPS topics are absent

If only one topic exists, it can still produce the CSV for that one topic.

## `run_pose_recovery_camera_gps.sh`

### What it does

This is the new end-to-end automation script for the event-driven workflow.

It:

1. resolves the bag path
2. creates a unique output directory
3. launches `roscore`
4. enables simulated time
5. launches FAST-LIO
6. starts the new camera/GPS sampler before unpausing replay
7. starts rosbag replay in paused mode
8. automatically unpauses replay after a short delay
9. waits for the sampler to finish
10. stops replay and FAST-LIO
11. copies FAST-LIO's saved `scans.pcd` into the run output folder

### Why the sampler starts before replay unpauses

This is intentional.

If replay starts before the sampler is subscribed, the first camera or GPS event could be missed. Starting the sampler first makes the workflow more reliable for event-driven capture.

### Output directory layout

Each run creates a directory like:

```text
~/ws_livox/pose_recovery_outputs/<bag_stem>_<timestamp_pid>/
```

Inside that directory:

```text
tf_camera_out.csv
tf_gps_out.csv
logs/
pcd/
```

The `pcd/` folder should contain:

```text
pcd/scans.pcd
```

The `logs/` folder contains:

- `fastlio.log`
- `rosbag.log`
- `sampler.log`
- `roscore.log`

### Where the PCD comes from

FAST-LIO already has PCD saving enabled in the current `horizon.yaml` configuration:

- `pcd_save_en: true`
- `interval: -1`

That causes FAST-LIO to write:

```text
~/ws_livox/src/FAST_LIO/PCD/scans.pcd
```

The runner copies that file into the per-run output directory after FAST-LIO exits.

## Command-Line Usage

### New event-driven workflow

Basic usage:

```bash
~/ws_livox/scripts/run_pose_recovery_camera_gps.sh ~/ws_livox/bags/YOUR_BAG.bag
```

With manual topic overrides:

```bash
~/ws_livox/scripts/run_pose_recovery_camera_gps.sh \
  ~/ws_livox/bags/YOUR_BAG.bag \
  --image-topic /image/compressed \
  --gps-topic /fix
```

With an alternate output root:

```bash
~/ws_livox/scripts/run_pose_recovery_camera_gps.sh \
  ~/ws_livox/bags/YOUR_BAG.bag \
  --output-root ~/ws_livox/custom_pose_outputs
```

With topic remaps passed to `rosbag play`:

```bash
~/ws_livox/scripts/run_pose_recovery_camera_gps.sh \
  ~/ws_livox/bags/YOUR_BAG.bag \
  --remap /livox/lidar:=/livox/lidar_points \
  --remap /livox/imu:=/imu/data
```

### Direct sampler usage

You can also run the sampler directly if ROS is already running:

```bash
~/ws_livox/scripts/tf_sample_camera_gps.py \
  --camera-out-csv ~/ws_livox/scripts/tf_camera_out.csv \
  --gps-out-csv ~/ws_livox/scripts/tf_gps_out.csv \
  --image-topic /image/compressed \
  --gps-topic /fix
```

### Legacy workflow

Original runner:

```bash
~/ws_livox/scripts/run_pose_recovery.sh \
  ~/ws_livox/bags/movingtest1.bag \
  ~/ws_livox/scripts/test_times.csv \
  ~/ws_livox/scripts/tf_out.csv
```

Original direct sampler:

```bash
~/ws_livox/scripts/tf_sample_csv.py \
  --in_csv ~/ws_livox/scripts/test_times.csv \
  --out_csv ~/ws_livox/scripts/tf_out.csv
```

## Dependencies

The scripts assume:

- Ubuntu with ROS Noetic
- a built `~/ws_livox` catkin workspace
- FAST-LIO in the workspace
- `expect` installed for automated rosbag unpause

If `expect` is missing:

```bash
sudo apt-get install -y expect
```

## Validation Status

The new scripts have been checked for:

- Python syntax via `py_compile`
- shell syntax via `bash -n`
- direct CLI startup for both new scripts

At the time this README was written, the current example bag in `~/ws_livox/bags` only contained:

- `/livox/imu`
- `/livox/lidar`

So the new event-driven workflow has not yet been fully exercised against a bag that actually contains:

- image messages
- `sensor_msgs/NavSatFix` GPS messages

This means the code path is ready, but the true end-to-end validation still depends on a future bag with those topics.

## Downstream Use

These outputs are meant to support the later fusion and mapping pipeline:

- `tf_camera_out.csv`
  - supports mask/image-to-point-cloud projection workflows
- `tf_gps_out.csv`
  - supports SLAM-to-GPS alignment and future map overlay
- `pcd/scans.pcd`
  - supplies the accumulated FAST-LIO map

The expected downstream mapping flow is:

1. Run FAST-LIO and save the SLAM point cloud
2. Capture image-triggered LiDAR poses
3. Capture GPS-triggered LiDAR poses
4. Use `tf_camera_out.csv` for semantic projection
5. Use `tf_gps_out.csv` plus GPS-to-LiDAR lever arm for georeferencing

## Troubleshooting

### Problem: no rows are written to `tf_camera_out.csv`

Possible causes:

- the bag does not contain an image topic
- the image topic name is different than expected
- the image topic was not auto-detected correctly

What to do:

- inspect the bag with `rosbag info`
- rerun with `--image-topic <exact_topic_name>`

### Problem: no rows are written to `tf_gps_out.csv`

Possible causes:

- the bag does not contain a `sensor_msgs/NavSatFix` topic
- the GPS topic was not auto-detected correctly

What to do:

- inspect the bag with `rosbag info`
- rerun with `--gps-topic <exact_topic_name>`

### Problem: rows exist but `status` is not `OK`

Possible causes:

- TF was not available at that timestamp
- the transform had not yet been published
- replay stopped before the lookup succeeded
- timestamps are inconsistent across systems

Useful places to inspect:

- `logs/sampler.log`
- `logs/fastlio.log`
- `logs/rosbag.log`

### Problem: `pcd/scans.pcd` is missing

Possible causes:

- FAST-LIO terminated early
- FAST-LIO did not finish writing its PCD
- the map save path changed

What to check:

- `~/ws_livox/src/FAST_LIO/PCD/scans.pcd`
- `logs/fastlio.log`
- `src/FAST_LIO/config/horizon.yaml`

### Problem: topic auto-detection chooses the wrong topic

Use manual overrides:

- `--image-topic`
- `--gps-topic`

This is the recommended solution when multiple similar topics exist in the bag.

## Notes

- The new runner clears its own positional arguments before sourcing ROS setup scripts so that `--help` and other CLI flags do not leak into the catkin setup layer.
- The scripts are intended to stay aligned with the existing `camera_init` / `body` TF convention unless the TF tree changes in the future.
- If the TF tree changes later, update both samplers consistently so downstream CSV consumers continue to interpret poses correctly.
