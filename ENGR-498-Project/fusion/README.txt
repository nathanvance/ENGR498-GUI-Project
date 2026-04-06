Fusion README
=============

Overview
--------
The Fusion pipeline projects YOLO segmentation masks into a SLAM point cloud
using camera intrinsics, LiDAR-to-camera extrinsics, and LiDAR poses. The main
entrypoint is:

  .\fuse_masks_to_slam.py

The script:
1. Loads a map/SLAM point cloud.
2. Loads camera intrinsics from JSON.
3. Loads LiDAR-to-camera extrinsics from JSON.
4. Loads LiDAR poses from a CSV file.
5. Matches image timestamps to poses.
6. Projects map points into each image.
7. Applies YOLO masks to label 3D points.
8. Keeps dense projected clusters.
9. Fuses labels across frames with a minimum vote threshold.
10. Segments object instances such as pole_01 and transformer_01.
11. Writes labeled outputs and opens an Open3D viewer unless disabled.


Environment
-----------
Run the Fusion scripts from the `fusion` folder with any Python environment
that has the required packages installed. If you want to force a specific
interpreter for the Leaflet helper, set:

  FUSION_PYTHON

The pipeline depends on packages already available in that environment,
including:
- numpy
- scipy
- open3d
- opencv-python

To recreate the repo's Windows-side Python environment in a venv, run from the
`ENGR-498-Project` folder:

  powershell -ExecutionPolicy Bypass -File .\install_repo_python_env.ps1 `
    -Python C:\path\to\python.exe

The actual virtual environment directory is intentionally not committed to the
repo. Portability is handled through:
- `..\requirements-repo-python.txt`
- `..\requirements-repo-python-lock.txt`
- optional MATLAB bridge files:
  - `..\requirements-repo-python-matlab.txt`
  - `..\requirements-repo-python-matlab-lock.txt`

On first run, the script builds a native C++ DLL for fast point projection and
mask lookup. Visual Studio Build Tools must be installed.


Files in This Folder
--------------------
- fuse_masks_to_slam.py
  Main fusion pipeline.

- run_yolo_inference.py
  Runs local YOLO segmentation directly on the JPG output produced by
  rosbag_preprocessing, or prepares a Colab fallback bundle.

- import_colab_inference_results.py
  Imports Colab-generated masks and metadata back into a local Fusion run
  folder.

- yolo_inference_common.py
  Shared helpers for exporting Fusion-ready masks and metadata.

- colab\run_inference_colab.py
  Colab-side inference entrypoint for prepared `colab_bundle` folders.

- native\
  Native C++ acceleration code and build script.

- calibration_test\
  Contains the current tested calibration and fusion outputs.

- sample_intrinsics.json
- sample_extrinsics.json
- sample_pose.csv
- sample_image_timestamps.csv
  Small example inputs used during development.


Portable Path Convention
------------------------
The Docker pose-recovery workflow now writes portable outputs into the mounted
container project folder:

  ..\rosbag_preprocessing\outputs\pose_recovery\<run_name>\

Typical files from that stage are:
- image_timestamps.csv
- images\frame_000001.jpg
- tf_camera_out.csv
- tf_gps_out.csv
- pcd\scans.pcd

The Fusion georeferencing step is designed to consume `tf_gps_out.csv` from
that folder directly.


Expected Inputs
---------------
1. intrinsics.json
   Required fields:
   - fx
   - fy
   - cx
   - cy
   - width
   - height
   - distortion

2. extrinsics.json
   Expected format:
   - T_lidar_cam as a 4x4 transform matrix

3. pose CSV
   The tested schema is:
   - t_in_sec
   - t_query_sec
   - x
   - y
   - z
   - qx
   - qy
   - qz
   - qw
   - status

4. image timestamp CSV
   Required columns:
   - filename
   - t_query_sec

   The rosbag preprocessing workflow now produces this file directly as:
   - ..\rosbag_preprocessing\outputs\pose_recovery\<run_name>\image_timestamps.csv

   and stores the corresponding JPG frames in:
   - ..\rosbag_preprocessing\outputs\pose_recovery\<run_name>\images\

5. point cloud
   Tested with:
   - .pcd

6. YOLO outputs
   - masks_npz directory with *_masks.npz
   - meta_json directory with *_meta.json

   These can now be produced either:
   - locally by `run_yolo_inference.py`
   - or in Colab by `colab\run_inference_colab.py`

7. GPS / TF georeferencing CSV
   Produced by the WSL event-driven sampler as:
   - tf_gps_out.csv

   Expected columns:
   - t_in_sec
   - t_query_sec
   - x
   - y
   - z
   - qx
   - qy
   - qz
   - qw
   - status
   - latitude_deg
   - longitude_deg
   - altitude_m
   - fix_status
   - service
   - cov_xx_m2
   - cov_yy_m2
   - cov_zz_m2
   - covariance_type


Current Behavior
----------------
- Uses nearest pose in time.
- Uses t_query_sec by default for pose/image matching.
- Default minimum vote threshold is 1.
- Keeps dense projected clusters.
- Segments object instances after fusion.
- Pedestal support is enabled by default.
- Uses pole-specific logic so one physical pole is less likely to split into
  multiple instances.
- Uses transformer-specific support-aware merging so fragments on the same pole
  can merge into one transformer instance.
- Colors each class differently in the output point cloud.
- Exports one combined JSON with all detected objects.
- GPS fields start as placeholders and are later filled by the GPS
  georeferencing stage.


YOLO Inference Workflow
-----------------------
The rosbag preprocessing stage now writes camera frames as JPG files in:

  ..\rosbag_preprocessing\outputs\pose_recovery\<run_name>\images\

with matching timestamps in:

  ..\rosbag_preprocessing\outputs\pose_recovery\<run_name>\image_timestamps.csv

Those JPGs are the direct inputs to the YOLO inference scripts in this folder.

Main entrypoint:

  .\run_yolo_inference.py

Supported runtime modes:
- `auto`
  Use a modern local NVIDIA CUDA GPU when available. If a suitable local GPU
  or local runtime is not available, prepare a Colab fallback bundle instead.
- `local`
  Force the local GPU path. This is the explicit option for users who want to
  use the local GPU instead of auto-detect.
- `colab`
  Skip local inference and prepare a Colab bundle immediately.

Local runtime notes:
- local inference expects a YOLO segmentation weights file such as `best.pt`
- use the repo-level installer `..\install_repo_python_env.ps1` to recreate
  the Windows-side Python environment
- local auto-detect treats CUDA devices with compute capability 7.0 or newer
  as "modern"
- `--local-device` lets you explicitly choose a local GPU such as `0` or
  `cuda:0`

Colab runtime notes:
- the bundled Colab entrypoint is `colab\run_inference_colab.py`
- it prefers an `A100` by policy and reports whether Colab actually assigned
  one
- Colab GPU type cannot be forced purely from notebook code; if an A100 is not
  assigned, the script uses the available CUDA GPU instead

Recommended local command:

  python .\run_yolo_inference.py `
    --pose-recovery-run-dir ..\rosbag_preprocessing\outputs\pose_recovery\<run_name> `
    --runtime auto `
    --weights ..\..\Colab\best.pt

To force local GPU usage:

  python .\run_yolo_inference.py `
    --pose-recovery-run-dir ..\rosbag_preprocessing\outputs\pose_recovery\<run_name> `
    --runtime local `
    --local-device 0 `
    --weights ..\..\Colab\best.pt

If the script falls back to Colab, it prepares:

  <pose_run>\yolo_inference\colab_bundle\

That bundle contains:
- `images\`
- `image_timestamps.csv`
- `colab_inference_config.json`
- `run_inference_colab.py`
- `yolo_inference_common.py`
- optionally the YOLO weights file if it was available locally

It also writes:

  <pose_run>\yolo_inference\colab_bundle.zip

To run the bundle in Colab:

  %run /content/drive/MyDrive/.../colab_bundle/run_inference_colab.py `
    --run-root /content/drive/MyDrive/.../colab_bundle

After Colab finishes, bring the results back locally with:

  python .\import_colab_inference_results.py `
    --pose-recovery-run-dir ..\rosbag_preprocessing\outputs\pose_recovery\<run_name> `
    --colab-run-root <downloaded_or_synced_colab_bundle>

The resulting local folders are:
- `pred_images\`
- `masks_npz\`
- `meta_json\`
- `combined_class_masks\`

These outputs can then be passed directly to `fuse_masks_to_slam.py`.


Outputs
-------
The pipeline writes:

- fused_semantic_map.ply
  Point cloud with semantic colors.

- fused_semantic_labels.npz
  NumPy archive with point-wise labels, class indices, instance indices,
  confidence scores, and votes.

- fused_objects.json
  Combined object-level output containing:
  - object_name
  - class_name
  - instance_number
  - confidence_score
  - num_points
  - centroid_map_xyz
  - bbox_aabb_min_xyz
  - bbox_aabb_max_xyz
  - class_color_rgb
  - gps placeholder fields

- pole_neighbor_distances.json
  Pole-to-pole spacing output used for line overlays and spacing labels.

- eng498_powerlines_overlay.json
  Optional Leaflet-ready powerline overlay JSON created from generated
  ENGR498 wire outputs.

- gps_alignment.json
  Transform report written by the GPS georeferencing stage. This stores the
  fitted local-to-ENU model, reference latitude/longitude/altitude, and fit
  quality metrics.

- tf_gps_georeferenced.csv
  Per-sample diagnostic CSV written by the GPS georeferencing stage. This
  includes predicted GPS coordinates, ENU coordinates, residuals, and inlier
  flags for every GPS/TF sample used in the fit.

- fused_objects_georeferenced.json
  Optional georeferenced copy of fused_objects.json. Every object in the
  `objects` array receives a real `gps` field.

- eng498_powerlines_overlay_georeferenced.json
  Optional georeferenced copy of the Leaflet powerline overlay. Each powerline
  receives a georeferenced centroid and a `polyline_gps` array.


Tested Calibration Files
------------------------
The latest tested intrinsic calibration artifacts are in:

  .\calibration_test

Important files there:
- whiteout_detected_camera_intrinsics.json
- current_extrinsics.json
- tf_out_from_wsl.csv
- ordered_image_timestamps_from_tf_out.csv


Run Command
-----------
This is the current tested command:

  python .\fuse_masks_to_slam.py `
    --intrinsics-json .\calibration_test\whiteout_detected_camera_intrinsics.json `
    --extrinsics-json .\calibration_test\current_extrinsics.json `
    --pose-csv .\calibration_test\tf_out_from_wsl.csv `
    --image-timestamps-csv .\calibration_test\ordered_image_timestamps_from_tf_out.csv `
    --point-cloud ..\Calibration\Calibration 2_12_26 (iPhone)\Moving\pc\scans.pcd `
    --mask-dir ..\Calibration\Calibration 2_12_26 (iPhone)\Moving\Test_Images_2_12_26_predict-moving\Test_Images_2_12_26_predict-moving\masks_npz `
    --meta-dir ..\Calibration\Calibration 2_12_26 (iPhone)\Moving\Test_Images_2_12_26_predict-moving\Test_Images_2_12_26_predict-moving\meta_json `
    --output-dir .\calibration_test

To run without the Open3D viewer, add:

  --no-visualize


Leaflet Viewer
--------------
A Leaflet viewer is included here:

  .\leaflet_viewer\index.html

It can load `fused_objects.json` as input when you pass a `data=` query
parameter or an optional powerline overlay when you pass `powerlines=`.

Current behavior:
- If GPS is present in the JSON, the app uses OpenStreetMap tiles.
- If GPS is missing, the app falls back to local XY coordinates using
  `centroid_map_xyz`.
- It can also load an optional powerline overlay JSON and draw generated
  powerlines as polylines.

To launch a local web server for the viewer, run:

  powershell -ExecutionPolicy Bypass -File ".\launch_leaflet_viewer.ps1"

Then open:

  http://localhost:8765/leaflet_viewer/index.html

Example with Fusion objects:

  http://localhost:8765/leaflet_viewer/index.html?data=../outputs/fused_objects.json

Example with both Fusion objects and the generated ENGR498 powerline overlay:

  http://localhost:8765/leaflet_viewer/index.html?data=../outputs/fused_objects.json&powerlines=../outputs/eng498_powerlines_overlay.json

Example with only the generated ENGR498 powerline overlay:

  http://localhost:8765/leaflet_viewer/index.html?powerlines=../outputs/eng498_powerlines_overlay.json

You can point the viewer at a different JSON file by changing the `data=`
query parameter. The optional `powerlines=` parameter can point to a
powerline overlay JSON generated by the hook script below.

Important:
- Fusion demo outputs and ENGR498 demo powerline outputs may not share the same
  coordinate frame.
- Only load both layers together when they are in the same local frame or both
  have been georeferenced.


ENGR498 Powerline Hook
----------------------
To display generated ENGR498 powerline outputs in the Leaflet viewer, run:

  python .\export_powerlines_to_leaflet.py `
    --wires-npz '.\path\to\wires_points.npz' `
    --wire-info-json '.\path\to\wire_info.json' `
    --ground-points-npz '.\path\to\ground_points.npz' `
    --output-json '.\path\to\eng498_powerlines_overlay.json'

The hook:
- reads generated wire point outputs
- tolerates older MATLAB-style or newer `wire_1`, `wire_2`, ... NPZ layouts
- creates ordered powerline polylines for Leaflet
- preserves fit metadata when available
- writes GPS placeholder fields for future georeferencing

An example output has been generated here:

  .\calibration_test\eng498_powerlines_overlay.json


ENGR498 Viewer Guide
--------------------
A design guide for integrating Fusion results into the newer ENGR498
`views/` architecture is here:

  .\ENGR498_VIEWER_INTEGRATION_GUIDE.md


GPS Georeferencing
------------------
The GPS georeferencing script is:

  .\georeference_from_tf_gps.py

It:
- reads `tf_gps_out.csv`
- fits a weighted local-to-GPS transform
- estimates yaw, XY translation, and a separate Z offset
- optionally estimates a uniform XY scale when `--allow-scale` is used
- supports an optional GPS-to-LiDAR lever arm
- can georeference:
  - `fused_objects.json`
  - `eng498_powerlines_overlay.json`
- writes:
  - `gps_alignment.json`
  - `tf_gps_georeferenced.csv`

Optional native acceleration is available in:

  .\native\geospatial_accel.cpp

What Gets Georeferenced
-----------------------
The script does not care about object class names. It georeferences every
object already present in `fused_objects.json` by transforming
`centroid_map_xyz` into a `gps` dictionary. That means it automatically applies
to:
- poles
- crossarms
- transformers
- pedestals
- any future object class added to the Fusion JSON

For powerlines, it georeferences:
- `centroid_map_xyz` to `gps`
- every vertex in `polyline_map_xyz` to `polyline_gps`

If an object or powerline is already present in the input JSON, the
georeferencing script will write GPS coordinates for it.

Algorithm Breakdown
-------------------
The fitted model reported in `gps_alignment.json` is:
- `weighted_se2_plus_z_offset` by default
- `weighted_similarity2d_plus_z_offset` when `--allow-scale` is enabled

In practical terms, the script assumes the FAST-LIO map is already level with
gravity, so it solves:
- yaw rotation in the horizontal plane
- XY translation in a local ENU frame
- Z offset
- optional uniform XY scale

It does not solve a free 3D rotation. That is intentional because FAST-LIO's
local frame is expected to already be gravity-aligned.

Step by step:
1. Load `tf_gps_out.csv` and keep rows whose TF lookup `status` is `OK`.
2. Parse the LiDAR pose sampled at each GPS message:
   - `x, y, z`
   - `qx, qy, qz, qw`
3. Parse the GPS fix from the same row:
   - `latitude_deg`
   - `longitude_deg`
   - `altitude_m`
   - `fix_status`
   - horizontal covariance from `cov_xx_m2` and `cov_yy_m2`
4. Reject poor GPS samples:
   - rows below `--min-fix-status`
   - rows whose average horizontal covariance exceeds
     `--max-horizontal-cov-m2`
5. Convert the measured GPS-to-LiDAR lever arm from body coordinates into the
   map frame using the sampled quaternion, then recover the GPS antenna
   position in local map coordinates. This is the local point set that gets
   aligned to GNSS.
6. Choose the first retained GNSS fix as the geodetic reference, convert every
   latitude/longitude/altitude sample to WGS84 ECEF coordinates, then convert
   those ECEF coordinates into a local ENU frame. This follows the standard
   WGS84 geodetic -> ECEF -> ENU construction described in [1] and [4].
7. Build a weight for each sample. The current implementation uses the inverse
   of the average horizontal variance so lower-covariance GPS fixes contribute
   more strongly to the fit. The covariance interpretation follows ROS
   `NavSatFix` ENU semantics in [1].
8. Solve a weighted 2D Procrustes / Kabsch-Umeyama alignment between:
   - local GPS antenna XY positions from FAST-LIO
   - GNSS ENU XY positions
   This yields yaw rotation, XY translation, and optional XY scale. The
   underlying closed-form SVD solve is the same family of method described in
   [5].
9. Run robust outlier rejection for up to five iterations:
   - transform every local point with the current model
   - compute XY residuals
   - mark inliers using a threshold based on the larger of:
     - `--outlier-threshold-m`
     - median residual + 3 * MAD
   - refit on the surviving inliers
10. Estimate the vertical offset separately as the weighted mean difference
    between the aligned local Z values and the ENU Z values of the inliers.
11. Write `gps_alignment.json` and `tf_gps_georeferenced.csv`, including:
    - yaw
    - translation
    - scale
    - inlier count
    - XY and Z RMSE
12. If `--objects-json` is provided, transform every `centroid_map_xyz` into
    geodetic coordinates and write the result to each object's `gps` field.
13. If `--powerlines-json` is provided, transform:
    - every `centroid_map_xyz` into `gps`
    - every `polyline_map_xyz` vertex into `polyline_gps`

Why This Model Was Chosen
-------------------------
- It matches the data we actually collect: GPS fixes plus the LiDAR pose in
  FAST-LIO's local frame at the same timestamps.
- It is fast and stable because the rotation solve is closed-form and only
  operates in the horizontal plane.
- It matches utility mapping needs well because the dominant unknown is usually
  yaw plus horizontal translation, not arbitrary roll/pitch.
- It works naturally with the ENU covariance convention published by ROS
  `NavSatFix`.

Data Packaging and ROS Assumptions
----------------------------------
`tf_gps_out.csv` is expected to come from the event-driven WSL sampler created
for this project. That sampler records the LiDAR pose whenever a GPS message is
seen, so every row already contains:
- the LiDAR pose in the FAST-LIO local frame
- the GPS fix from the same event

The CSV format is designed around the ROS `sensor_msgs/NavSatFix` message and
the outputs published by `nmea_navsat_driver` [1][2][3].

Important conventions:
- latitude / longitude / altitude are WGS84 geodetic coordinates [1]
- covariance is in an ENU tangent plane and reported in square meters
- `cov_xx_m2`, `cov_yy_m2`, and `cov_zz_m2` map to the ENU diagonal entries of
  `NavSatFix.position_covariance` [1]

Example command:

  python .\georeference_from_tf_gps.py `
    --tf-gps-csv ..\rosbag_preprocessing\outputs\pose_recovery\<run_name>\tf_gps_out.csv `
    --objects-json .\calibration_test\fused_objects.json `
    --powerlines-json .\calibration_test\eng498_powerlines_overlay.json `
    --output-dir .\calibration_test\georeferenced_outputs `
    --gps-to-lidar-offset-body '0,0,0'

Sources
-------
[1] ROS `sensor_msgs/NavSatFix` message definition:
    https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html

[2] ROS `nmea_navsat_driver` package overview:
    https://index.ros.org/p/nmea_navsat_driver/

[3] ROS `nmea_navsat_driver` API reference for `RosNMEADriver` publishers:
    https://docs.ros.org/en/melodic/api/nmea_navsat_driver/html/classlibnmea__navsat__driver_1_1driver_1_1RosNMEADriver.html

[4] ESA Navipedia reference for ECEF <-> ENU transforms:
    https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates

[5] NIST discussion of the Kabsch-Umeyama / orthogonal Procrustes solution:
    https://www.nist.gov/publications/purely-algebraic-justification-kabsch-umeyama-algorithm


Intrinsic Calibration Script
----------------------------
The intrinsic calibration script used for testing is:

  ..\..\Calibration\Basler Image Processing Scripts\Intrinsic_Calibrate.py

It was updated so its image folder and output paths can be passed with
environment variables:
- INTRINSIC_CAL_IMAGE_DIR
- INTRINSIC_CAL_OUTPUT_MAT
- INTRINSIC_CAL_OUTPUT_CSV
- INTRINSIC_CAL_PATTERN_COLS
- INTRINSIC_CAL_PATTERN_ROWS
- INTRINSIC_CAL_SQUARE_SIZE
- INTRINSIC_CAL_WORLD_UNITS


Known Notes
-----------
- The current test extrinsics file is copied from the existing sample
  extrinsics. A fresh non-MATLAB extrinsic calibration was not generated in
  this folder during the latest test.

- If the Open3D window does not open or closes unexpectedly, the processing may
  still complete and write the output files.

- If the scene changes, clustering tolerances may need to be retuned for new
  object spacing or density.
