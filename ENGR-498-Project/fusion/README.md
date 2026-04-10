# Fusion

This folder contains the mask-to-point-cloud fusion pipeline, optional native
accelerators, GPS georeferencing, and the Leaflet export/viewer hooks.

Windows-side Python setup for this folder is documented at:

- `..\README.md`

and installed from the project root with:

- `install_repo_python_env.ps1`

Recommended safe setup from `ENGR-498-Project/`:

```powershell
python -m venv .venv
powershell -ExecutionPolicy Bypass -File .\install_repo_python_env.ps1 `
  -Python .\.venv\Scripts\python.exe
```

That keeps the Fusion-side dependencies inside the repo-local virtual
environment instead of modifying a global Python installation.

Main scripts:

- `fuse_masks_to_slam.py`
- `georeference_from_tf_gps.py`
- `export_powerlines_to_leaflet.py`
- `run_yolo_inference.py`

Supporting folders:

- `native/`
  Optional C++ accelerators for projection and geospatial transform work.
- `leaflet_viewer/`
  Browser viewer for fused objects and powerline overlays.
- `tests/`
  Unit tests for the dense trajectory interpolation system.

Sample inputs:

- `sample_intrinsics.json`, `sample_extrinsics.json`
- `sample_pose.csv` — sparse event-driven pose CSV schema reference
- `sample_dense_trajectory.csv` — dense trajectory CSV schema reference
- `sample_image_timestamps.csv`

Example workflow:

1. Generate pose-recovery outputs in `..\rosbag_preprocessing\outputs\pose_recovery\<run_name>\`
   (this now includes `tf_dense_trajectory.csv` in addition to `tf_camera_out.csv` and `tf_gps_out.csv`)
2. Run `fuse_masks_to_slam.py` — pass `--dense-traj-csv` when using a time offset
3. Run `georeference_from_tf_gps.py`
4. Open the Leaflet viewer with the generated JSON outputs

## Dense Trajectory and Time-Offset Interpolation

`fuse_masks_to_slam.py` now has two distinct pose-lookup paths for time-offset mode:

**Dense trajectory path (preferred when `--dense-traj-csv` is supplied):**
- Loads `tf_dense_trajectory.csv`, produced by the preprocessing stage.
- Expects `tf_dense_trajectory.csv:timestamp_sec` to be in the same unix/header
  time domain as `image_timestamps.csv:t_query_sec`.
- Enforces quaternion sign continuity across the full trajectory at load time
  so that local SLERP always follows the short arc.
- For each camera frame, computes `t_query = frame.timestamp + time_offset_sec`.
- Binary-searches the dense timestamps to find the bracketing pair
  `[T[left], T[right]]` around `t_query`.
- Computes `alpha = (t_query - T[left]) / (T[right] - T[left])`.
- Interpolates translation with linear lerp, rotation with SLERP.
- Returns `None` (drops the frame) if `t_query` is outside `[T[0], T[-1]]`.

**Sparse fallback path (used when `--dense-traj-csv` is absent or file missing):**
- Same as the old behavior: calls `interpolate_pose_record()` against the
  sparse `--pose-csv` trajectory.
- A warning is printed when falling back.

**No-offset path (when `--time-offset-enabled` is not passed):**
- Unchanged: uses `match_nearest_pose_indices()` against `--pose-csv`.
- The dense trajectory is not loaded at all.

### Key design choices

- **Sign continuity** is enforced globally at load time (`_enforce_quaternion_sign_continuity`),
  not per SLERP call. This ensures SLERP calls always interpolate along the short arc
  even when the underlying quaternion path crosses a hemisphere boundary.
- **Local interpolation only** (no global splines). Bracket size is exactly two
  consecutive 10 ms samples. This prevents overshoot and is robust for uneven motion.
- **Diagnostics schema** returned by `query_dense_trajectory()` uses the same keys
  as the old `interpolate_pose_record()` — `match_mode`, `pose_index_lo/hi`,
  `pose_time_lo/hi`, `interp_alpha` — so the `used_frames` JSON output is
  schema-compatible with old runs.

### Running the unit tests

```powershell
python -m pytest fusion\tests\test_dense_trajectory.py -v
```

Tests cover: sign continuity, SLERP edge cases, `query_dense_trajectory` boundary
conditions, `load_dense_trajectory` filtering/sorting/normalization, and full
load→query round-trips.

The integrated dashboard also calls these scripts:

- `testDashboard.py`
- `gui_pipeline.py`

That GUI path:

1. consumes the JPGs and CSV outputs from `rosbag_preprocessing`,
2. reuses the per-scan wire extraction outputs when present,
3. runs `run_yolo_inference.py`,
4. runs `fuse_masks_to_slam.py`,
5. runs `georeference_from_tf_gps.py`,
6. opens the semantic viewer with wire + Fusion overlays together,
7. opens the Leaflet map when JSON outputs are present.

The GUI now also supports step-by-step execution for:

- Rosbag Preprocessing
- wire extraction
- image inference
- fusion + GPS

Calibration is a separate GUI mode. Fusion relies on that calibration mode
having already produced camera intrinsics and the LiDAR-camera extrinsic.

FLAI remains an external/manual stage.

Detailed usage and algorithm notes live in `README.txt`.
