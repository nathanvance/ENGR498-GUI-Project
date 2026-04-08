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

Example workflow:

1. Generate `tf_gps_out.csv` in `..\rosbag_preprocessing\outputs\pose_recovery\<run_name>\`
2. Run `fuse_masks_to_slam.py`
3. Run `georeference_from_tf_gps.py`
4. Open the Leaflet viewer with the generated JSON outputs

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
