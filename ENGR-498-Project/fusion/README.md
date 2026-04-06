# Fusion

This folder contains the mask-to-point-cloud fusion pipeline, optional native
accelerators, GPS georeferencing, and the Leaflet export/viewer hooks.

Windows-side Python setup for this folder is documented at:

- `..\README.md`

and installed from the project root with:

- `install_repo_python_env.ps1`

Main scripts:

- `fuse_masks_to_slam.py`
- `georeference_from_tf_gps.py`
- `export_powerlines_to_leaflet.py`

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

Detailed usage and algorithm notes live in `README.txt`.
