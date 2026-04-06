# Backend Docs

This folder is the click-through version of the backend documentation. It is
intended for developers who want to navigate the pipeline by topic instead of
reading the single long reference document.

If you want the single-file version, use:

- [../BACKEND_SOFTWARE_DESCRIPTION.md](../BACKEND_SOFTWARE_DESCRIPTION.md)
- [../BACKEND_SOFTWARE_DESCRIPTION.txt](../BACKEND_SOFTWARE_DESCRIPTION.txt)

## Read This First

The backend takes raw rosbags and turns them into portable semantic outputs.
The major flow is:

1. `rosbag_preprocessing/` replays the bag, runs FAST-LIO, exports JPG frames,
   and samples LiDAR poses on image and GPS timestamps.
2. `fusion/run_yolo_inference.py` runs local YOLO segmentation or prepares a
   Colab fallback bundle, then writes `masks_npz/` and `meta_json/`.
3. `fusion/fuse_masks_to_slam.py` projects those masks into the SLAM cloud,
   filters the result, segments object instances, and computes pole distances.
4. `fusion/georeference_from_tf_gps.py` fits a local-to-GPS transform and
   writes GPS coordinates back into the fused outputs.
5. `fusion/export_powerlines_to_leaflet.py` adapts generated wire outputs into
   a portable overlay format.

The most important idea in the backend is that every stage hands off explicit
files to the next stage. Those file contracts are the backbone of the system.

## Navigation

1. [System Overview](01-system-overview.md)
2. [ROS Bag Preprocessing](02-rosbag-preprocessing.md)
3. [Inference](03-inference.md)
4. [Camera-LiDAR Fusion](04-fusion.md)
5. [GPS Georeferencing And Powerlines](05-georeferencing-and-powerlines.md)
6. [File Contracts](06-file-contracts.md)
7. [Developer Guide](07-developer-guide.md)
8. [Validation And Troubleshooting](08-validation-and-troubleshooting.md)
9. [Command Cookbook And Glossary](09-command-cookbook-and-glossary.md)
