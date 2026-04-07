# ENGR498 GUI Project

This repository contains the integrated Senior Design software stack for:

- Windows-side GUI and visualization tools
- Fusion of camera segmentation masks into LiDAR / SLAM point clouds
- Docker / WSL rosbag preprocessing for calibration, FAST-LIO, and TF sampling
- MATLAB-side powerline processing assets and viewers

The experimental GUI/backend integration branch also wires those subsystems
together so the Windows dashboard can:

- launch the rosbag preprocessing stack,
- launch wire extraction into the per-scan processed folder,
- run local YOLO inference or surface the Colab fallback requirement,
- run camera-to-LiDAR fusion and GPS georeferencing,
- open a combined semantic viewer with wire and fusion overlays together,
- open the Leaflet map directly from the GUI when JSON outputs are available.

The main project folder is:

```text
ENGR-498-Project/
```

Start with:

- [ENGR-498-Project/README.md](ENGR-498-Project/README.md)

That document explains:

- host requirements
- Windows Python environment setup
- Docker / WSL setup for rosbag preprocessing
- local YOLO inference and Colab fallback
- the integrated GUI entrypoints and scan metadata flow
- where each subsystem lives in the repo
