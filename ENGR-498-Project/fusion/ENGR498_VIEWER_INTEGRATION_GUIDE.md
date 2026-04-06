# ENGR498 Viewer Integration Guide

## Goal
Integrate Fusion object results into the ENGR498 GUI without replacing or rewriting the existing powerline processing pipeline.

This guide assumes:
- powerlines remain owned by the ENGR498 workflow
- Fusion remains owned by the Fusion pipeline
- the GUI combines both as overlays for the same scan

## Current State

### Fusion side
Fusion already produces an object-level output in:
- `fused_objects.json`

Each entry already contains enough information for viewer overlays:
- `object_name`
- `class_name`
- `instance_number`
- `confidence_score`
- `num_points`
- `centroid_map_xyz`
- `bbox_aabb_min_xyz`
- `bbox_aabb_max_xyz`
- `class_color_rgb`
- `gps`

Fusion also writes:
- `fused_semantic_map.ply`
- `fused_semantic_labels.npz`

### ENGR498 side
The newer application architecture appears to live in:
- `views/lidar_dashboard.py`
- `views/lidar_dashboard2.py`
- `views/lidar_dashboard_stepbystep.py`

Those dashboards already track per-scan pipeline state and file outputs through scan metadata.

The older rich powerline viewer remains useful as a rendering reference, but it should not be treated as the long-term integration target:
- `Matlab_ExtractPowerLine/testSemanticLidarViewer.py`

## Recommended Architecture

### 1. Treat Fusion as another overlay source
Do not inject Fusion into the powerline processing code.

Instead, add Fusion as a separate per-scan overlay layer:
- base point cloud
- powerline overlays
- fusion object overlays

This keeps responsibilities clean:
- powerline code computes wires, sag, clearance
- fusion code computes poles, crossarms, transformers, pedestals
- viewer combines both

### 2. Extend scan metadata, not viewer hardcoding
The dashboards already use per-scan metadata files.

For each scan, add Fusion outputs under `files`, for example:

```json
{
  "files": {
    "pcd": "processed/slam/cloud.pcd",
    "filtered": "processed/filtered/cloud_filtered.pcd",
    "segmented": "processed/flai/segmented.las",
    "wires": "processed/wires/wires_points.npz",
    "wire_info": "processed/wires/wire_info.json",
    "ground_points": "processed/wires/ground_points.npz",
    "fused": "processed/fusion/fused_objects.json",
    "fused_map": "processed/fusion/fused_semantic_map.ply"
  }
}
```

This keeps the dashboard responsible for file discovery and status, not parsing.

### 3. Add a dedicated Fusion overlay loader
Create a small viewer-side loader module, for example:
- `views/fusion_overlay_loader.py`

Its job should be:
- load `fused_objects.json`
- validate schema
- normalize colors and labels
- return a clean in-memory overlay model

Suggested normalized object model:

```python
{
    "object_name": "pole_01",
    "class_name": "pole",
    "instance_number": 1,
    "confidence_score": 0.68,
    "num_points": 11094,
    "centroid_map_xyz": [19.09, 2.54, -2.35],
    "bbox_aabb_min_xyz": [...],
    "bbox_aabb_max_xyz": [...],
    "class_color_rgb": [220, 70, 70],
    "gps": {"lat": None, "lon": None, "alt": None}
}
```

### 4. Render Fusion overlays as viewer primitives
In the main viewer:
- render centroid markers
- optionally render bounding boxes
- optionally render labels on select or hover

Recommended first-pass rendering:
- one colored marker per object centroid
- one sidebar list grouped by class
- one info panel showing object metadata

Second-pass rendering:
- add bounding box actors
- add class filter toggles
- add confidence threshold sliders

### 5. Keep powerline rendering independent
Do not convert Fusion results into `wires_points.npz`.
Do not convert ENGR498 powerline outputs into Fusion objects.

Instead, keep two parallel loaders:
- `load_powerline_outputs(...)`
- `load_fusion_objects(...)`

Then compose both overlays in the same scene.

## Best Integration Target

The best long-term integration target is the newer `views/` architecture, not the old standalone MATLAB viewer.

Recommended target flow:
1. Dashboard selects a scan.
2. Dashboard resolves scan metadata paths.
3. Dashboard opens a unified scan viewer.
4. Unified scan viewer loads:
   - base point cloud
   - wire overlays if present
   - fusion overlays if present

Suggested new viewer:
- `views/scan_overlay_viewer.py`

This avoids overloading the older MATLAB-specific viewer with new responsibilities.

## Minimal Implementation Plan

### Phase 1
- Keep existing dashboards
- Reuse existing `openViewerRequested`
- When Fusion is the newest complete step, open a viewer that can load `fused_objects.json`
- Display centroid markers and an object info panel

### Phase 2
- Add support for displaying both powerlines and Fusion objects in the same view
- Use the scan metadata file to find both sets of outputs

### Phase 3
- Add GPS-aware map launch from the same scan metadata
- Launch the Leaflet map with:
  - `fused_objects.json`
  - optional powerline overlay JSON

## Coordinate System Contract

This is the most important technical requirement.

Fusion objects and ENGR498 powerlines must be in the same frame before they are rendered together as aligned overlays.

Possible cases:

### Case A: Same local frame
If both use the same local point-cloud frame:
- draw together directly

### Case B: Both georeferenced
If both have GPS:
- draw together in Leaflet/OSM

### Case C: Different local frames
If Fusion uses SLAM-map coordinates and powerlines use another frame:
- do not pretend they align
- show them separately until a transform is available

## Pedestal Support

Fusion should treat pedestals the same way as poles, crossarms, and transformers at the object JSON level.

Viewer implications:
- pedestal gets its own color
- pedestal appears in the Fusion object list
- pedestal can be toggled independently in filters

## Leaflet Strategy

For mapping:
- keep `fused_objects.json` as the object layer
- keep a separate powerline overlay JSON for wire polylines

Leaflet should load both optionally:
- object markers from Fusion
- powerline polylines from the generated powerline overlay

This is cleaner than trying to flatten everything into one geometry type.

## Summary

The clean design is:
- keep ENGR498 powerline processing unchanged
- keep Fusion processing unchanged
- use scan metadata to connect both outputs to the GUI
- add Fusion as a separate overlay layer in the newer `views/` architecture
- only render the two layers together when their coordinate systems are compatible
