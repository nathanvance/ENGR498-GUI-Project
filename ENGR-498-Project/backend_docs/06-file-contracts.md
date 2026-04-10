# File Contracts

[Previous: GPS Georeferencing And Powerlines](05-georeferencing-and-powerlines.md) | [Back to index](README.md) | [Next: Developer Guide](07-developer-guide.md)

## Why File Contracts Matter

The backend is built around stage-to-stage file handoffs. If one stage changes
its output shape or naming without the next stage being updated, the pipeline
breaks. This page captures the main contracts developers need to protect.

## Schema Tables

### `image_timestamps.csv`

| Column | Type | Meaning |
| --- | --- | --- |
| `filename` | string | Exported JPG filename |
| `t_query_sec` | float | Absolute timestamp used for matching |
| `t_in_sec` | float | Relative time since first image event |

Example:

```csv
filename,t_query_sec,t_in_sec
frame_000001.jpg,1771469608.123456,0.000000
frame_000002.jpg,1771469608.456789,0.333333
```

### `tf_camera_out.csv`

| Column | Type | Meaning |
| --- | --- | --- |
| `t_in_sec` | float | Relative time since first image event |
| `t_query_sec` | float | Absolute image timestamp |
| `x`,`y`,`z` | float | Local-frame translation |
| `qx`,`qy`,`qz`,`qw` | float | Local-frame orientation quaternion |
| `status` | string | Sampling status |

### `tf_dense_trajectory.csv`

Produced by `DenseTrajectorySampler` in `tf_sample_camera_gps.py`. One row per
fixed-interval sample (default 10 ms) over the full bag duration. Used by
`fuse_masks_to_slam.py` when time-offset mode is enabled.

| Column | Type | Meaning |
| --- | --- | --- |
| `timestamp_sec` | float | Absolute unix/header/query-domain timestamp aligned with `image_timestamps.csv:t_query_sec` and `tf_camera_out.csv:t_query_sec` |
| `x`,`y`,`z` | float | Local-frame translation (`camera_init <- body`) |
| `qx`,`qy`,`qz`,`qw` | float | Local-frame orientation quaternion (XYZW order) |
| `status` | string | `OK` for a valid lookup; `NO_TF`, or `FAIL:<reason>` otherwise |

Only `OK` rows with a quaternion `norm_sq` in `[0.5, 2.0]` are accepted by
`load_dense_trajectory()`. The sanitizer (`sanitize_pose_recovery_outputs.py`)
drops non-OK and invalid rows, sorts by `timestamp_sec`, and deduplicates before
the file is consumed by Fusion.

Example:

```csv
timestamp_sec,x,y,z,qx,qy,qz,qw,status
1771469600.000,0.0,0.0,0.0,0.0,0.0,0.0,1.0,OK
1771469600.010,0.001,0.0,0.0,0.0,0.0,0.0,1.0,OK
```

**Note:** `timestamp_sec` is the primary column in this file (not `t_query_sec`)
to distinguish it from the event-driven CSV files.

### `tf_gps_out.csv`

| Column | Type | Meaning |
| --- | --- | --- |
| pose columns | floats | LiDAR pose at the GPS event time |
| `latitude_deg` | float | GPS latitude |
| `longitude_deg` | float | GPS longitude |
| `altitude_m` | float | GPS altitude |
| `fix_status` | int | Fix quality |
| `service` | int | GPS service flags |
| `cov_xx_m2`,`cov_yy_m2`,`cov_zz_m2` | float | Covariance diagonal |
| `covariance_type` | int | Covariance interpretation enum |

### `*_masks.npz`

| Key | Type | Meaning |
| --- | --- | --- |
| `masks` | `uint8[N,H,W]` | Binary instance masks |
| `cls` | `int32[N]` | Class IDs |
| `conf` | `float32[N]` | Confidence values |

### `*_meta.json`

Minimal example:

```json
{
  "source_image": "frame_000001.jpg",
  "orig_shape": [1080, 1920],
  "names": {
    "0": "pole",
    "1": "transformer"
  },
  "detections": [
    {
      "instance_index": 0,
      "class_id": 0,
      "class_name": "pole",
      "confidence": 0.91,
      "box_xyxy": [100.0, 120.0, 180.0, 540.0]
    }
  ]
}
```

### `fused_objects.json`

Minimal object example:

```json
{
  "object_name": "pole_01",
  "class_name": "pole",
  "instance_number": 1,
  "confidence_score": 0.82,
  "num_points": 1483,
  "centroid_map_xyz": [1.25, 3.18, 0.74],
  "bbox_aabb_min_xyz": [1.10, 3.01, -0.10],
  "bbox_aabb_max_xyz": [1.39, 3.33, 4.91],
  "class_color_rgb": [220, 70, 70],
  "gps": {"lat": null, "lon": null, "alt": null}
}
```

### `gps_alignment.json`

Minimal example:

```json
{
  "pipeline": "georeference_from_tf_gps",
  "model": "weighted_se2_plus_z_offset",
  "gps_to_lidar_offset_body_m": [0.0, 0.0, 0.0],
  "transform": {
    "scale": 1.0,
    "yaw_deg": 12.4,
    "yaw_rad": 0.216,
    "translation_enu_m": [5.2, -1.7, 0.4]
  },
  "fit_quality": {
    "num_samples_total": 84,
    "num_samples_inliers": 71,
    "rmse_xy_m": 1.21,
    "rmse_z_m": 0.53
  }
}
```

## High-Risk Coupling Points

- Image stems must match across JPGs, masks, and metadata JSON.
- Masks must stay aligned with the original image orientation.
- `tf_camera_out.csv` and `image_timestamps.csv` must use the same time basis.
- `tf_dense_trajectory.csv` must be from the same preprocessing run as
  `tf_camera_out.csv`. Mixing runs produces silently misaligned poses.
- `tf_dense_trajectory.csv` must be sanitized before use. The preprocessing
  shell script calls the sanitizer automatically; manually-produced files must
  be sanitized with `sanitize_pose_recovery_outputs.py --dense-traj-csv`.
- The object JSON GPS fields are placeholders until georeferencing writes them.

[Previous: GPS Georeferencing And Powerlines](05-georeferencing-and-powerlines.md) | [Back to index](README.md) | [Next: Developer Guide](07-developer-guide.md)
