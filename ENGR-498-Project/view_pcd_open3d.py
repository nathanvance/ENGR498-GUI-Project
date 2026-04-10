from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

BACKGROUND_COLOR = np.array([140, 140, 140], dtype=np.uint8)


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: python view_pcd_open3d.py <point_cloud.pcd>")
        return 1

    point_cloud_path = Path(sys.argv[1]).resolve()
    if not point_cloud_path.is_file():
        print(f"Point cloud not found: {point_cloud_path}")
        return 1

    import open3d as o3d

    point_cloud = o3d.io.read_point_cloud(str(point_cloud_path))
    if not point_cloud.has_points():
        print(f"Point cloud is empty: {point_cloud_path}")
        return 1

    if not point_cloud.has_colors():
        num_points = len(point_cloud.points)
        colors = np.tile(BACKGROUND_COLOR, (num_points, 1))
        point_cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255.0)

    window_title = f"SLAM Point Cloud - {point_cloud_path.name}"
    try:
        app = o3d.visualization.gui.Application.instance
        app.initialize()
        visualizer = o3d.visualization.O3DVisualizer(window_title, 1600, 900)
        visualizer.show_skybox(False)
        visualizer.add_geometry("slam_cloud", point_cloud)
        visualizer.reset_camera_to_default()
        app.add_window(visualizer)
        app.run()
    except Exception as exc:
        print(f"[warn] O3DVisualizer failed ({exc}); falling back to draw_geometries.")
        o3d.visualization.draw_geometries(
            [point_cloud],
            window_name=window_title,
            width=1600,
            height=900,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
