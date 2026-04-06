from __future__ import annotations

import argparse
import random

import numpy as np
import open3d as o3d

from project_paths import DEFAULT_WIRES_NPZ_PATH


def main() -> int:
    parser = argparse.ArgumentParser(description="Visualize wires stored in a MATLAB-style NPZ archive.")
    parser.add_argument("--wires-npz", default=str(DEFAULT_WIRES_NPZ_PATH))
    args = parser.parse_args()

    data = np.load(args.wires_npz, allow_pickle=True)
    wires = data["wires"].item()
    locations = wires["Location"]

    pcds = []
    for wire_points in locations:
        pts = np.array(wire_points, dtype=float)
        if pts.size == 0:
            continue

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pts)
        pc.paint_uniform_color([random.random(), random.random(), random.random()])
        pcds.append(pc)

    o3d.visualization.draw_geometries(pcds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
