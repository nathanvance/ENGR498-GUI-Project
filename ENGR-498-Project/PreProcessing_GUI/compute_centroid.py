from __future__ import annotations

import argparse
import json

import numpy as np

from project_paths import DEFAULT_WIRES_NPZ_PATH


def compute_wire_centroids(npz_path: str) -> dict[str, dict[str, object]]:
    data = np.load(npz_path, allow_pickle=True)
    wires = data["wires"].item()
    locations = wires["Location"]

    centroids: dict[str, dict[str, object]] = {}
    for i, wire_points in enumerate(locations):
        pts = np.array(wire_points, dtype=float)
        if pts.size == 0:
            continue

        centroid = pts.mean(axis=0)
        centroids[f"wire_{i}"] = {
            "centroid": centroid.tolist(),
            "num_points": int(len(pts)),
        }

    return centroids


def main() -> int:
    parser = argparse.ArgumentParser(description="Compute centroids for wires stored in a MATLAB-style NPZ file.")
    parser.add_argument("--wires-npz", default=str(DEFAULT_WIRES_NPZ_PATH))
    parser.add_argument("--output-json", default="")
    args = parser.parse_args()

    centroids = compute_wire_centroids(args.wires_npz)
    if args.output_json:
        with open(args.output_json, "w", encoding="utf-8") as handle:
            json.dump(centroids, handle, indent=2)
    else:
        for wire_id, data in centroids.items():
            print(f"{wire_id}: centroid = {data['centroid']}, points = {data['num_points']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
