from __future__ import annotations

import argparse
from pathlib import Path

import laspy
import numpy as np


def fix_las_format(input_path: Path, output_path: Path) -> None:
    las = laspy.read(input_path)

    las.return_number[:] = 1
    las.number_of_returns[:] = 1

    for color_dim in ("red", "green", "blue"):
        if color_dim in las.point_format.dimension_names:
            getattr(las, color_dim)[:] = 0

    intensity = las.intensity.astype(float)
    intensity = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-9)
    las.intensity = (intensity * 48000 + 2000).astype(np.uint16)

    xyz = np.column_stack((np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)))
    offset = np.array([300000, 6800000, 130], dtype=float)

    las.header.offsets = offset
    las.x = xyz[:, 0] + offset[0]
    las.y = xyz[:, 1] + offset[1]
    las.z = xyz[:, 2] + offset[2]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    las.write(output_path)


def main() -> int:
    parser = argparse.ArgumentParser(description="Normalize a LAS file into the MATLAB-friendly format used by the powerline tooling.")
    parser.add_argument("input_las")
    parser.add_argument("output_las")
    args = parser.parse_args()

    fix_las_format(Path(args.input_las), Path(args.output_las))
    print(f"FINAL FIXED: {args.output_las}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
