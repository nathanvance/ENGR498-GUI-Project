from __future__ import annotations

import argparse
import json
from pathlib import Path

import matlab.engine
import numpy as np

from project_paths import DEFAULT_LAS_PATH, MATLAB_EXTRACT_DIR, MATLAB_OUTPUT_DIR


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the MATLAB powerline extraction wrapper.")
    parser.add_argument(
        "--las-path",
        default=str(DEFAULT_LAS_PATH),
        help="Input LAS file. Defaults to the repo sample LAS file.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(MATLAB_OUTPUT_DIR),
        help="Directory for wires_points.npz, wire_info.json, and ground_points.npz.",
    )
    args = parser.parse_args()

    las_path = Path(args.las_path).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    eng = matlab.engine.start_matlab()
    eng.addpath(str(MATLAB_EXTRACT_DIR), nargout=0)

    print("Running MATLAB wire extraction...")
    PL, poly, ground_pts = eng.demo_extract_powerline(str(las_path), nargout=3)

    wires_path = output_dir / "wires_points.npz"
    np.savez(wires_path, wires=np.array(PL))
    print(f"Saved wires points to {wires_path}")

    poly_list = []
    for entry in poly:
        poly_list.append(
            {
                "p": list(entry["p"][0]),
                "normr": float(entry["normr"]),
                "df": float(entry["df"]),
                "rsq": float(entry["rsq"]),
                "mu_mean": float(entry["mu_mean"]),
                "mu_std": float(entry["mu_std"]),
            }
        )

    wire_info_path = output_dir / "wire_info.json"
    with wire_info_path.open("w", encoding="utf-8") as f:
        json.dump(poly_list, f, indent=2)
    print(f"Saved wire info to {wire_info_path}")

    ground_points_path = output_dir / "ground_points.npz"
    np.savez(ground_points_path, ground_points=np.array(ground_pts))
    print(f"Saved ground points to {ground_points_path}")

    eng.quit()
    print("MATLAB engine closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
