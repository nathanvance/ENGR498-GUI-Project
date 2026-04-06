from __future__ import annotations

import argparse

import laspy
import numpy as np

from project_paths import DEFAULT_LAS_PATH


def header_info(las: laspy.LasData) -> dict[str, object]:
    h = las.header
    return {
        "version": h.version,
        "point_format": h.point_format.id,
        "point_count": h.point_count,
        "scales": h.scales,
        "offsets": h.offsets,
        "mins": h.mins,
        "maxs": h.maxs,
        "software": h.generating_software,
        "system": h.system_identifier,
    }


def stats(arr: np.ndarray) -> dict[str, float]:
    return {
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare two LAS files field-by-field.")
    parser.add_argument("--good", default=str(DEFAULT_LAS_PATH))
    parser.add_argument("--bad", required=True)
    args = parser.parse_args()

    good = laspy.read(args.good)
    bad = laspy.read(args.bad)

    print("\n========== HEADER COMPARISON ==========\n")
    g = header_info(good)
    b = header_info(bad)

    for key in g:
        print(key)
        print(" good:", g[key])
        print(" bad :", b[key])
        print()

    print("\n========== DIMENSION COMPARISON ==========\n")
    g_dims = list(good.point_format.dimension_names)
    b_dims = list(bad.point_format.dimension_names)
    print("good dims:", g_dims)
    print("bad dims :", b_dims)
    print()

    common_dims = sorted(set(g_dims) & set(b_dims))
    print("\n========== FIELD STATISTICS ==========\n")
    for dim in common_dims:
        g_vals = good[dim]
        b_vals = bad[dim]
        print(f"--- {dim} ---")
        print("good:", stats(g_vals))
        print("bad :", stats(b_vals))
        print()

    print("\n========== NaN CHECK ==========\n")
    for dim in common_dims:
        g_nan = np.isnan(good[dim]).sum() if good[dim].dtype.kind == "f" else 0
        b_nan = np.isnan(bad[dim]).sum() if bad[dim].dtype.kind == "f" else 0
        print(f"{dim}: good={g_nan}  bad={b_nan}")

    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
