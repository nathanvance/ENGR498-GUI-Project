#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def _is_finite_number(value: str | None) -> bool:
    if value is None:
        return False
    text = str(value).strip()
    if not text:
        return False
    try:
        parsed = float(text)
    except ValueError:
        return False
    return math.isfinite(parsed)


def _quaternion_is_valid(row: dict) -> bool:
    """Return True if the quaternion columns are finite and the norm is close to 1."""
    for col in ("qx", "qy", "qz", "qw"):
        if not _is_finite_number(row.get(col)):
            return False
    qx = float(row["qx"])
    qy = float(row["qy"])
    qz = float(row["qz"])
    qw = float(row["qw"])
    norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
    return 0.5 <= norm_sq <= 2.0


def sanitize_gps_csv(csv_path: Path) -> tuple[int, int]:
    if not csv_path.is_file():
        return 0, 0

    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    kept_rows = []
    dropped = 0
    for row in rows:
        if _is_finite_number(row.get("latitude_deg")) and _is_finite_number(row.get("longitude_deg")):
            kept_rows.append(row)
        else:
            dropped += 1

    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(kept_rows)

    return len(kept_rows), dropped


def sanitize_dense_trajectory_csv(csv_path: Path) -> tuple[int, int]:
    """Sort by timestamp, deduplicate, drop non-OK and geometrically invalid rows."""
    if not csv_path.is_file():
        return 0, 0

    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    required_cols = {"timestamp_sec", "x", "y", "z", "qx", "qy", "qz", "qw", "status"}
    if not required_cols.issubset(set(fieldnames)):
        print(f"[sanitize] dense traj csv: missing columns, skipping sanitization. path={csv_path}")
        return len(rows), 0

    kept_rows = []
    dropped = 0
    for row in rows:
        # Only keep OK rows with finite pose and valid quaternion.
        status = str(row.get("status", "")).strip().upper()
        if status != "OK":
            dropped += 1
            continue
        if not all(_is_finite_number(row.get(c)) for c in ("timestamp_sec", "x", "y", "z")):
            dropped += 1
            continue
        if not _quaternion_is_valid(row):
            dropped += 1
            continue
        kept_rows.append(row)

    # Sort by timestamp, then deduplicate (keep first occurrence per timestamp).
    kept_rows.sort(key=lambda r: float(r["timestamp_sec"]))
    deduped = []
    seen_ts: set[str] = set()
    for row in kept_rows:
        ts_key = row["timestamp_sec"]
        if ts_key not in seen_ts:
            seen_ts.add(ts_key)
            deduped.append(row)
        else:
            dropped += 1

    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(deduped)

    return len(deduped), dropped


def main() -> int:
    parser = argparse.ArgumentParser(description="Drop invalid rows from pose recovery CSV outputs.")
    parser.add_argument("--gps-csv", required=True)
    parser.add_argument("--dense-traj-csv", default="")
    args = parser.parse_args()

    kept, dropped = sanitize_gps_csv(Path(args.gps_csv))
    print(f"[sanitize] gps csv: kept={kept} dropped={dropped} path={args.gps_csv}")

    if args.dense_traj_csv:
        kept_d, dropped_d = sanitize_dense_trajectory_csv(Path(args.dense_traj_csv))
        print(f"[sanitize] dense traj csv: kept={kept_d} dropped={dropped_d} path={args.dense_traj_csv}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
