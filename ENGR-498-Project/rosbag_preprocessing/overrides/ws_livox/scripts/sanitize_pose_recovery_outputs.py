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


def main() -> int:
    parser = argparse.ArgumentParser(description="Drop invalid GPS rows from pose recovery CSV outputs.")
    parser.add_argument("--gps-csv", required=True)
    args = parser.parse_args()

    kept, dropped = sanitize_gps_csv(Path(args.gps_csv))
    print(f"[sanitize] gps csv: kept={kept} dropped={dropped} path={args.gps_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
