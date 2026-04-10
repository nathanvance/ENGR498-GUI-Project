from __future__ import annotations

import csv
import math
from pathlib import Path


def _parse_optional_float(value: object) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _parse_optional_int(value: object) -> int | None:
    parsed = _parse_optional_float(value)
    if parsed is None:
        return None
    try:
        return int(parsed)
    except (TypeError, ValueError):
        return None


def count_usable_tf_gps_rows(
    csv_path: Path,
    *,
    min_fix_status: int = 0,
    max_horizontal_cov_m2: float = 1000.0,
) -> int:
    """Count rows that are usable for local-to-GPS fitting."""
    if not csv_path.is_file():
        return 0

    usable = 0
    with csv_path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            status = str(row.get("status", "OK")).strip().upper()
            if status and status != "OK":
                continue

            lat = _parse_optional_float(row.get("latitude_deg"))
            lon = _parse_optional_float(row.get("longitude_deg"))
            alt = _parse_optional_float(row.get("altitude_m"))
            if lat is None or lon is None or alt is None:
                continue

            fix_status = _parse_optional_int(row.get("fix_status"))
            if fix_status is not None and fix_status < min_fix_status:
                continue

            cov_xx = _parse_optional_float(row.get("cov_xx_m2"))
            cov_yy = _parse_optional_float(row.get("cov_yy_m2"))
            if cov_xx is not None and cov_yy is not None:
                horiz_var = 0.5 * (cov_xx + cov_yy)
                if math.isfinite(horiz_var) and horiz_var > max_horizontal_cov_m2:
                    continue

            usable += 1

    return usable
