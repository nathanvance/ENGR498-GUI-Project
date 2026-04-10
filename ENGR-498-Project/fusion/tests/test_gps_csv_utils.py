from __future__ import annotations

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from gps_csv_utils import count_usable_tf_gps_rows  # noqa: E402


def _write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    fieldnames = [
        "t_in_sec",
        "t_query_sec",
        "x",
        "y",
        "z",
        "qx",
        "qy",
        "qz",
        "qw",
        "status",
        "latitude_deg",
        "longitude_deg",
        "altitude_m",
        "fix_status",
        "cov_xx_m2",
        "cov_yy_m2",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def test_count_usable_tf_gps_rows_returns_zero_for_missing_file(tmp_path: Path):
    assert count_usable_tf_gps_rows(tmp_path / "missing.csv") == 0


def test_count_usable_tf_gps_rows_filters_invalid_rows(tmp_path: Path):
    csv_path = tmp_path / "tf_gps_out.csv"
    _write_csv(
        csv_path,
        [
            {
                "t_in_sec": "0.0",
                "t_query_sec": "1.0",
                "x": "0",
                "y": "0",
                "z": "0",
                "qx": "0",
                "qy": "0",
                "qz": "0",
                "qw": "1",
                "status": "OK",
                "latitude_deg": "53.1",
                "longitude_deg": "-113.5",
                "altitude_m": "700.0",
                "fix_status": "1",
                "cov_xx_m2": "4.0",
                "cov_yy_m2": "4.0",
            },
            {
                "t_in_sec": "0.1",
                "t_query_sec": "1.1",
                "x": "0",
                "y": "0",
                "z": "0",
                "qx": "0",
                "qy": "0",
                "qz": "0",
                "qw": "1",
                "status": "NO_TF_AVAILABLE",
                "latitude_deg": "53.1",
                "longitude_deg": "-113.5",
                "altitude_m": "700.0",
                "fix_status": "1",
                "cov_xx_m2": "4.0",
                "cov_yy_m2": "4.0",
            },
            {
                "t_in_sec": "0.2",
                "t_query_sec": "1.2",
                "x": "0",
                "y": "0",
                "z": "0",
                "qx": "0",
                "qy": "0",
                "qz": "0",
                "qw": "1",
                "status": "OK",
                "latitude_deg": "",
                "longitude_deg": "-113.5",
                "altitude_m": "700.0",
                "fix_status": "1",
                "cov_xx_m2": "4.0",
                "cov_yy_m2": "4.0",
            },
        ],
    )

    assert count_usable_tf_gps_rows(csv_path) == 1


def test_count_usable_tf_gps_rows_applies_fix_status_and_covariance_filters(tmp_path: Path):
    csv_path = tmp_path / "tf_gps_out.csv"
    _write_csv(
        csv_path,
        [
            {
                "t_in_sec": "0.0",
                "t_query_sec": "1.0",
                "x": "0",
                "y": "0",
                "z": "0",
                "qx": "0",
                "qy": "0",
                "qz": "0",
                "qw": "1",
                "status": "OK",
                "latitude_deg": "53.1",
                "longitude_deg": "-113.5",
                "altitude_m": "700.0",
                "fix_status": "0",
                "cov_xx_m2": "1.0",
                "cov_yy_m2": "1.0",
            },
            {
                "t_in_sec": "0.1",
                "t_query_sec": "1.1",
                "x": "0",
                "y": "0",
                "z": "0",
                "qx": "0",
                "qy": "0",
                "qz": "0",
                "qw": "1",
                "status": "OK",
                "latitude_deg": "53.2",
                "longitude_deg": "-113.6",
                "altitude_m": "701.0",
                "fix_status": "2",
                "cov_xx_m2": "100.0",
                "cov_yy_m2": "100.0",
            },
            {
                "t_in_sec": "0.2",
                "t_query_sec": "1.2",
                "x": "0",
                "y": "0",
                "z": "0",
                "qx": "0",
                "qy": "0",
                "qz": "0",
                "qw": "1",
                "status": "OK",
                "latitude_deg": "53.3",
                "longitude_deg": "-113.7",
                "altitude_m": "702.0",
                "fix_status": "2",
                "cov_xx_m2": "4.0",
                "cov_yy_m2": "4.0",
            },
        ],
    )

    assert count_usable_tf_gps_rows(csv_path, min_fix_status=1, max_horizontal_cov_m2=10.0) == 1
