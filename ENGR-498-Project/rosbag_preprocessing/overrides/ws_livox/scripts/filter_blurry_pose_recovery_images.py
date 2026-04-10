#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import cv2


DEFAULT_BLUR_THRESHOLD = 100.0
SUPPORTED_IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def _timestamp_key(value: str) -> str:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ValueError(f"Invalid timestamp value: {value!r}") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"Non-finite timestamp value: {value!r}")
    return f"{parsed:.9f}"


def _read_csv_rows(csv_path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)
    if not fieldnames:
        raise ValueError(f"CSV is missing a header: {csv_path}")
    return fieldnames, rows


def _write_csv_rows(csv_path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _laplacian_variance(image_path: Path) -> float:
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"OpenCV could not read image: {image_path}")
    return float(cv2.Laplacian(image, cv2.CV_64F).var())


def filter_blurry_images(
    *,
    images_dir: Path,
    image_timestamps_csv: Path,
    camera_csv: Path,
    blur_threshold: float,
) -> tuple[int, int, int]:
    if blur_threshold < 0:
        raise ValueError(f"Blur threshold must be non-negative, got {blur_threshold}")
    if not images_dir.is_dir():
        raise FileNotFoundError(f"Images directory not found: {images_dir}")
    if not image_timestamps_csv.is_file():
        raise FileNotFoundError(f"Image timestamps CSV not found: {image_timestamps_csv}")
    if not camera_csv.is_file():
        raise FileNotFoundError(f"Camera CSV not found: {camera_csv}")

    ts_fieldnames, timestamp_rows = _read_csv_rows(image_timestamps_csv)
    cam_fieldnames, camera_rows = _read_csv_rows(camera_csv)

    required_ts_cols = {"filename", "t_query_sec"}
    required_cam_cols = {"t_query_sec"}
    if not required_ts_cols.issubset(ts_fieldnames):
        raise ValueError(f"{image_timestamps_csv} is missing required columns: {sorted(required_ts_cols - set(ts_fieldnames))}")
    if not required_cam_cols.issubset(cam_fieldnames):
        raise ValueError(f"{camera_csv} is missing required columns: {sorted(required_cam_cols - set(cam_fieldnames))}")

    disk_images = {
        path.name: path
        for path in images_dir.iterdir()
        if path.is_file() and path.suffix.lower() in SUPPORTED_IMAGE_SUFFIXES
    }

    seen_filenames: set[str] = set()
    kept_timestamp_rows: list[dict[str, str]] = []
    blurry_filenames: list[str] = []
    kept_timestamp_keys: list[str] = []

    for row in timestamp_rows:
        filename = (row.get("filename") or "").strip()
        if not filename:
            raise ValueError(f"{image_timestamps_csv} contains an empty filename row")
        if filename in seen_filenames:
            raise ValueError(f"{image_timestamps_csv} contains duplicate filename entries for {filename}")
        seen_filenames.add(filename)

        image_path = disk_images.get(filename)
        if image_path is None:
            raise FileNotFoundError(f"Timestamp CSV references a missing image file: {images_dir / filename}")

        focus_score = _laplacian_variance(image_path)
        if focus_score < blur_threshold:
            blurry_filenames.append(filename)
            continue

        kept_timestamp_rows.append(row)
        kept_timestamp_keys.append(_timestamp_key(row["t_query_sec"]))

    extra_images = sorted(set(disk_images) - seen_filenames)
    if extra_images:
        raise ValueError(
            f"Images directory contains files not tracked by {image_timestamps_csv}: "
            + ", ".join(extra_images[:10])
        )

    camera_by_key: dict[str, dict[str, str]] = {}
    for row in camera_rows:
        key = _timestamp_key(row["t_query_sec"])
        if key in camera_by_key:
            raise ValueError(f"{camera_csv} contains duplicate t_query_sec entries for {row['t_query_sec']}")
        camera_by_key[key] = row

    kept_camera_rows: list[dict[str, str]] = []
    for key in kept_timestamp_keys:
        if key not in camera_by_key:
            raise ValueError(f"Camera CSV is missing a row for retained image timestamp {key}")
        kept_camera_rows.append(camera_by_key[key])

    for filename in blurry_filenames:
        (images_dir / filename).unlink()

    _write_csv_rows(image_timestamps_csv, ts_fieldnames, kept_timestamp_rows)
    _write_csv_rows(camera_csv, cam_fieldnames, kept_camera_rows)

    total = len(timestamp_rows)
    discarded = len(blurry_filenames)
    retained = len(kept_timestamp_rows)
    return total, discarded, retained


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Remove blurry extracted pose-recovery images and keep camera metadata aligned."
    )
    parser.add_argument("--images-dir", required=True)
    parser.add_argument("--image-timestamps-csv", required=True)
    parser.add_argument("--camera-csv", required=True)
    parser.add_argument("--blur-threshold", type=float, default=DEFAULT_BLUR_THRESHOLD)
    args = parser.parse_args()

    total, discarded, retained = filter_blurry_images(
        images_dir=Path(args.images_dir),
        image_timestamps_csv=Path(args.image_timestamps_csv),
        camera_csv=Path(args.camera_csv),
        blur_threshold=args.blur_threshold,
    )

    print(f"[blur-filter] threshold={args.blur_threshold}")
    print(f"[blur-filter] total images checked={total}")
    print(f"[blur-filter] blurry images discarded={discarded}")
    print(f"[blur-filter] sharp images retained={retained}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
