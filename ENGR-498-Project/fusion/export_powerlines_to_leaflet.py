from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


POWERLINE_COLOR_RGB = [95, 195, 235]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert generated ENGR498 powerline outputs (wires_points.npz and wire_info.json) "
            "into a Leaflet-friendly JSON overlay."
        )
    )
    parser.add_argument("--wires-npz", required=True, help="Path to generated wires_points.npz")
    parser.add_argument("--wire-info-json", default="", help="Path to generated wire_info.json")
    parser.add_argument("--ground-points-npz", default="", help="Optional ground_points.npz for provenance only")
    parser.add_argument("--output-json", required=True, help="Output JSON path for the Leaflet overlay")
    parser.add_argument(
        "--max-polyline-points",
        type=int,
        default=250,
        help="Maximum number of points to keep per powerline polyline in the exported JSON",
    )
    return parser.parse_args()


def _to_serializable(value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return [_to_serializable(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _to_serializable(item) for key, item in value.items()}
    return str(value)


def load_wire_points(npz_path: Path) -> list[tuple[str, np.ndarray]]:
    with np.load(npz_path, allow_pickle=True) as archive:
        if "wires" in archive.files:
            wires_obj = archive["wires"].item()
            if not isinstance(wires_obj, dict) or "Location" not in wires_obj:
                raise ValueError(f"Unsupported MATLAB-style wires archive format in {npz_path}")
            locations = wires_obj["Location"]
            result: list[tuple[str, np.ndarray]] = []
            for index, location in enumerate(locations, start=1):
                points = np.asarray(location, dtype=np.float64)
                if points.ndim != 2 or points.shape[1] != 3:
                    continue
                result.append((f"wire_{index}", points))
            return result

        result = []
        for key in sorted(archive.files):
            points = np.asarray(archive[key], dtype=np.float64)
            if points.ndim != 2 or points.shape[1] != 3:
                continue
            result.append((key, points))
        return result


def load_wire_metadata(json_path: Path | None) -> list[dict[str, Any]]:
    if json_path is None:
        return []
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    if not isinstance(payload, list):
        raise ValueError(f"Expected a JSON array in {json_path}")
    return [item for item in payload if isinstance(item, dict)]


def load_ground_summary(npz_path: Path | None) -> dict[str, Any] | None:
    if npz_path is None:
        return None
    with np.load(npz_path, allow_pickle=True) as archive:
        first_key = archive.files[0] if archive.files else None
        if first_key is None:
            return {"num_ground_points": 0}
        points = np.asarray(archive[first_key], dtype=np.float64)
        return {"num_ground_points": int(points.shape[0])}


def order_points_along_wire(points: np.ndarray) -> np.ndarray:
    if points.shape[0] <= 2:
        return points

    xy = points[:, :2]
    xy_centered = xy - xy.mean(axis=0)
    if np.allclose(xy_centered, 0.0):
        xyz_centered = points - points.mean(axis=0)
        cov = np.cov(xyz_centered, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        ordering = np.argsort(xyz_centered @ axis)
        return points[ordering]

    cov = np.cov(xy_centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    axis_xy = eigenvectors[:, int(np.argmax(eigenvalues))]
    ordering = np.argsort(xy_centered @ axis_xy)
    return points[ordering]


def resample_polyline(points: np.ndarray, max_points: int) -> np.ndarray:
    ordered = order_points_along_wire(points)
    if ordered.shape[0] <= max_points:
        return ordered
    sample_idx = np.linspace(0, ordered.shape[0] - 1, max_points)
    sample_idx = np.unique(np.round(sample_idx).astype(np.int64))
    return ordered[sample_idx]


def polyline_length(points: np.ndarray) -> float:
    if points.shape[0] < 2:
        return 0.0
    deltas = np.diff(points, axis=0)
    return float(np.linalg.norm(deltas, axis=1).sum())


def build_powerline_record(
    wire_name: str,
    points: np.ndarray,
    instance_number: int,
    fit_metadata: dict[str, Any] | None,
    max_polyline_points: int,
) -> dict[str, Any]:
    centroid = points.mean(axis=0)
    bbox_min = points.min(axis=0)
    bbox_max = points.max(axis=0)
    polyline_points = resample_polyline(points, max_polyline_points)
    end_to_end_length = (
        float(np.linalg.norm(polyline_points[-1] - polyline_points[0]))
        if polyline_points.shape[0] >= 2
        else 0.0
    )

    fit_payload = _to_serializable(fit_metadata) if fit_metadata is not None else None
    quality_score = None
    if isinstance(fit_metadata, dict) and fit_metadata.get("rsq") is not None:
        try:
            quality_score = float(fit_metadata["rsq"])
        except (TypeError, ValueError):
            quality_score = None

    return {
        "object_name": f"powerline_{instance_number:02d}",
        "source_wire_key": wire_name,
        "class_name": "powerline",
        "instance_number": instance_number,
        "quality_score": quality_score,
        "num_points": int(points.shape[0]),
        "centroid_map_xyz": [float(value) for value in centroid],
        "bbox_aabb_min_xyz": [float(value) for value in bbox_min],
        "bbox_aabb_max_xyz": [float(value) for value in bbox_max],
        "class_color_rgb": POWERLINE_COLOR_RGB,
        "polyline_map_xyz": [[float(value) for value in row] for row in polyline_points],
        "metrics": {
            "polyline_length_m": polyline_length(polyline_points),
            "end_to_end_length_m": end_to_end_length,
            "sag_m": None,
            "ground_clearance_m": None,
        },
        "fit_metadata": fit_payload,
        "gps": {"lat": None, "lon": None, "alt": None},
        "polyline_gps": [],
    }


def main() -> int:
    args = parse_args()

    wires_npz_path = Path(args.wires_npz)
    wire_info_path = Path(args.wire_info_json) if args.wire_info_json else None
    ground_points_path = Path(args.ground_points_npz) if args.ground_points_npz else None
    output_path = Path(args.output_json)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    wires = load_wire_points(wires_npz_path)
    wire_metadata = load_wire_metadata(wire_info_path)
    ground_summary = load_ground_summary(ground_points_path)

    powerlines: list[dict[str, Any]] = []
    for index, (wire_name, points) in enumerate(wires, start=1):
        metadata = wire_metadata[index - 1] if index - 1 < len(wire_metadata) else None
        powerlines.append(
            build_powerline_record(
                wire_name=wire_name,
                points=points,
                instance_number=index,
                fit_metadata=metadata,
                max_polyline_points=max(8, args.max_polyline_points),
            )
        )

    payload = {
        "pipeline": "eng498_powerline_leaflet_hook",
        "sources": {
            "wires_npz": str(wires_npz_path.resolve()),
            "wire_info_json": str(wire_info_path.resolve()) if wire_info_path else None,
            "ground_points_npz": str(ground_points_path.resolve()) if ground_points_path else None,
        },
        "summary": {
            "num_powerlines": len(powerlines),
            "wire_metadata_entries": len(wire_metadata),
            "wire_metadata_count_matches": len(powerlines) == len(wire_metadata) if wire_info_path else False,
            "ground_summary": ground_summary,
        },
        "powerlines": powerlines,
    }

    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"[done] wrote {len(powerlines)} powerlines to {output_path}")
    if wire_info_path and len(powerlines) != len(wire_metadata):
        print(
            "[warn] wire_info.json entry count does not match wires_points.npz; "
            "metadata was applied by index where available."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
