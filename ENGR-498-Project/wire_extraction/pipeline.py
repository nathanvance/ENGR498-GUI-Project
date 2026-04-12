from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import laspy
import numpy as np
from scipy import ndimage, sparse
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from project_paths import DEFAULT_LAS_PATH, MATLAB_OUTPUT_DIR
from wire_extraction.native import compute_local_principal_features
from wire_extraction.stitching import (
    ClusterFeature,
    WireStitchingParameters,
    stitch_wire_clusters,
)


@dataclass(frozen=True)
class WireExtractionParameters:
    grid_resolution: float = 1.0
    max_window_radius: float = 8.0
    slope_threshold: float = 0.2
    elevation_threshold: float = 0.4
    elevation_scale: float = 1.25
    pca_radius: float = 0.5
    angle_threshold_deg: float = 15.0
    linearness_threshold: float = 0.95
    first_cluster_distance: float = 1.0
    second_cluster_distance: float = 0.3
    min_cluster_size: int = 15
    min_wire_span_m: float = 1.5
    feature_chunk_size: int = 50000


@dataclass
class WireCluster:
    location: np.ndarray
    label: int = 0
    count: int = 0
    ids: list[int] | None = None

    def __post_init__(self) -> None:
        self.location = np.asarray(self.location, dtype=np.float64)
        if self.ids is None:
            self.ids = []
        if self.count == 0:
            self.count = int(self.location.shape[0])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the fast Python/C++ wire extraction pipeline.")
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
    return parser.parse_args()


def read_las_xyz(path: Path) -> np.ndarray:
    las = laspy.read(path)
    return np.column_stack((las.x, las.y, las.z)).astype(np.float64, copy=False)


def fill_missing_grid_cells(grid: np.ndarray) -> np.ndarray:
    missing_mask = ~np.isfinite(grid)
    if not np.any(missing_mask):
        return grid
    nearest_idx = ndimage.distance_transform_edt(
        missing_mask,
        return_distances=False,
        return_indices=True,
    )
    return grid[tuple(nearest_idx)]


def segment_ground_smrf_like(points: np.ndarray, params: WireExtractionParameters) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if points.size == 0:
        empty_mask = np.zeros(0, dtype=bool)
        return empty_mask, points, points

    min_xy = points[:, :2].min(axis=0)
    grid_idx = np.floor((points[:, :2] - min_xy) / params.grid_resolution).astype(np.int32)
    nx = int(grid_idx[:, 0].max()) + 1
    ny = int(grid_idx[:, 1].max()) + 1

    min_grid = np.full((ny, nx), np.inf, dtype=np.float64)
    np.minimum.at(min_grid, (grid_idx[:, 1], grid_idx[:, 0]), points[:, 2])
    min_grid = fill_missing_grid_cells(min_grid)

    window_radius_cells = max(1, int(round(params.max_window_radius / params.grid_resolution)))
    opened = ndimage.grey_opening(min_grid, size=(2 * window_radius_cells + 1, 2 * window_radius_cells + 1))

    base_tol = params.elevation_threshold * params.elevation_scale
    point_surface = np.minimum(min_grid, opened + base_tol)[grid_idx[:, 1], grid_idx[:, 0]]
    point_tolerance = base_tol + params.slope_threshold * params.grid_resolution
    ground_mask = points[:, 2] <= (point_surface + point_tolerance)

    ground_points = points[ground_mask]
    non_ground_points = points[~ground_mask]
    return ground_mask, non_ground_points, ground_points


def _flatten_neighbor_lists(neighbor_lists: list[list[int] | np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    lengths = np.fromiter((len(neighbors) for neighbors in neighbor_lists), count=len(neighbor_lists), dtype=np.int64)
    offsets = np.empty(lengths.size + 1, dtype=np.int64)
    offsets[0] = 0
    np.cumsum(lengths, out=offsets[1:])

    flat = np.empty(int(offsets[-1]), dtype=np.int64)
    cursor = 0
    for neighbors in neighbor_lists:
        count = len(neighbors)
        if count:
            flat[cursor:cursor + count] = neighbors
        cursor += count
    return flat, offsets


def extract_powerline_candidates(points: np.ndarray, params: WireExtractionParameters) -> np.ndarray:
    if points.shape[0] == 0:
        return np.zeros(0, dtype=bool)

    tree = cKDTree(points)
    principal_dirs = np.zeros((points.shape[0], 3), dtype=np.float64)
    linearness = np.zeros(points.shape[0], dtype=np.float64)
    valid = np.zeros(points.shape[0], dtype=bool)

    for start in range(0, points.shape[0], params.feature_chunk_size):
        stop = min(points.shape[0], start + params.feature_chunk_size)
        chunk = points[start:stop]
        neighbor_lists = tree.query_ball_point(chunk, params.pca_radius, workers=-1)
        flat_neighbors, offsets = _flatten_neighbor_lists(neighbor_lists)
        dirs_chunk, linearness_chunk, valid_chunk = compute_local_principal_features(
            points,
            flat_neighbors,
            offsets,
        )
        principal_dirs[start:stop] = dirs_chunk
        linearness[start:stop] = linearness_chunk
        valid[start:stop] = valid_chunk

    dir_norm = np.linalg.norm(principal_dirs, axis=1)
    safe_norm = np.where(dir_norm > 1e-12, dir_norm, 1.0)
    cos_theta = np.clip(principal_dirs[:, 2] / safe_norm, -1.0, 1.0)
    angle = np.degrees(np.arccos(cos_theta))
    return valid & (np.abs(angle - 90.0) < params.angle_threshold_deg) & (linearness > params.linearness_threshold)


def euclidean_cluster_labels(points: np.ndarray, distance: float) -> tuple[np.ndarray, int]:
    if points.shape[0] == 0:
        return np.zeros(0, dtype=np.int32), 0
    if points.shape[0] == 1:
        return np.ones(1, dtype=np.int32), 1

    tree = cKDTree(points)
    pairs = tree.query_pairs(distance, output_type="ndarray")
    if pairs.size == 0:
        labels = np.arange(points.shape[0], dtype=np.int32) + 1
        return labels, int(labels.size)

    row = np.concatenate([pairs[:, 0], pairs[:, 1]])
    col = np.concatenate([pairs[:, 1], pairs[:, 0]])
    data = np.ones(row.shape[0], dtype=np.uint8)
    graph = sparse.coo_matrix((data, (row, col)), shape=(points.shape[0], points.shape[0]))
    num_components, labels = connected_components(graph, directed=False, return_labels=True)
    return labels.astype(np.int32) + 1, int(num_components)


def keep_large_candidate_clusters(points: np.ndarray, labels: np.ndarray, num_clusters: int, min_cluster_size: int) -> np.ndarray:
    kept_points: list[np.ndarray] = []
    for cluster_id in range(1, num_clusters + 1):
        cluster_mask = labels == cluster_id
        if int(np.sum(cluster_mask)) < min_cluster_size:
            continue
        kept_points.append(points[cluster_mask])
    if not kept_points:
        return np.empty((0, 3), dtype=np.float64)
    return np.vstack(kept_points)


def rotate_z(points: np.ndarray, alpha_rad: float) -> np.ndarray:
    rotation = np.array(
        [
            [math.cos(alpha_rad), -math.sin(alpha_rad), 0.0],
            [math.sin(alpha_rad), math.cos(alpha_rad), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return points @ rotation.T


def eigen_direction(points: np.ndarray) -> tuple[float, np.ndarray, float]:
    covariance = np.cov(points, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    principal_vector = eigenvectors[:, order[0]]
    if np.cross(np.array([1.0, 0.0, 0.0], dtype=np.float64), principal_vector)[2] < 0.0:
        principal_vector = -principal_vector
    dot_value = float(np.clip(principal_vector[0] / max(np.linalg.norm(principal_vector), 1e-12), -1.0, 1.0))
    angle_deg = math.degrees(math.acos(dot_value))
    return float(eigenvalues[order[0]]), principal_vector, angle_deg


def get_dist(points: np.ndarray) -> float:
    if points.shape[0] < 3:
        return 0.0
    shifted = points - points.mean(axis=0)
    _, _, angle_deg = eigen_direction(shifted)
    rotated = rotate_z(shifted, -math.radians(angle_deg))
    shifted_x = rotated[:, 0]
    return float(shifted_x.max() - shifted_x.min())


def find_merge(power_lines: list[WireCluster], order: np.ndarray, begin: int) -> list[int]:
    merged_ids: list[int] = []
    anchor = power_lines[int(order[begin])].location
    if anchor.shape[0] < 10:
        return merged_ids

    mean_anchor_z = float(np.mean(anchor[:, 2]))
    anchor_shift = anchor - anchor.mean(axis=0)
    _, _, angle_deg = eigen_direction(anchor_shift)
    anchor_rotated = rotate_z(anchor_shift, -math.radians(angle_deg))
    anchor_shift_x = anchor_rotated[:, 0]
    anchor_shift_z = anchor_rotated[:, 2]
    fit_coeffs = np.polyfit(anchor_shift_x, anchor_shift_z, 2)
    xy_fit = np.polyfit(anchor[:, 0], anchor[:, 1], 1)

    combined_x = anchor_shift_x.copy()
    combined_z = anchor_shift_z.copy()

    for cursor in range(begin + 1, order.size):
        other_idx = int(order[cursor])
        if power_lines[other_idx].label == 1:
            continue

        other = power_lines[other_idx].location
        mean_other_z = float(np.mean(other[:, 2]))
        other_shift = other - anchor.mean(axis=0)
        other_rotated = rotate_z(other_shift, -math.radians(angle_deg))
        other_shift_x = other_rotated[:, 0]
        other_shift_z = other_rotated[:, 2]

        projected_z = np.polyval(fit_coeffs, other_shift_x)
        projected_y = np.polyval(xy_fit, other[:, 0])
        delta_by = float(np.abs(np.mean(projected_y - other[:, 1])))
        delta_bz = float(np.abs(mean_anchor_z - mean_other_z))
        mean_bd = float(np.max(np.abs(projected_z - other_shift_z)))

        if delta_by < 0.5 and mean_bd < 0.2 and delta_bz < 4.0:
            merged_ids.append(other_idx)
            combined_x = np.concatenate([combined_x, other_shift_x])
            combined_z = np.concatenate([combined_z, other_shift_z])
            fit_coeffs = np.polyfit(combined_x, combined_z, 2)

    return merged_ids


def merge_once(power_lines: list[WireCluster], order: np.ndarray) -> tuple[list[WireCluster], np.ndarray]:
    for cursor in range(order.size):
        cluster_idx = int(order[cursor])
        if power_lines[cluster_idx].label == 1:
            continue
        ids = find_merge(power_lines, order, cursor)
        power_lines[cluster_idx].ids = ids
        for merged_idx in ids:
            power_lines[merged_idx].label = 1

    merged_clusters = [cluster for cluster in power_lines if cluster.label == 0]
    for cluster in merged_clusters:
        if cluster.ids:
            cluster.location = np.vstack([cluster.location] + [power_lines[idx].location for idx in cluster.ids])

    counts: list[int] = []
    for cluster in merged_clusters:
        cluster.label = 0
        cluster.count = int(cluster.location.shape[0])
        cluster.ids = []
        counts.append(cluster.count)

    if counts:
        new_order = np.argsort(np.asarray(counts))[::-1]
    else:
        new_order = np.zeros(0, dtype=np.int64)
    return merged_clusters, new_order.astype(np.int64)


def merge_powerline_clusters(power_lines: list[WireCluster]) -> list[WireCluster]:
    if not power_lines:
        return []
    counts = np.asarray([cluster.count for cluster in power_lines], dtype=np.int64)
    order = np.argsort(counts)[::-1]
    merged = power_lines
    for _ in range(3):
        merged, order = merge_once(merged, order)
        if not merged:
            break
    return merged


def build_powerline_clusters(points: np.ndarray, labels: np.ndarray, num_clusters: int) -> list[WireCluster]:
    clusters: list[WireCluster] = []
    for cluster_id in range(1, num_clusters + 1):
        cluster_points = points[labels == cluster_id]
        if cluster_points.size == 0:
            continue
        clusters.append(WireCluster(location=cluster_points))
    return clusters


def fit_quadratic_metadata(points: np.ndarray) -> dict[str, Any]:
    x = np.asarray(points[:, 0], dtype=np.float64)
    z = np.asarray(points[:, 2], dtype=np.float64)
    mu_mean = float(np.mean(x))
    mu_std = float(np.std(x))
    if not math.isfinite(mu_std) or mu_std <= 1e-12:
        mu_std = 1.0
    x_norm = (x - mu_mean) / mu_std

    coeffs, residuals, _, _, _ = np.polyfit(x_norm, z, 2, full=True)
    predicted = np.polyval(coeffs, x_norm)
    sse = float(np.sum((z - predicted) ** 2))
    normr = float(math.sqrt(sse))
    df = int(max(0, x.shape[0] - 3))
    sst = float(np.sum((z - np.mean(z)) ** 2))
    rsq = 1.0 - (sse / sst) if sst > 1e-12 else 1.0

    return {
        "p": [float(value) for value in coeffs],
        "normr": normr,
        "df": df,
        "rsq": float(rsq),
        "mu_mean": mu_mean,
        "mu_std": mu_std,
    }


def _vector_to_list(vector: np.ndarray) -> list[float]:
    return [float(value) for value in np.asarray(vector, dtype=np.float64).tolist()]


def build_wire_metadata_entry(
    points: np.ndarray,
    feature: ClusterFeature,
    *,
    source_cluster_indices: list[int],
    stage: str,
    accepted_edge_indices: list[int] | None = None,
    validation: dict[str, Any] | None = None,
) -> dict[str, Any]:
    metadata = fit_quadratic_metadata(points)
    metadata["point_count"] = int(points.shape[0])
    metadata["geometry"] = {
        "centroid": _vector_to_list(feature.centroid),
        "bbox_min": _vector_to_list(feature.bbox_min),
        "bbox_max": _vector_to_list(feature.bbox_max),
        "principal_direction": _vector_to_list(feature.principal_direction),
        "span_direction": _vector_to_list(feature.span_direction),
        "plane_point": _vector_to_list(feature.plane_point),
        "plane_normal": _vector_to_list(feature.plane_normal),
        "plane_rms_m": float(feature.plane_rms_m),
        "projected_span_length_m": float(feature.projected_span_length_m),
        "start_point": _vector_to_list(feature.start_point),
        "end_point": _vector_to_list(feature.end_point),
    }
    metadata["catenary_fit"] = feature.catenary_fit.to_dict()
    metadata["stitching"] = {
        "stage": stage,
        "source_cluster_indices": [int(value) for value in source_cluster_indices],
        "source_cluster_count": int(len(source_cluster_indices)),
        "accepted_edge_indices": [int(value) for value in (accepted_edge_indices or [])],
        "validation": validation,
    }
    return metadata


def build_wire_payload(clusters: list[WireCluster]) -> dict[str, Any]:
    return {
        "Location": [np.asarray(cluster.location, dtype=np.float64) for cluster in clusters],
        "Label": [int(cluster.label) for cluster in clusters],
        "Count": [int(cluster.count) for cluster in clusters],
        "Ids": [list(cluster.ids or []) for cluster in clusters],
    }


def save_outputs(
    output_dir: Path,
    wire_payload: dict[str, Any],
    poly_metadata: list[dict[str, Any]],
    ground_points: np.ndarray,
    *,
    pre_stitch_payload: dict[str, Any] | None = None,
    pre_stitch_metadata: list[dict[str, Any]] | None = None,
    stitching_report: dict[str, Any] | None = None,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    wires_path = output_dir / "wires_points.npz"
    np.savez(wires_path, wires=wire_payload)
    print(f"Saved wires points to {wires_path}")

    wire_info_path = output_dir / "wire_info.json"
    wire_info_path.write_text(json.dumps(poly_metadata, indent=2), encoding="utf-8")
    print(f"Saved wire info to {wire_info_path}")

    ground_points_path = output_dir / "ground_points.npz"
    np.savez(ground_points_path, ground_points=np.asarray(ground_points, dtype=np.float64))
    print(f"Saved ground points to {ground_points_path}")

    if pre_stitch_payload is not None:
        pre_stitch_path = output_dir / "wire_clusters_pre_stitch.npz"
        np.savez(pre_stitch_path, wires=pre_stitch_payload)
        print(f"Saved pre-stitch wire clusters to {pre_stitch_path}")

    if pre_stitch_metadata is not None:
        pre_stitch_info_path = output_dir / "wire_clusters_pre_stitch_info.json"
        pre_stitch_info_path.write_text(json.dumps(pre_stitch_metadata, indent=2), encoding="utf-8")
        print(f"Saved pre-stitch wire metadata to {pre_stitch_info_path}")

    if stitching_report is not None:
        stitching_report_path = output_dir / "wire_stitching_report.json"
        stitching_report_path.write_text(json.dumps(stitching_report, indent=2), encoding="utf-8")
        print(f"Saved wire stitching report to {stitching_report_path}")


def run_wire_extraction(
    las_path: Path,
    output_dir: Path,
    params: WireExtractionParameters | None = None,
    stitching_params: WireStitchingParameters | None = None,
) -> tuple[dict[str, Any], list[dict[str, Any]], np.ndarray]:
    params = params or WireExtractionParameters()
    stitching_params = stitching_params or WireStitchingParameters()
    start_time = time.perf_counter()

    points = read_las_xyz(las_path)
    ground_mask, non_ground_points, ground_points = segment_ground_smrf_like(points, params)
    print(f"Total points: {points.shape[0]}")
    print(f"Ground points: {int(np.sum(ground_mask))}")
    print(f"Non-ground points: {int(non_ground_points.shape[0])}")

    candidate_mask = extract_powerline_candidates(non_ground_points, params)
    candidate_points = non_ground_points[candidate_mask]
    print(f"Candidate powerline points: {candidate_points.shape[0]}")

    first_labels, first_num_clusters = euclidean_cluster_labels(candidate_points, params.first_cluster_distance)
    clustered_candidates = keep_large_candidate_clusters(
        candidate_points,
        first_labels,
        first_num_clusters,
        params.min_cluster_size,
    )
    print(f"Points after first clustering: {clustered_candidates.shape[0]}")

    second_labels, second_num_clusters = euclidean_cluster_labels(clustered_candidates, params.second_cluster_distance)
    power_lines = build_powerline_clusters(clustered_candidates, second_labels, second_num_clusters)
    extracted_clusters = merge_powerline_clusters(power_lines)
    extracted_clusters = [cluster for cluster in extracted_clusters if get_dist(cluster.location) >= params.min_wire_span_m]
    print(f"Retained wire clusters before stitching: {len(extracted_clusters)}")

    stitching_result = stitch_wire_clusters(
        [cluster.location for cluster in extracted_clusters],
        stitching_params,
    )
    stitched_clusters = [
        WireCluster(
            location=group.points,
            ids=list(group.source_cluster_indices),
        )
        for group in stitching_result.groups
    ]
    merged_cluster_total = sum(
        1
        for group in stitching_result.groups
        if len(group.source_cluster_indices) > 1
    )
    print(
        "Stitched wire groups: "
        f"{len(stitched_clusters)} "
        f"(merged groups={merged_cluster_total})"
    )

    pre_stitch_clusters = [
        WireCluster(location=cluster.location, ids=[index])
        for index, cluster in enumerate(extracted_clusters)
    ]
    pre_stitch_payload = build_wire_payload(pre_stitch_clusters)
    pre_stitch_metadata = [
        build_wire_metadata_entry(
            feature.points,
            feature,
            source_cluster_indices=[feature.cluster_index],
            stage="pre_stitch_cluster",
        )
        for feature in stitching_result.cluster_features
    ]
    final_metadata = [
        build_wire_metadata_entry(
            group.points,
            group.feature,
            source_cluster_indices=list(group.source_cluster_indices),
            stage="stitched_group",
            accepted_edge_indices=list(group.accepted_edge_indices),
            validation=group.validation,
        )
        for group in stitching_result.groups
    ]
    wire_payload = build_wire_payload(stitched_clusters)
    save_outputs(
        output_dir,
        wire_payload,
        final_metadata,
        ground_points,
        pre_stitch_payload=pre_stitch_payload,
        pre_stitch_metadata=pre_stitch_metadata,
        stitching_report=stitching_result.to_report_dict(),
    )

    elapsed = time.perf_counter() - start_time
    print(f"Wire extraction completed in {elapsed:.2f} sec")
    return wire_payload, final_metadata, ground_points


def main() -> int:
    args = parse_args()
    las_path = Path(args.las_path).resolve()
    output_dir = Path(args.output_dir).resolve()
    run_wire_extraction(las_path, output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
