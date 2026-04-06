from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from native.pointcloud_accel import ensure_built, project_assign_best_detection


FUSION_ROOT = Path(__file__).resolve().parent
DEFAULT_ALLOWED_CLASSES = (
    "pole",
    "crossarm",
    "transformer",
    "pedestal",
    "pedestal_box",
    "pedestal box",
)
DEFAULT_REJECT_CLASSES = (
    "wire",
    "wires",
    "powerline",
    "powerlines",
    "power_line",
    "power_lines",
    "cable",
    "line",
    "lines",
)
CLASS_COLOR_TABLE = {
    "pole": np.array([220, 70, 70], dtype=np.uint8),
    "crossarm": np.array([70, 190, 105], dtype=np.uint8),
    "transformer": np.array([235, 160, 60], dtype=np.uint8),
    "pedestal": np.array([70, 120, 235], dtype=np.uint8),
    "pedestal_box": np.array([70, 120, 235], dtype=np.uint8),
    "pedestal box": np.array([70, 120, 235], dtype=np.uint8),
    "wire": np.array([95, 195, 235], dtype=np.uint8),
}
BACKGROUND_COLOR = np.array([140, 140, 140], dtype=np.uint8)


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion: np.ndarray


@dataclass(frozen=True)
class PoseRecord:
    timestamp: float
    translation: np.ndarray
    quaternion_xyzw: np.ndarray
    raw: dict[str, Any]


@dataclass(frozen=True)
class FrameRecord:
    stem: str
    image_name: str
    timestamp: float
    npz_path: Path
    meta_path: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fuse YOLO mask projections into a SLAM point cloud using JSON calibrations and a C++ accelerator."
    )
    parser.add_argument("--intrinsics-json", required=True)
    parser.add_argument("--extrinsics-json", required=True)
    parser.add_argument("--pose-csv", required=True)
    parser.add_argument("--image-timestamps-csv", required=True)
    parser.add_argument("--point-cloud", required=True)
    parser.add_argument("--mask-dir", required=True)
    parser.add_argument("--meta-dir", required=True)
    parser.add_argument("--output-dir", default=str(FUSION_ROOT))
    parser.add_argument("--time-column", default="t_query_sec")
    parser.add_argument("--image-filename-column", default="filename")
    parser.add_argument("--image-time-column", default="")
    parser.add_argument("--time-offset-sec", type=float, default=0.0)
    parser.add_argument("--allowed-classes", default=",".join(DEFAULT_ALLOWED_CLASSES))
    parser.add_argument("--reject-classes", default=",".join(DEFAULT_REJECT_CLASSES))
    parser.add_argument("--min-vote-to-keep", type=int, default=1)
    parser.add_argument("--knn-cluster", type=int, default=16)
    parser.add_argument("--eps-factor", type=float, default=2.5)
    parser.add_argument("--min-cluster-points", type=int, default=20)
    parser.add_argument("--min-keep-cluster", type=int, default=30)
    parser.add_argument("--stat-nb-neighbors", type=int, default=20)
    parser.add_argument("--stat-std-ratio", type=float, default=2.5)
    parser.add_argument("--min-pole-spacing-m", type=float, default=5.0)
    parser.add_argument("--max-pole-spacing-m", type=float, default=90.0)
    parser.add_argument("--pole-neighbor-top-k", type=int, default=2)
    parser.add_argument("--pole-spacing-adaptive-multiplier", type=float, default=2.2)
    parser.add_argument("--axis-length", type=float, default=0.25)
    parser.add_argument("--no-visualize", action="store_true")
    return parser.parse_args()


def _flatten_numeric_list(values: Any) -> list[float]:
    if values is None:
        return []
    if isinstance(values, (int, float)):
        return [float(values)]
    if isinstance(values, list):
        flattened: list[float] = []
        for item in values:
            flattened.extend(_flatten_numeric_list(item))
        return flattened
    return []


def _shape_matrix(matrix_like: Any, rows: int, cols: int) -> np.ndarray | None:
    values = _flatten_numeric_list(matrix_like)
    if len(values) != rows * cols:
        return None
    return np.asarray(values, dtype=np.float64).reshape(rows, cols)


def _find_first(mapping: dict[str, Any], keys: tuple[str, ...]) -> Any:
    for key in keys:
        if key in mapping:
            return mapping[key]
    return None


def load_intrinsics(path: Path) -> CameraIntrinsics:
    payload = json.loads(path.read_text(encoding="utf-8"))

    width = _find_first(payload, ("width", "image_width"))
    height = _find_first(payload, ("height", "image_height"))
    image_size = _find_first(payload, ("image_size", "ImageSize"))
    if image_size and (width is None or height is None):
        flat_size = _flatten_numeric_list(image_size)
        if len(flat_size) >= 2:
            if width is None:
                width = int(flat_size[1] if flat_size[0] > flat_size[1] else flat_size[0])
            if height is None:
                height = int(flat_size[0] if flat_size[0] > flat_size[1] else flat_size[1])

    fx = _find_first(payload, ("fx", "f_x"))
    fy = _find_first(payload, ("fy", "f_y"))
    cx = _find_first(payload, ("cx", "c_x"))
    cy = _find_first(payload, ("cy", "c_y"))

    if None in (fx, fy, cx, cy):
        camera_matrix = _shape_matrix(
            _find_first(payload, ("camera_matrix", "K", "intrinsic_matrix")),
            3,
            3,
        )
        if camera_matrix is None:
            raise ValueError(f"Could not parse camera intrinsics from {path}")
        fx = float(camera_matrix[0, 0])
        fy = float(camera_matrix[1, 1])
        cx = float(camera_matrix[0, 2])
        cy = float(camera_matrix[1, 2])

    if width is None or height is None:
        raise ValueError(f"Could not parse image width/height from {path}")

    distortion = _flatten_numeric_list(
        _find_first(
            payload,
            ("distortion", "dist_coeffs", "distortion_coefficients"),
        )
    )
    if not distortion:
        radial = _flatten_numeric_list(
            _find_first(payload, ("radial_distortion", "RadialDistortion"))
        )
        tangential = _flatten_numeric_list(
            _find_first(payload, ("tangential_distortion", "TangentialDistortion"))
        )
        distortion = [0.0] * 8
        if radial:
            distortion[0] = radial[0]
        if len(radial) > 1:
            distortion[1] = radial[1]
        if tangential:
            distortion[2] = tangential[0]
        if len(tangential) > 1:
            distortion[3] = tangential[1]
        if len(radial) > 2:
            distortion[4] = radial[2]
    distortion_array = np.asarray(distortion[:8], dtype=np.float64)

    return CameraIntrinsics(
        fx=float(fx),
        fy=float(fy),
        cx=float(cx),
        cy=float(cy),
        width=int(width),
        height=int(height),
        distortion=distortion_array,
    )


def load_extrinsics(path: Path) -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    matrix = _shape_matrix(_find_first(payload, ("T_lidar_cam", "matrix", "transform")), 4, 4)
    if matrix is not None:
        return matrix

    rotation = _shape_matrix(_find_first(payload, ("rotation", "R")), 3, 3)
    translation_values = _flatten_numeric_list(_find_first(payload, ("translation", "t")))
    if rotation is None or len(translation_values) < 3:
        raise ValueError(f"Could not parse LiDAR->camera extrinsics from {path}")

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation_values[:3], dtype=np.float64)
    return transform


def load_pose_records(path: Path, time_column: str) -> list[PoseRecord]:
    poses: list[PoseRecord] = []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            status_value = str(row.get("status", "OK")).strip().upper()
            if status_value and status_value != "OK":
                continue

            poses.append(
                PoseRecord(
                    timestamp=float(row[time_column]),
                    translation=np.asarray(
                        [float(row["x"]), float(row["y"]), float(row["z"])],
                        dtype=np.float64,
                    ),
                    quaternion_xyzw=np.asarray(
                        [float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])],
                        dtype=np.float64,
                    ),
                    raw=dict(row),
                )
            )
    if not poses:
        raise ValueError(f"No valid poses were loaded from {path}")
    poses.sort(key=lambda record: record.timestamp)
    return poses


def load_image_timestamps(path: Path, filename_column: str, time_column: str) -> dict[str, float]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        if not time_column:
            for candidate in ("t_query_sec", "timestamp", "time_sec", "t_in_sec", "t_sec_ms"):
                if candidate in fieldnames:
                    time_column = candidate
                    break
        if not time_column:
            raise ValueError(
                f"Could not infer an image timestamp column from {path}; available columns: {fieldnames}"
            )

        mapping: dict[str, float] = {}
        for row in reader:
            image_name = Path(row[filename_column]).name
            mapping[image_name] = float(row[time_column])
    if not mapping:
        raise ValueError(f"No image timestamps were loaded from {path}")
    return mapping


def build_frame_records(mask_dir: Path, meta_dir: Path, image_timestamps: dict[str, float]) -> list[FrameRecord]:
    frames: list[FrameRecord] = []
    for npz_path in sorted(mask_dir.glob("*_masks.npz")):
        stem = npz_path.name.removesuffix("_masks.npz")
        meta_path = meta_dir / f"{stem}_meta.json"
        if not meta_path.exists():
            print(f"[warn] skipping {stem}: missing metadata JSON")
            continue

        image_name = f"{stem}.jpg"
        if image_name not in image_timestamps:
            print(f"[warn] skipping {stem}: missing timestamp for {image_name}")
            continue

        frames.append(
            FrameRecord(
                stem=stem,
                image_name=image_name,
                timestamp=image_timestamps[image_name],
                npz_path=npz_path,
                meta_path=meta_path,
            )
        )
    if not frames:
        raise ValueError(f"No mask/metadata/timestamp triplets were found in {mask_dir}")
    frames.sort(key=lambda record: record.timestamp)
    return frames


def transform_from_pose(record: PoseRecord) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_quat(record.quaternion_xyzw).as_matrix()
    transform[:3, 3] = record.translation
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def match_nearest_pose_indices(frame_times: np.ndarray, pose_times: np.ndarray) -> np.ndarray:
    insertion = np.searchsorted(pose_times, frame_times)
    indices = np.clip(insertion, 0, pose_times.size - 1)
    previous = np.clip(indices - 1, 0, pose_times.size - 1)
    choose_previous = np.abs(frame_times - pose_times[previous]) <= np.abs(frame_times - pose_times[indices])
    return np.where(choose_previous, previous, indices)


def sanitize_masks(mask_array: np.ndarray) -> np.ndarray:
    masks = np.asarray(mask_array)
    if masks.ndim != 3:
        raise ValueError(f"Expected masks with shape (num_instances, H, W); got {masks.shape}")
    return np.ascontiguousarray(masks.astype(np.uint8))


def parse_class_lists(values: str) -> tuple[str, ...]:
    return tuple(sorted({item.strip().lower() for item in values.split(",") if item.strip()}))


def stable_hsv_color(name: str) -> np.ndarray:
    if name in CLASS_COLOR_TABLE:
        return CLASS_COLOR_TABLE[name]

    import colorsys

    hue = (abs(hash(name)) % 360) / 360.0
    rgb = colorsys.hsv_to_rgb(hue, 0.68, 0.95)
    return np.asarray([round(channel * 255) for channel in rgb], dtype=np.uint8)


def compute_adaptive_eps(points: np.ndarray, k_neighbors: int, eps_factor: float) -> float | None:
    if points.shape[0] <= 2:
        return None
    k_use = min(k_neighbors, points.shape[0] - 1)
    if k_use < 2:
        return None
    tree = cKDTree(points)
    distances, _ = tree.query(points, k=k_use + 1, workers=-1)
    kth_distances = distances[:, -1]
    kth_distances = kth_distances[np.isfinite(kth_distances)]
    if kth_distances.size == 0:
        return None
    eps = eps_factor * float(np.median(kth_distances))
    if not np.isfinite(eps) or eps <= 0.0:
        return None
    return eps


def instance_clustering_points(points: np.ndarray, class_name: str) -> np.ndarray:
    # Poles are vertically elongated, so clustering in XY is more stable than full 3D.
    if class_name == "pole":
        return points[:, :2]
    return points


def keep_largest_dense_cluster(
    points: np.ndarray,
    indices: np.ndarray,
    k_neighbors: int,
    eps_factor: float,
    min_cluster_points: int,
    min_keep_cluster: int,
) -> np.ndarray:
    if indices.size < min_keep_cluster:
        return indices

    subset = points[indices]
    eps = compute_adaptive_eps(subset, k_neighbors, eps_factor)
    if eps is None:
        return indices

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(subset)
    labels = np.asarray(
        point_cloud.cluster_dbscan(eps=eps, min_points=min_cluster_points, print_progress=False),
        dtype=np.int32,
    )
    positive = labels[labels >= 0]
    if positive.size == 0:
        return indices

    counts = np.bincount(positive)
    keep_label = int(np.argmax(counts))
    return indices[labels == keep_label]


def segment_class_instances(
    points: np.ndarray,
    point_indices: np.ndarray,
    best_confidences: np.ndarray,
    class_name: str,
    k_neighbors: int,
    eps_factor: float,
    min_cluster_points: int,
    stat_nb_neighbors: int,
    stat_std_ratio: float,
) -> list[dict[str, Any]]:
    if point_indices.size == 0:
        return []

    class_points = points[point_indices]
    clustering_points = instance_clustering_points(class_points, class_name)
    eps = compute_adaptive_eps(clustering_points, k_neighbors, eps_factor)

    if eps is None:
        clusters = [np.arange(point_indices.size, dtype=np.int32)]
    else:
        if clustering_points.shape[1] == 2:
            labels = np.asarray(
                cluster_dbscan_2d(
                    clustering_points,
                    eps=eps,
                    min_points=min_cluster_points,
                ),
                dtype=np.int32,
            )
        else:
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(class_points)
            labels = np.asarray(
                point_cloud.cluster_dbscan(eps=eps, min_points=min_cluster_points, print_progress=False),
                dtype=np.int32,
            )
        clusters = [np.where(labels == label)[0] for label in np.unique(labels) if label >= 0]
        if not clusters:
            clusters = [np.arange(point_indices.size, dtype=np.int32)]

    instances: list[dict[str, Any]] = []
    for cluster_local_indices in clusters:
        cluster_global_indices = point_indices[cluster_local_indices]
        cluster_points = points[cluster_global_indices]
        if cluster_global_indices.size < min_cluster_points:
            continue

        cluster_cloud = o3d.geometry.PointCloud()
        cluster_cloud.points = o3d.utility.Vector3dVector(cluster_points)
        _, inlier_indices = cluster_cloud.remove_statistical_outlier(
            nb_neighbors=max(2, min(stat_nb_neighbors, cluster_global_indices.size - 1)),
            std_ratio=stat_std_ratio,
        )
        inlier_indices = np.asarray(inlier_indices, dtype=np.int32)
        if inlier_indices.size == 0:
            continue

        inlier_global_indices = cluster_global_indices[inlier_indices]
        inlier_points = points[inlier_global_indices]
        if inlier_global_indices.size < min_cluster_points:
            continue

        confidences = best_confidences[inlier_global_indices]
        centroid = inlier_points.mean(axis=0)
        bbox_min = inlier_points.min(axis=0)
        bbox_max = inlier_points.max(axis=0)
        instances.append(
            {
                "class_name": class_name,
                "point_indices": inlier_global_indices,
                "mean_confidence": float(np.mean(confidences)),
                "num_points": int(inlier_global_indices.size),
                "centroid": centroid,
                "bbox_min": bbox_min,
                "bbox_max": bbox_max,
            }
        )

    if class_name == "pole" and len(instances) > 1:
        instances = merge_pole_fragments(points, best_confidences, instances)

    instances.sort(key=lambda item: (-item["num_points"], item["centroid"][0], item["centroid"][1], item["centroid"][2]))
    for idx, instance in enumerate(instances, start=1):
        instance["name"] = f"{class_name}_{idx:02d}"
        instance["instance_number"] = idx
    return instances


def cluster_dbscan_2d(points_xy: np.ndarray, eps: float, min_points: int) -> np.ndarray:
    if points_xy.shape[0] == 0:
        return np.empty(0, dtype=np.int32)

    tree = cKDTree(points_xy)
    neighbors = tree.query_ball_point(points_xy, r=eps, workers=-1)
    labels = np.full(points_xy.shape[0], -1, dtype=np.int32)
    visited = np.zeros(points_xy.shape[0], dtype=bool)
    cluster_id = 0

    for idx in range(points_xy.shape[0]):
        if visited[idx]:
            continue
        visited[idx] = True
        seed_neighbors = neighbors[idx]
        if len(seed_neighbors) < min_points:
            continue

        labels[idx] = cluster_id
        queue = list(seed_neighbors)
        queue_head = 0
        while queue_head < len(queue):
            neighbor_idx = queue[queue_head]
            queue_head += 1

            if not visited[neighbor_idx]:
                visited[neighbor_idx] = True
                neighbor_neighbors = neighbors[neighbor_idx]
                if len(neighbor_neighbors) >= min_points:
                    queue.extend(neighbor_neighbors)

            if labels[neighbor_idx] == -1:
                labels[neighbor_idx] = cluster_id

        cluster_id += 1

    return labels


def xy_boxes_overlap(instance_a: dict[str, Any], instance_b: dict[str, Any], margin: float = 0.35) -> bool:
    a_min = instance_a["bbox_min"][:2]
    a_max = instance_a["bbox_max"][:2]
    b_min = instance_b["bbox_min"][:2]
    b_max = instance_b["bbox_max"][:2]

    overlap_x = min(a_max[0], b_max[0]) + margin >= max(a_min[0], b_min[0])
    overlap_y = min(a_max[1], b_max[1]) + margin >= max(a_min[1], b_min[1])
    return bool(overlap_x and overlap_y)


def centroid_xy_distance(instance_a: dict[str, Any], instance_b: dict[str, Any]) -> float:
    delta = instance_a["centroid"][:2] - instance_b["centroid"][:2]
    return float(np.linalg.norm(delta))


def merge_instance_group(
    class_name: str,
    points: np.ndarray,
    best_confidences: np.ndarray,
    group: list[dict[str, Any]],
) -> dict[str, Any]:
    if len(group) == 1:
        return group[0]

    point_indices = np.unique(np.concatenate([item["point_indices"] for item in group]))
    merged_points = points[point_indices]
    confidences = best_confidences[point_indices]
    return {
        "class_name": class_name,
        "point_indices": point_indices,
        "mean_confidence": float(np.mean(confidences)),
        "num_points": int(point_indices.size),
        "centroid": merged_points.mean(axis=0),
        "bbox_min": merged_points.min(axis=0),
        "bbox_max": merged_points.max(axis=0),
    }


def merge_pole_fragments(
    points: np.ndarray,
    best_confidences: np.ndarray,
    instances: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    parent = list(range(len(instances)))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra = find(a)
        rb = find(b)
        if ra != rb:
            parent[rb] = ra

    for i in range(len(instances)):
        for j in range(i + 1, len(instances)):
            if xy_boxes_overlap(instances[i], instances[j]):
                union(i, j)

    grouped: dict[int, list[dict[str, Any]]] = {}
    for index, instance in enumerate(instances):
        grouped.setdefault(find(index), []).append(instance)

    merged: list[dict[str, Any]] = []
    for group in grouped.values():
        merged.append(merge_instance_group("pole", points, best_confidences, group))

    return merged


def merge_transformer_fragments(
    instances: list[dict[str, Any]],
    points: np.ndarray,
    best_confidences: np.ndarray,
    support_radius_xy: float = 1.9,
    merge_distance_xy: float = 1.9,
    overlap_margin_xy: float = 0.75,
) -> list[dict[str, Any]]:
    poles = [instance for instance in instances if instance["class_name"] == "pole"]
    transformers = [instance for instance in instances if instance["class_name"] == "transformer"]
    others = [instance for instance in instances if instance["class_name"] != "transformer"]

    if len(poles) == 0 or len(transformers) <= 1:
        return instances

    support_ids: list[int] = []
    for transformer in transformers:
        best_pole_index = -1
        best_distance = float("inf")
        for pole_index, pole in enumerate(poles):
            distance_xy = centroid_xy_distance(transformer, pole)
            if distance_xy < best_distance:
                best_distance = distance_xy
                best_pole_index = pole_index
        support_ids.append(best_pole_index if best_distance <= support_radius_xy else -1)

    parent = list(range(len(transformers)))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra = find(a)
        rb = find(b)
        if ra != rb:
            parent[rb] = ra

    for i in range(len(transformers)):
        for j in range(i + 1, len(transformers)):
            same_support = support_ids[i] != -1 and support_ids[i] == support_ids[j]
            if not same_support:
                continue

            close_xy = centroid_xy_distance(transformers[i], transformers[j]) <= merge_distance_xy
            overlap_xy = xy_boxes_overlap(transformers[i], transformers[j], margin=overlap_margin_xy)
            if close_xy or overlap_xy:
                union(i, j)

    groups: dict[int, list[dict[str, Any]]] = {}
    for index, transformer in enumerate(transformers):
        groups.setdefault(find(index), []).append(transformer)

    merged_transformers = [
        merge_instance_group("transformer", points, best_confidences, group)
        for group in groups.values()
    ]
    return others + merged_transformers


def assign_instance_names(instances: list[dict[str, Any]], class_names: list[str]) -> list[dict[str, Any]]:
    ordered: list[dict[str, Any]] = []
    for class_name in class_names:
        class_instances = [instance for instance in instances if instance["class_name"] == class_name]
        class_instances.sort(
            key=lambda item: (
                -item["num_points"],
                item["centroid"][0],
                item["centroid"][1],
                item["centroid"][2],
            )
        )
        for idx, instance in enumerate(class_instances, start=1):
            instance["name"] = f"{class_name}_{idx:02d}"
            instance["instance_number"] = idx
            ordered.append(instance)
    return ordered


def pairwise_horizontal_distances(points_xy: np.ndarray) -> np.ndarray:
    deltas = points_xy[:, None, :] - points_xy[None, :, :]
    distances = np.linalg.norm(deltas, axis=2)
    np.fill_diagonal(distances, np.inf)
    return distances


def compute_pole_neighbor_distances(
    instances: list[dict[str, Any]],
    min_spacing_m: float = 5.0,
    max_spacing_m: float = 90.0,
    top_k_neighbors: int = 2,
    adaptive_multiplier: float = 2.2,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    poles = [instance for instance in instances if instance["class_name"] == "pole"]
    if len(poles) < 2:
        return [], {
            "num_poles": len(poles),
            "min_spacing_m": min_spacing_m,
            "max_spacing_m": max_spacing_m,
            "adaptive_max_spacing_m": None,
        }

    centroids = np.asarray([pole["centroid"] for pole in poles], dtype=np.float64)
    centroids_xy = centroids[:, :2]
    distances_xy = pairwise_horizontal_distances(centroids_xy)
    nearest_distances = np.min(distances_xy, axis=1)
    nearest_distances = nearest_distances[np.isfinite(nearest_distances)]

    adaptive_max = max_spacing_m
    if nearest_distances.size > 0:
        adaptive_guess = float(np.median(nearest_distances) * adaptive_multiplier)
        adaptive_max = min(max_spacing_m, max(min_spacing_m * 2.0, adaptive_guess))

    top_k = min(top_k_neighbors, len(poles) - 1)
    neighbor_order = np.argsort(distances_xy, axis=1)
    top_neighbor_sets = [set(order[:top_k]) for order in neighbor_order]

    candidates: list[tuple[float, int, int]] = []
    for i in range(len(poles)):
        for j in range(i + 1, len(poles)):
            distance_xy = float(distances_xy[i, j])
            if distance_xy < min_spacing_m or distance_xy > adaptive_max:
                continue

            if j not in top_neighbor_sets[i] and i not in top_neighbor_sets[j]:
                continue

            candidates.append((distance_xy, i, j))

    parent = list(range(len(poles)))
    degree = [0] * len(poles)

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra = find(a)
        rb = find(b)
        if ra != rb:
            parent[rb] = ra

    selected_edges: list[dict[str, Any]] = []
    for distance_xy, i, j in sorted(candidates, key=lambda item: item[0]):
        if degree[i] >= 2 or degree[j] >= 2:
            continue
        if find(i) == find(j):
            continue

        pole_a = poles[i]
        pole_b = poles[j]
        centroid_a = np.asarray(pole_a["centroid"], dtype=np.float64)
        centroid_b = np.asarray(pole_b["centroid"], dtype=np.float64)
        delta = centroid_b - centroid_a
        midpoint = (centroid_a + centroid_b) / 2.0
        distance_3d = float(np.linalg.norm(delta))
        delta_z = float(abs(delta[2]))

        selected_edges.append(
            {
                "from_object_name": pole_a["name"],
                "to_object_name": pole_b["name"],
                "from_centroid_map_xyz": centroid_a.tolist(),
                "to_centroid_map_xyz": centroid_b.tolist(),
                "midpoint_map_xyz": midpoint.tolist(),
                "horizontal_distance_m": distance_xy,
                "delta_z_m": delta_z,
                "distance_3d_m": distance_3d,
            }
        )
        degree[i] += 1
        degree[j] += 1
        union(i, j)

    metadata = {
        "num_poles": len(poles),
        "min_spacing_m": min_spacing_m,
        "max_spacing_m": max_spacing_m,
        "adaptive_max_spacing_m": adaptive_max,
        "num_distance_links": len(selected_edges),
    }
    return selected_edges, metadata


def build_pole_distance_geometry(
    pole_distances: list[dict[str, Any]],
    line_color_rgb: tuple[float, float, float] = (0.16, 0.40, 0.82),
) -> tuple[o3d.geometry.LineSet | None, list[tuple[np.ndarray, str]]]:
    if not pole_distances:
        return None, []

    line_points: list[list[float]] = []
    lines: list[list[int]] = []
    colors: list[list[float]] = []
    labels: list[tuple[np.ndarray, str]] = []

    for edge in pole_distances:
        start = np.asarray(edge["from_centroid_map_xyz"], dtype=np.float64)
        end = np.asarray(edge["to_centroid_map_xyz"], dtype=np.float64)
        midpoint = np.asarray(edge["midpoint_map_xyz"], dtype=np.float64)

        start_idx = len(line_points)
        line_points.append(start.tolist())
        line_points.append(end.tolist())
        lines.append([start_idx, start_idx + 1])
        colors.append(list(line_color_rgb))
        labels.append((midpoint, f'{edge["horizontal_distance_m"]:.1f} m'))

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.asarray(line_points, dtype=np.float64))
    line_set.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    line_set.colors = o3d.utility.Vector3dVector(np.asarray(colors, dtype=np.float64))
    return line_set, labels


def make_frame_axes_geometry(camera_transform: np.ndarray, axis_length: float) -> o3d.geometry.TriangleMesh:
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_length)
    frame.transform(camera_transform)
    return frame


def visualize_scene(
    point_cloud: o3d.geometry.PointCloud,
    camera_geometries: list[o3d.geometry.Geometry],
    instances: list[dict[str, Any]],
    extra_geometries: list[o3d.geometry.Geometry] | None = None,
    extra_labels: list[tuple[np.ndarray, str]] | None = None,
) -> None:
    try:
        app = o3d.visualization.gui.Application.instance
        app.initialize()
        visualizer = o3d.visualization.O3DVisualizer("Fusion Results", 1600, 900)
        visualizer.show_skybox(False)
        visualizer.add_geometry("fused_cloud", point_cloud)
        for index, geometry in enumerate(camera_geometries):
            visualizer.add_geometry(f"camera_{index:03d}", geometry)
        if extra_geometries:
            for index, geometry in enumerate(extra_geometries):
                visualizer.add_geometry(f"extra_{index:03d}", geometry)
        for instance in instances:
            label = f'{instance["name"]} ({instance["mean_confidence"]:.2f})'
            visualizer.add_3d_label(instance["centroid"], label)
        if extra_labels:
            for position, label in extra_labels:
                visualizer.add_3d_label(position, label)
        visualizer.reset_camera_to_default()
        app.add_window(visualizer)
        app.run()
    except Exception as exc:
        print(f"[warn] O3DVisualizer failed ({exc}); falling back to draw_geometries.")
        geometries: list[o3d.geometry.Geometry] = [point_cloud]
        geometries.extend(camera_geometries)
        if extra_geometries:
            geometries.extend(extra_geometries)
        o3d.visualization.draw_geometries(geometries, window_name="Fusion Results")


def save_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def main() -> int:
    args = parse_args()
    ensure_built()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    intrinsics = load_intrinsics(Path(args.intrinsics_json))
    lidar_to_camera = load_extrinsics(Path(args.extrinsics_json))
    poses = load_pose_records(Path(args.pose_csv), args.time_column)
    image_timestamps = load_image_timestamps(
        Path(args.image_timestamps_csv),
        args.image_filename_column,
        args.image_time_column,
    )
    frames = build_frame_records(Path(args.mask_dir), Path(args.meta_dir), image_timestamps)

    frame_times = np.asarray([frame.timestamp + args.time_offset_sec for frame in frames], dtype=np.float64)
    pose_times = np.asarray([pose.timestamp for pose in poses], dtype=np.float64)
    pose_matches = match_nearest_pose_indices(frame_times, pose_times)

    point_cloud = o3d.io.read_point_cloud(str(Path(args.point_cloud)))
    if point_cloud.is_empty():
        raise ValueError(f"Point cloud is empty: {args.point_cloud}")
    points_map = np.asarray(point_cloud.points, dtype=np.float64)
    num_points = points_map.shape[0]
    print(f"Loaded {num_points:,} map points.")

    allowed_classes = parse_class_lists(args.allowed_classes)
    reject_classes = parse_class_lists(args.reject_classes)

    class_name_to_index: dict[str, int] = {}
    class_names: list[str] = []

    best_conf = np.zeros(num_points, dtype=np.float64)
    vote_count = np.zeros(num_points, dtype=np.uint16)
    winner_class = np.full(num_points, -1, dtype=np.int32)
    camera_geometries: list[o3d.geometry.Geometry] = []
    used_frames: list[dict[str, Any]] = []

    camera_to_lidar = invert_transform(lidar_to_camera)

    for frame_index, frame in enumerate(frames):
        pose = poses[int(pose_matches[frame_index])]
        lidar_to_map = transform_from_pose(pose)
        map_to_lidar = invert_transform(lidar_to_map)
        camera_to_map = lidar_to_map @ camera_to_lidar
        camera_geometries.append(make_frame_axes_geometry(camera_to_map, args.axis_length))

        with np.load(frame.npz_path, allow_pickle=False) as npz:
            masks = sanitize_masks(npz["masks"])

        if masks.shape[1] != intrinsics.height or masks.shape[2] != intrinsics.width:
            print(
                f"[warn] {frame.stem}: mask size {masks.shape[1]}x{masks.shape[2]} "
                f"does not match intrinsics {intrinsics.height}x{intrinsics.width}"
            )

        meta = json.loads(frame.meta_path.read_text(encoding="utf-8"))
        detections = meta.get("detections", [])
        num_instances = masks.shape[0]
        confidences = np.zeros(num_instances, dtype=np.float64)
        allowed = np.zeros(num_instances, dtype=np.uint8)
        detection_classes = np.full(num_instances, -1, dtype=np.int32)

        for detection in detections:
            detection_index = int(detection["instance_index"])
            if not 0 <= detection_index < num_instances:
                continue
            class_name = str(detection["class_name"]).strip().lower()
            confidence = float(detection["confidence"])

            if class_name not in class_name_to_index:
                class_name_to_index[class_name] = len(class_names)
                class_names.append(class_name)

            class_index = class_name_to_index[class_name]
            confidences[detection_index] = confidence
            detection_classes[detection_index] = class_index
            is_allowed = class_name in allowed_classes and class_name not in reject_classes
            allowed[detection_index] = 1 if is_allowed else 0

        if not np.any(allowed):
            print(f"[info] {frame.stem}: no allowed detections")
            continue

        detection_index, detection_confidence = project_assign_best_detection(
            points_map=points_map,
            map_to_lidar=map_to_lidar,
            lidar_to_cam=lidar_to_camera,
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            cx=intrinsics.cx,
            cy=intrinsics.cy,
            width=intrinsics.width,
            height=intrinsics.height,
            distortion=intrinsics.distortion,
            masks=masks,
            confidences=confidences,
            allowed=allowed,
        )

        labeled_points = detection_index >= 0
        if not np.any(labeled_points):
            print(f"[info] {frame.stem}: no map points landed inside allowed masks")
            continue

        for det_id in np.unique(detection_index[labeled_points]):
            det_points = np.where(detection_index == det_id)[0]
            keep_points = keep_largest_dense_cluster(
                points=points_map,
                indices=det_points,
                k_neighbors=args.knn_cluster,
                eps_factor=args.eps_factor,
                min_cluster_points=args.min_cluster_points,
                min_keep_cluster=args.min_keep_cluster,
            )
            if keep_points.size == det_points.size:
                continue
            drop_points = np.setdiff1d(det_points, keep_points, assume_unique=False)
            detection_index[drop_points] = -1
            detection_confidence[drop_points] = 0.0

        labeled_points = np.where(detection_index >= 0)[0]
        if labeled_points.size == 0:
            print(f"[info] {frame.stem}: dense cluster filtering removed all labels")
            continue

        vote_count[labeled_points] = np.minimum(
            vote_count[labeled_points].astype(np.uint32) + 1,
            np.iinfo(np.uint16).max,
        ).astype(np.uint16)

        local_class_indices = detection_classes[detection_index[labeled_points]]
        better = detection_confidence[labeled_points] > best_conf[labeled_points]
        better_points = labeled_points[better]
        winner_class[better_points] = local_class_indices[better]
        best_conf[better_points] = detection_confidence[better_points]

        used_frames.append(
            {
                "frame": frame.image_name,
                "frame_timestamp": frame.timestamp,
                "matched_pose_timestamp": pose.timestamp,
                "matched_pose_index": int(pose_matches[frame_index]),
                "num_labeled_points": int(labeled_points.size),
            }
        )
        print(
            f"[info] {frame.image_name}: matched pose {pose.timestamp:.6f}, "
            f"kept {labeled_points.size:,} projected points"
        )

    if args.min_vote_to_keep > 1:
        drop_mask = vote_count < args.min_vote_to_keep
        best_conf[drop_mask] = 0.0
        winner_class[drop_mask] = -1

    instances: list[dict[str, Any]] = []

    for class_index, class_name in enumerate(class_names):
        class_points = np.where(winner_class == class_index)[0]
        segmented = segment_class_instances(
            points=points_map,
            point_indices=class_points,
            best_confidences=best_conf,
            class_name=class_name,
            k_neighbors=args.knn_cluster,
            eps_factor=args.eps_factor,
            min_cluster_points=args.min_cluster_points,
            stat_nb_neighbors=args.stat_nb_neighbors,
            stat_std_ratio=args.stat_std_ratio,
        )
        for instance in segmented:
            instances.append(instance)

    instances = merge_transformer_fragments(instances, points_map, best_conf)
    instances = assign_instance_names(instances, class_names)

    pole_distances, pole_distance_metadata = compute_pole_neighbor_distances(
        instances=instances,
        min_spacing_m=args.min_pole_spacing_m,
        max_spacing_m=args.max_pole_spacing_m,
        top_k_neighbors=args.pole_neighbor_top_k,
        adaptive_multiplier=args.pole_spacing_adaptive_multiplier,
    )
    pole_distance_geometry, pole_distance_labels = build_pole_distance_geometry(pole_distances)

    final_instance_index = np.full(num_points, -1, dtype=np.int32)
    final_class_index = np.full(num_points, -1, dtype=np.int32)
    for instance_number, instance in enumerate(instances):
        point_indices = instance["point_indices"]
        class_index = class_name_to_index[instance["class_name"]]
        final_instance_index[point_indices] = instance_number
        final_class_index[point_indices] = class_index
        instance["class_color_rgb"] = stable_hsv_color(instance["class_name"]).tolist()

    colors = np.tile(BACKGROUND_COLOR, (num_points, 1))
    for instance in instances:
        point_indices = instance["point_indices"]
        colors[point_indices] = np.asarray(instance["class_color_rgb"], dtype=np.uint8)

    point_cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255.0)

    fused_ply_path = output_dir / "fused_semantic_map.ply"
    if not o3d.io.write_point_cloud(str(fused_ply_path), point_cloud):
        raise RuntimeError(f"Failed to write point cloud to {fused_ply_path}")

    labels_npz_path = output_dir / "fused_semantic_labels.npz"
    np.savez_compressed(
        labels_npz_path,
        class_names=np.asarray(class_names, dtype=object),
        winner_class=winner_class,
        final_class_index=final_class_index,
        final_instance_index=final_instance_index,
        best_conf=best_conf,
        vote_count=vote_count,
    )

    combined_objects: list[dict[str, Any]] = []
    for instance in instances:
        combined_objects.append(
            {
                "object_name": instance["name"],
                "class_name": instance["class_name"],
                "instance_number": int(instance["instance_number"]),
                "confidence_score": float(instance["mean_confidence"]),
                "num_points": int(instance["num_points"]),
                "centroid_map_xyz": [float(value) for value in instance["centroid"]],
                "bbox_aabb_min_xyz": [float(value) for value in instance["bbox_min"]],
                "bbox_aabb_max_xyz": [float(value) for value in instance["bbox_max"]],
                "class_color_rgb": instance["class_color_rgb"],
                "gps": {"lat": None, "lon": None, "alt": None},
            }
        )

    combined_json = {
        "pipeline": "fuse_masks_to_slam",
        "inputs": {
            "intrinsics_json": str(Path(args.intrinsics_json).resolve()),
            "extrinsics_json": str(Path(args.extrinsics_json).resolve()),
            "pose_csv": str(Path(args.pose_csv).resolve()),
            "image_timestamps_csv": str(Path(args.image_timestamps_csv).resolve()),
            "point_cloud": str(Path(args.point_cloud).resolve()),
            "mask_dir": str(Path(args.mask_dir).resolve()),
            "meta_dir": str(Path(args.meta_dir).resolve()),
        },
        "settings": {
            "time_column": args.time_column,
            "time_offset_sec": args.time_offset_sec,
            "allowed_classes": list(allowed_classes),
            "reject_classes": list(reject_classes),
            "min_vote_to_keep": args.min_vote_to_keep,
            "knn_cluster": args.knn_cluster,
            "eps_factor": args.eps_factor,
            "min_cluster_points": args.min_cluster_points,
            "min_keep_cluster": args.min_keep_cluster,
            "stat_nb_neighbors": args.stat_nb_neighbors,
            "stat_std_ratio": args.stat_std_ratio,
            "min_pole_spacing_m": args.min_pole_spacing_m,
            "max_pole_spacing_m": args.max_pole_spacing_m,
            "pole_neighbor_top_k": args.pole_neighbor_top_k,
            "pole_spacing_adaptive_multiplier": args.pole_spacing_adaptive_multiplier,
        },
        "summary": {
            "num_input_points": int(num_points),
            "num_frames_used": len(used_frames),
            "num_objects": len(combined_objects),
            "num_pole_distance_links": len(pole_distances),
        },
        "frame_matches": used_frames,
        "objects": combined_objects,
    }

    combined_json_path = output_dir / "fused_objects.json"
    save_json(combined_json_path, combined_json)

    pole_distances_json = {
        "pipeline": "fuse_masks_to_slam",
        "source_objects_json": str(combined_json_path.resolve()),
        "metadata": pole_distance_metadata,
        "distances": pole_distances,
    }
    pole_distances_path = output_dir / "pole_neighbor_distances.json"
    save_json(pole_distances_path, pole_distances_json)

    print(f"[done] wrote point cloud to {fused_ply_path}")
    print(f"[done] wrote labels to {labels_npz_path}")
    print(f"[done] wrote object JSON to {combined_json_path}")
    print(f"[done] wrote pole distances to {pole_distances_path}")

    if not args.no_visualize:
        extra_geometries = [pole_distance_geometry] if pole_distance_geometry is not None else []
        visualize_scene(
            point_cloud,
            camera_geometries,
            instances,
            extra_geometries=extra_geometries,
            extra_labels=pole_distance_labels,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
