from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

try:
    from native import geospatial_accel
except ModuleNotFoundError:  # pragma: no cover - allows module-style imports during tests
    from Fusion.native import geospatial_accel


FUSION_ROOT = Path(__file__).resolve().parent
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
WGS84_B = WGS84_A * (1.0 - WGS84_F)
WGS84_EP2 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B)


@dataclass(frozen=True)
class GpsPoseRecord:
    t_in_sec: float
    t_query_sec: float
    translation_xyz: np.ndarray
    quaternion_xyzw: np.ndarray
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    fix_status: int | None
    service: int | None
    cov_xx_m2: float | None
    cov_yy_m2: float | None
    cov_zz_m2: float | None
    covariance_type: int | None
    raw: dict[str, Any]


@dataclass(frozen=True)
class AlignmentResult:
    scale: float
    yaw_rad: float
    translation_enu_m: np.ndarray
    reference_llh_deg_m: np.ndarray
    reference_ecef_m: np.ndarray
    inlier_mask: np.ndarray
    weights: np.ndarray
    residual_xy_m: np.ndarray
    residual_z_m: np.ndarray
    rmse_xy_m: float
    rmse_z_m: float


@dataclass(frozen=True)
class DenseTranslationTrajectory:
    timestamps: np.ndarray
    translations: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit a local-to-GPS transform from tf_gps_out.csv and optionally georeference "
            "Fusion objects or powerline overlay JSON outputs."
        )
    )
    parser.add_argument("--tf-gps-csv", required=True)
    parser.add_argument("--objects-json", default="")
    parser.add_argument("--powerlines-json", default="")
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--transform-output-json", default="")
    parser.add_argument("--objects-output-json", default="")
    parser.add_argument("--powerlines-output-json", default="")
    parser.add_argument("--aligned-csv-output", default="")
    parser.add_argument("--gps-to-lidar-offset-body", default="0,0,0")
    parser.add_argument("--gps-to-lidar-offset-json", default="")
    parser.add_argument("--min-fix-status", type=int, default=0)
    parser.add_argument("--max-horizontal-cov-m2", type=float, default=1000.0)
    parser.add_argument("--outlier-threshold-m", type=float, default=5.0)
    parser.add_argument("--allow-scale", action="store_true")
    parser.add_argument("--gps-time-offset-sec", type=float, default=0.0)
    parser.add_argument("--dense-traj-csv", default="")
    parser.add_argument("--native-mode", choices=("auto", "on", "off"), default="auto")
    return parser.parse_args()


def parse_vector3(text: str) -> np.ndarray:
    parts = [item.strip() for item in text.split(",")]
    if len(parts) != 3:
        raise ValueError(f"Expected three comma-separated values, got: {text}")
    return np.asarray([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float64)


def load_offset(args: argparse.Namespace) -> np.ndarray:
    if args.gps_to_lidar_offset_json:
        payload = json.loads(Path(args.gps_to_lidar_offset_json).read_text(encoding="utf-8"))
        values = payload.get("gps_to_lidar_xyz_m")
        if not isinstance(values, list) or len(values) != 3:
            raise ValueError(f"Expected gps_to_lidar_xyz_m in {args.gps_to_lidar_offset_json}")
        return np.asarray(values, dtype=np.float64)
    return parse_vector3(args.gps_to_lidar_offset_body)


def _parse_optional_int(value: Any) -> int | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return int(float(text))
    except ValueError:
        return None


def _parse_optional_float(value: Any) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        out = float(text)
    except ValueError:
        return None
    return out if math.isfinite(out) else None


def load_tf_gps_records(path: Path) -> list[GpsPoseRecord]:
    records: list[GpsPoseRecord] = []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
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

            records.append(
                GpsPoseRecord(
                    t_in_sec=float(row["t_in_sec"]),
                    t_query_sec=float(row["t_query_sec"]),
                    translation_xyz=np.asarray(
                        [float(row["x"]), float(row["y"]), float(row["z"])],
                        dtype=np.float64,
                    ),
                    quaternion_xyzw=np.asarray(
                        [float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])],
                        dtype=np.float64,
                    ),
                    latitude_deg=lat,
                    longitude_deg=lon,
                    altitude_m=alt,
                    fix_status=_parse_optional_int(row.get("fix_status")),
                    service=_parse_optional_int(row.get("service")),
                    cov_xx_m2=_parse_optional_float(row.get("cov_xx_m2")),
                    cov_yy_m2=_parse_optional_float(row.get("cov_yy_m2")),
                    cov_zz_m2=_parse_optional_float(row.get("cov_zz_m2")),
                    covariance_type=_parse_optional_int(row.get("covariance_type")),
                    raw=dict(row),
                )
            )
    if not records:
        raise ValueError(f"No valid GPS/TF samples loaded from {path}")
    return records


def load_dense_translation_trajectory(path: Path) -> DenseTranslationTrajectory:
    timestamps: list[float] = []
    translations: list[list[float]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            status = str(row.get("status", "")).strip().upper()
            if status != "OK":
                continue
            try:
                timestamp = float(row["timestamp_sec"])
                x = float(row["x"])
                y = float(row["y"])
                z = float(row["z"])
            except (KeyError, ValueError):
                continue
            if not all(math.isfinite(value) for value in (timestamp, x, y, z)):
                continue
            timestamps.append(timestamp)
            translations.append([x, y, z])
    if not timestamps:
        raise ValueError(f"No valid dense trajectory samples loaded from {path}")

    order = np.argsort(np.asarray(timestamps, dtype=np.float64), kind="stable")
    ts_sorted = np.asarray([timestamps[int(index)] for index in order], dtype=np.float64)
    tr_sorted = np.asarray([translations[int(index)] for index in order], dtype=np.float64)
    unique_mask = np.ones(ts_sorted.shape[0], dtype=bool)
    unique_mask[1:] = ts_sorted[1:] != ts_sorted[:-1]
    return DenseTranslationTrajectory(
        timestamps=ts_sorted[unique_mask],
        translations=tr_sorted[unique_mask],
    )


def interpolate_dense_translation(
    traj: DenseTranslationTrajectory,
    t_query_sec: float,
) -> np.ndarray | None:
    if traj.timestamps.size == 0:
        return None
    if t_query_sec < float(traj.timestamps[0]) or t_query_sec > float(traj.timestamps[-1]):
        return None

    right = int(np.searchsorted(traj.timestamps, t_query_sec, side="left"))
    if right <= 0:
        return traj.translations[0].copy()
    if right >= traj.timestamps.size:
        return traj.translations[-1].copy()

    left = right - 1
    t0 = float(traj.timestamps[left])
    t1 = float(traj.timestamps[right])
    if t1 <= t0:
        return None
    alpha = float((t_query_sec - t0) / (t1 - t0))
    alpha = max(0.0, min(1.0, alpha))
    return ((1.0 - alpha) * traj.translations[left] + alpha * traj.translations[right]).astype(np.float64)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return np.asarray([x, y, z], dtype=np.float64)


def ecef_to_geodetic(ecef_xyz: np.ndarray) -> np.ndarray:
    x, y, z = [float(value) for value in ecef_xyz]
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    theta = math.atan2(z * WGS84_A, p * WGS84_B)
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    lat = math.atan2(
        z + WGS84_EP2 * WGS84_B * sin_theta * sin_theta * sin_theta,
        p - WGS84_E2 * WGS84_A * cos_theta * cos_theta * cos_theta,
    )
    sin_lat = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    alt = p / math.cos(lat) - n
    return np.asarray([math.degrees(lat), math.degrees(lon), alt], dtype=np.float64)


def ecef_to_enu(ecef_xyz: np.ndarray, ref_llh: np.ndarray, ref_ecef: np.ndarray) -> np.ndarray:
    lat = math.radians(float(ref_llh[0]))
    lon = math.radians(float(ref_llh[1]))
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    rot = np.asarray(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=np.float64,
    )
    return rot @ (ecef_xyz - ref_ecef)


def enu_to_ecef(enu_xyz: np.ndarray, ref_llh: np.ndarray, ref_ecef: np.ndarray) -> np.ndarray:
    lat = math.radians(float(ref_llh[0]))
    lon = math.radians(float(ref_llh[1]))
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    rot_t = np.asarray(
        [
            [-sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon],
            [cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon],
            [0.0, cos_lat, sin_lat],
        ],
        dtype=np.float64,
    )
    return ref_ecef + rot_t @ enu_xyz


def enu_to_geodetic(enu_xyz: np.ndarray, ref_llh: np.ndarray, ref_ecef: np.ndarray) -> np.ndarray:
    return ecef_to_geodetic(enu_to_ecef(enu_xyz, ref_llh, ref_ecef))


def compute_weights(records: list[GpsPoseRecord]) -> np.ndarray:
    weights = np.ones(len(records), dtype=np.float64)
    for index, record in enumerate(records):
        if record.cov_xx_m2 is None or record.cov_yy_m2 is None:
            continue
        horiz_var = max(0.01, 0.5 * (record.cov_xx_m2 + record.cov_yy_m2))
        weights[index] = 1.0 / horiz_var
    return weights


def filter_records(records: list[GpsPoseRecord], *, min_fix_status: int, max_horizontal_cov_m2: float) -> list[GpsPoseRecord]:
    filtered = []
    for record in records:
        if record.fix_status is not None and record.fix_status < min_fix_status:
            continue
        if record.cov_xx_m2 is not None and record.cov_yy_m2 is not None:
            horiz_var = 0.5 * (record.cov_xx_m2 + record.cov_yy_m2)
            if math.isfinite(horiz_var) and horiz_var > max_horizontal_cov_m2:
                continue
        filtered.append(record)
    if len(filtered) < 3:
        raise ValueError(
            f"Need at least 3 valid GPS/TF rows after filtering "
            f"(kept {len(filtered)} of {len(records)}; "
            f"min_fix_status={min_fix_status}, max_horizontal_cov_m2={max_horizontal_cov_m2})"
        )
    return filtered


def local_gps_positions(
    records: list[GpsPoseRecord],
    gps_to_lidar_offset_body: np.ndarray,
    *,
    local_translations_xyz: np.ndarray | None = None,
) -> np.ndarray:
    positions = []
    for index, record in enumerate(records):
        rot = Rotation.from_quat(record.quaternion_xyzw).as_matrix()
        translation_xyz = (
            record.translation_xyz
            if local_translations_xyz is None
            else np.asarray(local_translations_xyz[index], dtype=np.float64)
        )
        positions.append(translation_xyz - rot @ gps_to_lidar_offset_body)
    return np.asarray(positions, dtype=np.float64)


def prepare_georeference_inputs(
    records: list[GpsPoseRecord],
    *,
    gps_to_lidar_offset_body: np.ndarray,
    gps_time_offset_sec: float,
    dense_traj_csv: str,
) -> tuple[list[GpsPoseRecord], np.ndarray]:
    use_gps_offset_compensation = abs(float(gps_time_offset_sec)) > 1e-12 and bool(str(dense_traj_csv).strip())
    if not use_gps_offset_compensation:
        reason = "zero offset" if abs(float(gps_time_offset_sec)) <= 1e-12 else "no dense trajectory csv provided"
        print(f"[gps-offset] inactive: {reason}; using original tf_gps local translations for {len(records)} rows")
        return records, local_gps_positions(records, gps_to_lidar_offset_body)

    dense_path = Path(dense_traj_csv)
    if not dense_path.is_file():
        print(
            f"[gps-offset] inactive: dense trajectory file not found ({dense_path}); "
            f"using original tf_gps local translations for {len(records)} rows"
        )
        return records, local_gps_positions(records, gps_to_lidar_offset_body)

    dense_traj = load_dense_translation_trajectory(dense_path)
    kept_records: list[GpsPoseRecord] = []
    compensated_translations: list[np.ndarray] = []
    dropped_rows = 0
    for record in records:
        t_effective = float(record.t_query_sec) + float(gps_time_offset_sec)
        translation_xyz = interpolate_dense_translation(dense_traj, t_effective)
        if translation_xyz is None:
            dropped_rows += 1
            continue
        kept_records.append(record)
        compensated_translations.append(translation_xyz)

    print(
        f"[gps-offset] active: offset={gps_time_offset_sec:.6f} sec, dense={dense_path.name}, "
        f"used {len(kept_records)}/{len(records)} rows, dropped {dropped_rows}"
    )
    if len(kept_records) < 3:
        raise ValueError(
            "GPS offset compensation left fewer than 3 usable rows after dense-trajectory interpolation "
            f"(kept {len(kept_records)} of {len(records)})"
        )
    return kept_records, local_gps_positions(
        kept_records,
        gps_to_lidar_offset_body,
        local_translations_xyz=np.asarray(compensated_translations, dtype=np.float64),
    )


def weighted_rigid_2d(
    source_xy: np.ndarray,
    target_xy: np.ndarray,
    weights: np.ndarray,
    *,
    allow_scale: bool,
) -> tuple[float, np.ndarray, np.ndarray]:
    w = np.maximum(np.asarray(weights, dtype=np.float64).reshape(-1), 1e-12)
    w /= np.sum(w)
    src_centroid = np.sum(source_xy * w[:, None], axis=0)
    tgt_centroid = np.sum(target_xy * w[:, None], axis=0)
    src_centered = source_xy - src_centroid
    tgt_centered = target_xy - tgt_centroid
    covariance = (w[:, None] * src_centered).T @ tgt_centered
    u, singular_values, vt = np.linalg.svd(covariance)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0.0:
        vt[-1, :] *= -1.0
        rot = vt.T @ u.T
    if allow_scale:
        denom = np.sum(w * np.sum(src_centered * src_centered, axis=1))
        scale = float(np.sum(singular_values) / max(denom, 1e-12))
    else:
        scale = 1.0
    translation = tgt_centroid - scale * (rot @ src_centroid)
    return scale, rot, translation


def apply_similarity_numpy(points_xyz: np.ndarray, *, scale: float, yaw_rad: float, translation_enu_m: np.ndarray) -> np.ndarray:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    out = np.empty_like(points_xyz, dtype=np.float64)
    out[:, 0] = scale * (cos_yaw * points_xyz[:, 0] - sin_yaw * points_xyz[:, 1]) + translation_enu_m[0]
    out[:, 1] = scale * (sin_yaw * points_xyz[:, 0] + cos_yaw * points_xyz[:, 1]) + translation_enu_m[1]
    out[:, 2] = points_xyz[:, 2] + translation_enu_m[2]
    return out


def apply_similarity(points_xyz: np.ndarray, *, scale: float, yaw_rad: float, translation_enu_m: np.ndarray, native_mode: str) -> np.ndarray:
    points_xyz = np.ascontiguousarray(points_xyz, dtype=np.float64)
    if native_mode != "off":
        try:
            if native_mode == "on":
                geospatial_accel.ensure_built()
            elif native_mode == "auto" and not (Path(geospatial_accel._dll_path()).exists()):  # type: ignore[attr-defined]
                raise FileNotFoundError
            return geospatial_accel.apply_similarity_xyz(
                points_xyz,
                scale=scale,
                cos_yaw=math.cos(yaw_rad),
                sin_yaw=math.sin(yaw_rad),
                tx=float(translation_enu_m[0]),
                ty=float(translation_enu_m[1]),
                tz=float(translation_enu_m[2]),
            )
        except Exception:
            if native_mode == "on":
                raise
    return apply_similarity_numpy(
        points_xyz,
        scale=scale,
        yaw_rad=yaw_rad,
        translation_enu_m=translation_enu_m,
    )


def fit_alignment(
    records: list[GpsPoseRecord],
    *,
    gps_to_lidar_offset_body: np.ndarray,
    local_points: np.ndarray | None = None,
    allow_scale: bool,
    outlier_threshold_m: float,
) -> AlignmentResult:
    reference_llh = np.asarray(
        [records[0].latitude_deg, records[0].longitude_deg, records[0].altitude_m],
        dtype=np.float64,
    )
    reference_ecef = geodetic_to_ecef(*reference_llh)
    enu_points = np.asarray(
        [
            ecef_to_enu(
                geodetic_to_ecef(record.latitude_deg, record.longitude_deg, record.altitude_m),
                reference_llh,
                reference_ecef,
            )
            for record in records
        ],
        dtype=np.float64,
    )
    if local_points is None:
        local_points = local_gps_positions(records, gps_to_lidar_offset_body)
    weights = compute_weights(records)
    inlier_mask = np.ones(len(records), dtype=bool)

    for _ in range(5):
        if int(np.count_nonzero(inlier_mask)) < 3:
            break
        scale, rot_2d, trans_xy = weighted_rigid_2d(
            local_points[inlier_mask, :2],
            enu_points[inlier_mask, :2],
            weights[inlier_mask],
            allow_scale=allow_scale,
        )
        yaw_rad = math.atan2(rot_2d[1, 0], rot_2d[0, 0])
        transformed = apply_similarity_numpy(
            local_points,
            scale=scale,
            yaw_rad=yaw_rad,
            translation_enu_m=np.asarray([trans_xy[0], trans_xy[1], 0.0], dtype=np.float64),
        )
        residual_xy = np.linalg.norm(transformed[:, :2] - enu_points[:, :2], axis=1)
        median = float(np.median(residual_xy[inlier_mask]))
        mad = float(np.median(np.abs(residual_xy[inlier_mask] - median)))
        threshold = max(outlier_threshold_m, median + 3.0 * max(mad, 0.25))
        new_mask = residual_xy <= threshold
        if np.array_equal(new_mask, inlier_mask):
            inlier_mask = new_mask
            break
        inlier_mask = new_mask

    if int(np.count_nonzero(inlier_mask)) < 3:
        raise ValueError("GPS alignment failed: not enough inliers after robust filtering")

    scale, rot_2d, trans_xy = weighted_rigid_2d(
        local_points[inlier_mask, :2],
        enu_points[inlier_mask, :2],
        weights[inlier_mask],
        allow_scale=allow_scale,
    )
    yaw_rad = math.atan2(rot_2d[1, 0], rot_2d[0, 0])
    w_inliers = np.maximum(weights[inlier_mask], 1e-12)
    w_inliers /= np.sum(w_inliers)
    tz = float(np.sum(w_inliers * (enu_points[inlier_mask, 2] - local_points[inlier_mask, 2])))
    translation = np.asarray([trans_xy[0], trans_xy[1], tz], dtype=np.float64)
    transformed = apply_similarity_numpy(
        local_points,
        scale=scale,
        yaw_rad=yaw_rad,
        translation_enu_m=translation,
    )
    residual_xy = np.linalg.norm(transformed[:, :2] - enu_points[:, :2], axis=1)
    residual_z = transformed[:, 2] - enu_points[:, 2]
    rmse_xy = float(np.sqrt(np.mean(residual_xy[inlier_mask] ** 2)))
    rmse_z = float(np.sqrt(np.mean(residual_z[inlier_mask] ** 2)))
    return AlignmentResult(
        scale=scale,
        yaw_rad=yaw_rad,
        translation_enu_m=translation,
        reference_llh_deg_m=reference_llh,
        reference_ecef_m=reference_ecef,
        inlier_mask=inlier_mask,
        weights=weights,
        residual_xy_m=residual_xy,
        residual_z_m=residual_z,
        rmse_xy_m=rmse_xy,
        rmse_z_m=rmse_z,
    )


def determine_output_dir(args: argparse.Namespace) -> Path:
    if args.output_dir:
        out_dir = Path(args.output_dir)
    elif args.objects_json:
        out_dir = Path(args.objects_json).resolve().parent
    elif args.powerlines_json:
        out_dir = Path(args.powerlines_json).resolve().parent
    else:
        out_dir = FUSION_ROOT
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def output_path(user_value: str, fallback_name: str, output_dir: Path) -> Path:
    return Path(user_value) if user_value else output_dir / fallback_name

def remap_local_xy(points_xyz: np.ndarray) -> np.ndarray:
    out = points_xyz.copy()
    out[:, 0] = points_xyz[:, 1]
    out[:, 1] = -points_xyz[:, 0]
    out[:, 2] = points_xyz[:, 2]
    return out

def points_to_gps(points_xyz: np.ndarray, alignment: AlignmentResult, native_mode: str) -> tuple[np.ndarray, list[dict[str, float]]]:
    remapped_points = np.asarray(points_xyz, dtype=np.float64).copy()

    # Temporary debug patch:
    # rotate local XY by -90 deg before applying the fitted local->ENU transform
    # (x, y) -> (y, -x)
    x_local = remapped_points[:, 0].copy()
    y_local = remapped_points[:, 1].copy()
    remapped_points[:, 0] = -y_local
    remapped_points[:, 1] = x_local

    points_enu = apply_similarity(
        remapped_points,
        scale=alignment.scale,
        yaw_rad=alignment.yaw_rad,
        translation_enu_m=alignment.translation_enu_m,
        native_mode=native_mode,
    )
    gps_points = []
    for point in points_enu:
        llh = enu_to_geodetic(point, alignment.reference_llh_deg_m, alignment.reference_ecef_m)
        gps_points.append({"lat": float(llh[0]), "lon": float(llh[1]), "alt": float(llh[2])})
    return points_enu, gps_points


def georeference_objects_json(input_path: Path, output_value: Path, alignment: AlignmentResult, native_mode: str, transform_path: Path) -> None:
    payload = json.loads(input_path.read_text(encoding="utf-8"))
    for obj in payload.get("objects", []):
        centroid = np.asarray([obj["centroid_map_xyz"]], dtype=np.float64)
        _, gps_points = points_to_gps(centroid, alignment, native_mode)
        obj["gps"] = gps_points[0]
    payload["gps_alignment"] = {"transform_file": str(transform_path.resolve()), "georeferenced": True}
    output_value.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def georeference_powerlines_json(input_path: Path, output_value: Path, alignment: AlignmentResult, native_mode: str, transform_path: Path) -> None:
    payload = json.loads(input_path.read_text(encoding="utf-8"))
    for line in payload.get("powerlines", []):
        centroid = np.asarray([line["centroid_map_xyz"]], dtype=np.float64)
        _, centroid_gps = points_to_gps(centroid, alignment, native_mode)
        line["gps"] = centroid_gps[0]
        polyline = np.asarray(line.get("polyline_map_xyz", []), dtype=np.float64)
        if polyline.ndim == 2 and polyline.shape[1] == 3 and polyline.shape[0] > 0:
            _, polyline_gps = points_to_gps(polyline, alignment, native_mode)
            line["polyline_gps"] = polyline_gps
        else:
            line["polyline_gps"] = []
    payload["gps_alignment"] = {"transform_file": str(transform_path.resolve()), "georeferenced": True}
    output_value.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def write_aligned_csv(output_value: Path, records: list[GpsPoseRecord], local_points: np.ndarray, alignment: AlignmentResult) -> None:
    predicted_enu = apply_similarity_numpy(
        local_points,
        scale=alignment.scale,
        yaw_rad=alignment.yaw_rad,
        translation_enu_m=alignment.translation_enu_m,
    )
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
        "latitude_deg",
        "longitude_deg",
        "altitude_m",
        "predicted_latitude_deg",
        "predicted_longitude_deg",
        "predicted_altitude_m",
        "predicted_enu_x_m",
        "predicted_enu_y_m",
        "predicted_enu_z_m",
        "residual_xy_m",
        "residual_z_m",
        "inlier",
    ]
    with output_value.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for idx, record in enumerate(records):
            predicted_llh = enu_to_geodetic(predicted_enu[idx], alignment.reference_llh_deg_m, alignment.reference_ecef_m)
            writer.writerow(
                {
                    "t_in_sec": f"{record.t_in_sec:.9f}",
                    "t_query_sec": f"{record.t_query_sec:.9f}",
                    "x": f"{record.translation_xyz[0]:.9f}",
                    "y": f"{record.translation_xyz[1]:.9f}",
                    "z": f"{record.translation_xyz[2]:.9f}",
                    "qx": f"{record.quaternion_xyzw[0]:.9f}",
                    "qy": f"{record.quaternion_xyzw[1]:.9f}",
                    "qz": f"{record.quaternion_xyzw[2]:.9f}",
                    "qw": f"{record.quaternion_xyzw[3]:.9f}",
                    "latitude_deg": f"{record.latitude_deg:.9f}",
                    "longitude_deg": f"{record.longitude_deg:.9f}",
                    "altitude_m": f"{record.altitude_m:.6f}",
                    "predicted_latitude_deg": f"{predicted_llh[0]:.9f}",
                    "predicted_longitude_deg": f"{predicted_llh[1]:.9f}",
                    "predicted_altitude_m": f"{predicted_llh[2]:.6f}",
                    "predicted_enu_x_m": f"{predicted_enu[idx, 0]:.6f}",
                    "predicted_enu_y_m": f"{predicted_enu[idx, 1]:.6f}",
                    "predicted_enu_z_m": f"{predicted_enu[idx, 2]:.6f}",
                    "residual_xy_m": f"{alignment.residual_xy_m[idx]:.6f}",
                    "residual_z_m": f"{alignment.residual_z_m[idx]:.6f}",
                    "inlier": int(alignment.inlier_mask[idx]),
                }
            )


def write_transform_json(
    output_value: Path,
    source_csv: Path,
    gps_to_lidar_offset_body: np.ndarray,
    records: list[GpsPoseRecord],
    alignment: AlignmentResult,
    allow_scale: bool,
    gps_time_offset_sec: float,
    dense_traj_csv: str,
) -> None:
    yaw_deg = math.degrees(alignment.yaw_rad)
    cos_yaw = math.cos(alignment.yaw_rad)
    sin_yaw = math.sin(alignment.yaw_rad)
    gps_offset_active = bool(
        abs(float(gps_time_offset_sec)) > 1e-12 and dense_traj_csv and Path(dense_traj_csv).is_file()
    )
    payload = {
        "pipeline": "georeference_from_tf_gps",
        "source_tf_gps_csv": str(source_csv.resolve()),
        "model": "weighted_se2_plus_z_offset" if not allow_scale else "weighted_similarity2d_plus_z_offset",
        "gps_to_lidar_offset_body_m": [float(value) for value in gps_to_lidar_offset_body],
        "reference_llh_deg_m": {
            "lat": float(alignment.reference_llh_deg_m[0]),
            "lon": float(alignment.reference_llh_deg_m[1]),
            "alt": float(alignment.reference_llh_deg_m[2]),
        },
        "transform": {
            "scale": float(alignment.scale),
            "yaw_deg": float(yaw_deg),
            "yaw_rad": float(alignment.yaw_rad),
            "rotation_matrix_enu_from_local": [
                [float(cos_yaw), float(-sin_yaw), 0.0],
                [float(sin_yaw), float(cos_yaw), 0.0],
                [0.0, 0.0, 1.0],
            ],
            "translation_enu_m": [float(value) for value in alignment.translation_enu_m],
        },
        "fit_quality": {
            "num_samples_total": len(records),
            "num_samples_inliers": int(np.count_nonzero(alignment.inlier_mask)),
            "rmse_xy_m": float(alignment.rmse_xy_m),
            "rmse_z_m": float(alignment.rmse_z_m),
        },
        "gps_offset_compensation": {
            "gps_time_offset_sec": float(gps_time_offset_sec),
            "dense_traj_csv": str(Path(dense_traj_csv).resolve()) if dense_traj_csv else "",
            "active": gps_offset_active,
        },
    }
    output_value.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def main() -> int:
    args = parse_args()

    tf_gps_csv = Path(args.tf_gps_csv)
    output_dir = determine_output_dir(args)
    transform_output = output_path(args.transform_output_json, "gps_alignment.json", output_dir)
    objects_output = output_path(args.objects_output_json, "fused_objects_georeferenced.json", output_dir)
    powerlines_output = output_path(args.powerlines_output_json, "eng498_powerlines_overlay_georeferenced.json", output_dir)
    aligned_csv_output = output_path(args.aligned_csv_output, "tf_gps_georeferenced.csv", output_dir)

    gps_to_lidar_offset_body = load_offset(args)
    filtered_records = filter_records(
        load_tf_gps_records(tf_gps_csv),
        min_fix_status=args.min_fix_status,
        max_horizontal_cov_m2=args.max_horizontal_cov_m2,
    )
    records, local_points = prepare_georeference_inputs(
        filtered_records,
        gps_to_lidar_offset_body=gps_to_lidar_offset_body,
        gps_time_offset_sec=args.gps_time_offset_sec,
        dense_traj_csv=args.dense_traj_csv,
    )
    alignment = fit_alignment(
        records,
        gps_to_lidar_offset_body=gps_to_lidar_offset_body,
        local_points=local_points,
        allow_scale=args.allow_scale,
        outlier_threshold_m=args.outlier_threshold_m,
    )

    write_transform_json(
        transform_output,
        tf_gps_csv,
        gps_to_lidar_offset_body,
        records,
        alignment,
        args.allow_scale,
        args.gps_time_offset_sec,
        args.dense_traj_csv,
    )
    write_aligned_csv(aligned_csv_output, records, local_points, alignment)

    if args.objects_json:
        georeference_objects_json(Path(args.objects_json), objects_output, alignment, args.native_mode, transform_output)
        print(f"[done] wrote georeferenced objects to {objects_output}")
    if args.powerlines_json:
        georeference_powerlines_json(Path(args.powerlines_json), powerlines_output, alignment, args.native_mode, transform_output)
        print(f"[done] wrote georeferenced powerlines to {powerlines_output}")

    print(f"[done] wrote transform report to {transform_output}")
    print(f"[done] wrote aligned CSV to {aligned_csv_output}")
    print(
        f"[done] inliers {int(np.count_nonzero(alignment.inlier_mask))}/{len(records)}, "
        f"RMSE XY {alignment.rmse_xy_m:.3f} m, RMSE Z {alignment.rmse_z_m:.3f} m"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
