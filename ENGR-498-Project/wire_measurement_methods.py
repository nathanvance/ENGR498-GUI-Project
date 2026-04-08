from __future__ import annotations

from typing import Any

import numpy as np
from scipy.optimize import curve_fit


_EPS = 1e-9


def normalize_sag_method(value: str | None) -> str:
    return "fusion_span" if str(value or "").strip().lower() == "fusion_span" else "legacy"


def compute_legacy_wire_measurement(curve_points: np.ndarray | None) -> dict[str, Any]:
    points = np.asarray(curve_points, dtype=np.float64) if curve_points is not None else np.empty((0, 3))
    if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 3:
        return {
            "method": "legacy",
            "status": "unavailable",
            "message": "Legacy sag requires a fitted wire curve.",
            "sag_m": None,
        }

    p1 = points[0]
    p2 = points[-1]
    idx_lowest = int(np.argmin(points[:, 2]))
    lowest_point = points[idx_lowest]
    chord_vec = p2 - p1

    chord_norm_sq = float(np.dot(chord_vec, chord_vec))
    if chord_norm_sq < _EPS:
        sag_m = 0.0
    else:
        rel = lowest_point - p1
        t = float(np.clip(np.dot(rel, chord_vec) / chord_norm_sq, 0.0, 1.0))
        point_on_chord = p1 + t * chord_vec
        sag_m = max(0.0, float(point_on_chord[2] - lowest_point[2]))

    return {
        "method": "legacy",
        "status": "ok",
        "message": "Legacy chord-based sag measurement.",
        "sag_m": sag_m,
    }


def compute_fusion_span_wire_measurement(
    wire_points: np.ndarray | None,
    pole_links: list[dict[str, Any]] | None,
) -> dict[str, Any]:
    points = np.asarray(wire_points, dtype=np.float64) if wire_points is not None else np.empty((0, 3))
    if points.ndim != 2 or points.shape[0] < 6 or points.shape[1] != 3:
        return {
            "method": "fusion_span",
            "status": "unavailable",
            "message": "Fusion-span sag requires raw wire points.",
            "sag_m": None,
        }

    if not pole_links:
        return {
            "method": "fusion_span",
            "status": "unavailable",
            "message": "Fusion-span sag requires completed Fusion pole-pair outputs.",
            "sag_m": None,
        }

    best_candidate: dict[str, Any] | None = None
    for link in pole_links:
        candidate = _score_pole_link(points, link)
        if candidate is None:
            continue
        if best_candidate is None or float(candidate["score"]) > float(best_candidate["score"]):
            best_candidate = candidate

    if best_candidate is None:
        return {
            "method": "fusion_span",
            "status": "unavailable",
            "message": "No Fusion pole span matched this wire closely enough.",
            "sag_m": None,
        }

    mask = np.asarray(best_candidate["mask"], dtype=bool)
    selected_points = points[mask]
    along = np.asarray(best_candidate["along"], dtype=np.float64)[mask]
    span_m = float(best_candidate["horizontal_span_m"])

    fit = _fit_shifted_catenary(along, selected_points[:, 2], span_m)
    if fit is None:
        return {
            "method": "fusion_span",
            "status": "fit_failed",
            "message": "Catenary fit failed for the Fusion-matched pole span.",
            "sag_m": None,
            "support_pair": best_candidate["support_pair"],
            "horizontal_span_m": span_m,
            "used_point_count": int(selected_points.shape[0]),
            "corridor_corners_map_xyz": best_candidate["corridor_corners_map_xyz"],
        }

    a_m, x0_m, z0_m = fit
    sag_m = max(0.0, float((span_m * span_m) / (8.0 * max(a_m, _EPS))))
    exact_sag_m = _compute_exact_catenary_sag(span_m, a_m, x0_m, z0_m)

    support_pair = best_candidate["support_pair"]
    return {
        "method": "fusion_span",
        "status": "ok",
        "message": (
            f"Fusion-assisted catenary sag using span "
            f"{support_pair['source_name']} -> {support_pair['target_name']}."
        ),
        "sag_m": sag_m,
        "exact_sag_m": exact_sag_m,
        "horizontal_span_m": span_m,
        "support_pair": support_pair,
        "used_point_count": int(selected_points.shape[0]),
        "fitted_a_m": a_m,
        "fitted_x0_m": x0_m,
        "fitted_z0_m": z0_m,
        "corridor_half_width_m": float(best_candidate["corridor_half_width_m"]),
        "corridor_corners_map_xyz": best_candidate["corridor_corners_map_xyz"],
    }


def _score_pole_link(points: np.ndarray, link: dict[str, Any]) -> dict[str, Any] | None:
    source = np.asarray(link.get("source_centroid"), dtype=np.float64)
    target = np.asarray(link.get("target_centroid"), dtype=np.float64)
    if source.shape != (3,) or target.shape != (3,):
        return None

    span_xy = target[:2] - source[:2]
    horizontal_span_m = float(np.linalg.norm(span_xy))
    if horizontal_span_m < 1.0:
        return None

    along_axis = span_xy / horizontal_span_m
    lateral_axis = np.array([-along_axis[1], along_axis[0]], dtype=np.float64)
    xy = points[:, :2]
    rel_xy = xy - source[:2]
    along = rel_xy @ along_axis
    lateral = rel_xy @ lateral_axis

    corridor_padding_m = max(0.75, 0.06 * horizontal_span_m)
    corridor_half_width_m = max(1.5, 0.12 * horizontal_span_m)
    mask = (
        (along >= -corridor_padding_m)
        & (along <= horizontal_span_m + corridor_padding_m)
        & (np.abs(lateral) <= corridor_half_width_m)
    )
    core_mask = (
        (along >= 0.0)
        & (along <= horizontal_span_m)
        & (np.abs(lateral) <= corridor_half_width_m)
    )

    mask_count = int(np.count_nonzero(mask))
    core_count = int(np.count_nonzero(core_mask))
    if mask_count < 6 or core_count < 4:
        return None

    used_along = along[mask]
    used_lateral = np.abs(lateral[mask])
    span_coverage = 0.0
    if used_along.size:
        span_coverage = float(np.clip((used_along.max() - used_along.min()) / (horizontal_span_m + _EPS), 0.0, 1.0))

    mean_abs_lateral = float(np.mean(used_lateral)) if used_lateral.size else float("inf")
    score = (2.0 * core_count) + (1.0 * mask_count) + (8.0 * span_coverage) - (2.0 * mean_abs_lateral)

    z_low = float(min(points[mask, 2].min(), source[2], target[2]) - 1.0)
    z_high = float(max(points[mask, 2].max(), source[2], target[2]) + 1.0)
    corridor_corners = _build_corridor_box(
        source,
        target,
        along_axis,
        lateral_axis,
        corridor_padding_m,
        corridor_half_width_m,
        z_low,
        z_high,
    )

    return {
        "score": score,
        "mask": mask,
        "along": along,
        "horizontal_span_m": horizontal_span_m,
        "corridor_half_width_m": corridor_half_width_m,
        "corridor_corners_map_xyz": corridor_corners,
        "support_pair": {
            "source_name": str(link.get("source_name", "pole_a")),
            "target_name": str(link.get("target_name", "pole_b")),
        },
    }


def _build_corridor_box(
    source: np.ndarray,
    target: np.ndarray,
    along_axis: np.ndarray,
    lateral_axis: np.ndarray,
    padding_m: float,
    half_width_m: float,
    z_low: float,
    z_high: float,
) -> np.ndarray:
    source_xy = source[:2] - (padding_m * along_axis)
    target_xy = target[:2] + (padding_m * along_axis)

    p0 = source_xy - (half_width_m * lateral_axis)
    p1 = source_xy + (half_width_m * lateral_axis)
    p2 = target_xy + (half_width_m * lateral_axis)
    p3 = target_xy - (half_width_m * lateral_axis)

    lower = np.array(
        [
            [p0[0], p0[1], z_low],
            [p1[0], p1[1], z_low],
            [p2[0], p2[1], z_low],
            [p3[0], p3[1], z_low],
        ],
        dtype=np.float64,
    )
    upper = lower.copy()
    upper[:, 2] = z_high
    return np.vstack((lower, upper))


def _fit_shifted_catenary(along: np.ndarray, z: np.ndarray, horizontal_span_m: float) -> tuple[float, float, float] | None:
    x = np.asarray(along, dtype=np.float64)
    z_vals = np.asarray(z, dtype=np.float64)
    if x.size < 6 or z_vals.size != x.size:
        return None

    order = np.argsort(x)
    x = x[order]
    z_vals = z_vals[order]

    idx_min = int(np.argmin(z_vals))
    x0_guess = float(np.clip(x[idx_min], 0.0, horizontal_span_m))
    support_z_guess = float(np.mean([z_vals[0], z_vals[-1]]))
    observed_sag = max(0.1, support_z_guess - float(z_vals.min()))
    a_guess = max(horizontal_span_m / 2.0, (horizontal_span_m * horizontal_span_m) / (8.0 * observed_sag))
    z0_guess = float(z_vals.min() - a_guess)

    def shifted_catenary(x_axis: np.ndarray, a_m: float, x0_m: float, z0_m: float) -> np.ndarray:
        arg = np.clip((x_axis - x0_m) / max(a_m, _EPS), -50.0, 50.0)
        return a_m * np.cosh(arg) + z0_m

    try:
        params, _ = curve_fit(
            shifted_catenary,
            x,
            z_vals,
            p0=(a_guess, x0_guess, z0_guess),
            bounds=(
                np.array([0.05, -0.25 * horizontal_span_m, -1.0e6], dtype=np.float64),
                np.array([1.0e6, 1.25 * horizontal_span_m, 1.0e6], dtype=np.float64),
            ),
            maxfev=30000,
        )
    except Exception:
        return None

    return float(params[0]), float(params[1]), float(params[2])


def _compute_exact_catenary_sag(horizontal_span_m: float, a_m: float, x0_m: float, z0_m: float) -> float:
    if horizontal_span_m <= 0.0 or a_m <= 0.0:
        return 0.0

    def shifted_catenary(x_axis: float) -> float:
        arg = float(np.clip((x_axis - x0_m) / max(a_m, _EPS), -50.0, 50.0))
        return float(a_m * np.cosh(arg) + z0_m)

    z_start = shifted_catenary(0.0)
    z_end = shifted_catenary(horizontal_span_m)
    x_min = float(np.clip(x0_m, 0.0, horizontal_span_m))
    z_min = shifted_catenary(x_min)
    z_chord = z_start + ((z_end - z_start) * (x_min / max(horizontal_span_m, _EPS)))
    return max(0.0, z_chord - z_min)
