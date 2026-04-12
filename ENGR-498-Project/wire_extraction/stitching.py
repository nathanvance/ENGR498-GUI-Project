from __future__ import annotations

import math
from dataclasses import dataclass, field
from itertools import combinations
from typing import Any, Iterable, Sequence

import numpy as np
from scipy.optimize import curve_fit


_EPS = 1e-9
_WORLD_UP = np.array([0.0, 0.0, 1.0], dtype=np.float64)


@dataclass(frozen=True)
class WireStitchingParameters:
    min_points_for_feature: int = 6
    endpoint_sample_size: int = 3
    min_span_for_catenary_m: float = 1.0
    min_baseline_rmse_m: float = 0.05
    min_points_for_bimodality: int = 12
    max_centroid_distance_m: float = 14.0
    max_centroid_to_plane_distance_m: float = 0.75
    min_direction_similarity: float = 0.965
    max_projected_gap_m: float = 8.0
    max_vertical_offset_m: float = 4.0
    max_endpoint_distance_m: float = 8.0
    max_lateral_offset_m: float = 0.60
    max_lateral_offset_norm: float = 4.0
    min_side_by_side_overlap_ratio: float = 0.35
    min_side_by_side_overlap_m: float = 2.0
    min_side_by_side_delta_v_m: float = 0.22
    min_side_by_side_delta_v_norm: float = 2.25
    max_pair_plane_rms_m: float = 0.45
    max_pair_catenary_rmse_m: float = 0.30
    max_pair_rmse_increase_m: float = 0.18
    max_pair_rmse_ratio: float = 2.50
    max_pair_catenary_log_a_shift: float = 1.25
    max_component_plane_rms_m: float = 0.50
    max_component_catenary_rmse_m: float = 0.35
    max_component_rmse_increase_m: float = 0.20
    max_component_rmse_ratio: float = 2.75
    max_component_track_spread_m: float = 0.22
    max_component_track_rmse_m: float = 0.18
    max_component_cluster_track_deviation_m: float = 0.20
    max_component_cluster_track_deviation_norm: float = 3.0
    bimodal_min_gap_m: float = 0.18
    bimodal_min_center_separation_m: float = 0.28
    bimodal_gap_scale: float = 2.0
    edge_accept_cost: float = 0.85
    plane_cost_weight: float = 1.20
    direction_cost_weight: float = 1.15
    gap_cost_weight: float = 1.00
    endpoint_cost_weight: float = 0.90
    vertical_cost_weight: float = 0.60
    lateral_cost_weight: float = 1.40
    lateral_norm_cost_weight: float = 1.10
    overlap_cost_weight: float = 0.80
    pair_plane_cost_weight: float = 1.05
    pair_rmse_cost_weight: float = 1.20
    pair_rmse_increase_weight: float = 1.10
    pair_catenary_shift_weight: float = 0.70
    max_clusters_per_span_candidate: int = 8
    max_span_catenary_rmse_m: float = 0.35
    max_span_rmse_increase_m: float = 0.20
    max_span_rmse_ratio: float = 2.75
    max_span_track_spread_m: float = 0.22
    max_span_track_rmse_m: float = 0.18
    max_span_cluster_track_deviation_m: float = 0.20
    max_span_cluster_track_deviation_norm: float = 3.0
    span_partition_base_cost: float = 0.55
    singleton_span_extra_cost: float = 0.25
    singleton_invalid_fit_cost: float = 1.35
    segment_rmse_cost_weight: float = 1.50
    segment_rmse_increase_cost_weight: float = 1.15
    segment_gap_cost_weight: float = 0.70
    segment_short_span_cost_weight: float = 0.50
    segment_track_cost_weight: float = 0.90

    def to_dict(self) -> dict[str, Any]:
        return {
            field_name: getattr(self, field_name)
            for field_name in self.__dataclass_fields__
        }


@dataclass
class CatenaryFitResult:
    status: str
    horizontal_span_m: float
    point_count: int
    rmse_m: float | None = None
    residual_mean_abs_m: float | None = None
    a_m: float | None = None
    x0_m: float | None = None
    z0_m: float | None = None
    along_min_m: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "horizontal_span_m": float(self.horizontal_span_m),
            "point_count": int(self.point_count),
            "rmse_m": _maybe_float(self.rmse_m),
            "residual_mean_abs_m": _maybe_float(self.residual_mean_abs_m),
            "a_m": _maybe_float(self.a_m),
            "x0_m": _maybe_float(self.x0_m),
            "z0_m": _maybe_float(self.z0_m),
            "along_min_m": _maybe_float(self.along_min_m),
        }


@dataclass
class ClusterFeature:
    cluster_index: int
    points: np.ndarray
    centroid: np.ndarray
    point_count: int
    bbox_min: np.ndarray
    bbox_max: np.ndarray
    principal_direction: np.ndarray
    span_direction: np.ndarray
    lateral_direction: np.ndarray
    vertical_direction: np.ndarray
    plane_point: np.ndarray
    plane_normal: np.ndarray
    plane_rms_m: float
    projected_span_length_m: float
    start_point: np.ndarray
    end_point: np.ndarray
    u_min_m: float
    u_max_m: float
    v_center_m: float
    v_spread_m: float
    v_mad_m: float
    track_rmse_m: float
    track_p95_abs_m: float
    catenary_fit: CatenaryFitResult

    def to_debug_dict(self) -> dict[str, Any]:
        return {
            "cluster_index": int(self.cluster_index),
            "point_count": int(self.point_count),
            "centroid": _vector_to_list(self.centroid),
            "bbox_min": _vector_to_list(self.bbox_min),
            "bbox_max": _vector_to_list(self.bbox_max),
            "principal_direction": _vector_to_list(self.principal_direction),
            "span_direction": _vector_to_list(self.span_direction),
            "lateral_direction": _vector_to_list(self.lateral_direction),
            "vertical_direction": _vector_to_list(self.vertical_direction),
            "plane_point": _vector_to_list(self.plane_point),
            "plane_normal": _vector_to_list(self.plane_normal),
            "plane_rms_m": float(self.plane_rms_m),
            "projected_span_length_m": float(self.projected_span_length_m),
            "start_point": _vector_to_list(self.start_point),
            "end_point": _vector_to_list(self.end_point),
            "u_min_m": float(self.u_min_m),
            "u_max_m": float(self.u_max_m),
            "v_center_m": float(self.v_center_m),
            "v_spread_m": float(self.v_spread_m),
            "v_mad_m": float(self.v_mad_m),
            "track_rmse_m": float(self.track_rmse_m),
            "track_p95_abs_m": float(self.track_p95_abs_m),
            "catenary_fit": self.catenary_fit.to_dict(),
        }


@dataclass
class CandidateEdge:
    edge_index: int
    cluster_a: int
    cluster_b: int
    cost: float
    score: float
    accepted_for_graph: bool
    decision_reason: str
    centroid_distance_m: float
    mutual_plane_distance_m: float
    direction_similarity: float
    delta_v_m: float
    delta_v_norm: float
    u_overlap_m: float
    u_overlap_ratio: float
    projected_gap_m: float
    vertical_offset_m: float
    endpoint_distance_m: float
    pair_plane_rms_m: float
    pair_catenary_rmse_m: float
    pair_rmse_increase_m: float
    pair_rmse_ratio: float
    pair_catenary_log_a_shift: float
    pair_track_spread_m: float
    pair_track_rmse_m: float
    retained_after_validation: bool = False

    def to_debug_dict(self) -> dict[str, Any]:
        return {
            "edge_index": int(self.edge_index),
            "cluster_a": int(self.cluster_a),
            "cluster_b": int(self.cluster_b),
            "cost": float(self.cost),
            "score": float(self.score),
            "accepted_for_graph": bool(self.accepted_for_graph),
            "decision_reason": self.decision_reason,
            "centroid_distance_m": float(self.centroid_distance_m),
            "mutual_plane_distance_m": float(self.mutual_plane_distance_m),
            "direction_similarity": float(self.direction_similarity),
            "delta_v_m": float(self.delta_v_m),
            "delta_v_norm": float(self.delta_v_norm),
            "u_overlap_m": float(self.u_overlap_m),
            "u_overlap_ratio": float(self.u_overlap_ratio),
            "projected_gap_m": float(self.projected_gap_m),
            "vertical_offset_m": float(self.vertical_offset_m),
            "endpoint_distance_m": float(self.endpoint_distance_m),
            "pair_plane_rms_m": float(self.pair_plane_rms_m),
            "pair_catenary_rmse_m": float(self.pair_catenary_rmse_m),
            "pair_rmse_increase_m": float(self.pair_rmse_increase_m),
            "pair_rmse_ratio": float(self.pair_rmse_ratio),
            "pair_catenary_log_a_shift": float(self.pair_catenary_log_a_shift),
            "pair_track_spread_m": float(self.pair_track_spread_m),
            "pair_track_rmse_m": float(self.pair_track_rmse_m),
            "retained_after_validation": bool(self.retained_after_validation),
        }


@dataclass
class StitchedWireGroup:
    group_index: int
    source_cluster_indices: list[int]
    points: np.ndarray
    feature: ClusterFeature
    validation: dict[str, Any]
    accepted_edge_indices: list[int] = field(default_factory=list)

    def to_debug_dict(self) -> dict[str, Any]:
        return {
            "group_index": int(self.group_index),
            "source_cluster_indices": [int(value) for value in self.source_cluster_indices],
            "point_count": int(self.points.shape[0]),
            "accepted_edge_indices": [int(value) for value in self.accepted_edge_indices],
            "validation": _to_serializable(self.validation),
            "feature": self.feature.to_debug_dict(),
        }


@dataclass
class WireStitchingResult:
    groups: list[StitchedWireGroup]
    cluster_features: list[ClusterFeature]
    candidate_edges: list[CandidateEdge]
    parameters: WireStitchingParameters
    initial_components: list[list[int]]
    candidate_tracks: list[list[int]]
    gate_failures: dict[str, int]

    def to_report_dict(self) -> dict[str, Any]:
        retained_edge_ids = {
            edge_index
            for group in self.groups
            for edge_index in group.accepted_edge_indices
        }
        for edge in self.candidate_edges:
            edge.retained_after_validation = edge.edge_index in retained_edge_ids

        input_cluster_count = len(self.cluster_features)
        merged_group_count = sum(1 for group in self.groups if len(group.source_cluster_indices) > 1)
        return {
            "algorithm": "graph_track_partition_wire_stitching",
            "parameters": self.parameters.to_dict(),
            "summary": {
                "input_cluster_count": input_cluster_count,
                "candidate_edge_count": len(self.candidate_edges),
                "accepted_edge_count": sum(1 for edge in self.candidate_edges if edge.accepted_for_graph),
                "retained_edge_count": len(retained_edge_ids),
                "initial_component_count": len(self.initial_components),
                "candidate_track_count": len(self.candidate_tracks),
                "final_group_count": len(self.groups),
                "merged_group_count": merged_group_count,
                "clusters_merged_into_larger_groups": int(
                    sum(len(group.source_cluster_indices) for group in self.groups if len(group.source_cluster_indices) > 1)
                ),
                "gate_failures": dict(sorted(self.gate_failures.items())),
            },
            "initial_components": [
                [int(cluster_index) for cluster_index in component]
                for component in self.initial_components
            ],
            "candidate_tracks": [
                [int(cluster_index) for cluster_index in component]
                for component in self.candidate_tracks
            ],
            "cluster_features": [feature.to_debug_dict() for feature in self.cluster_features],
            "candidate_edges": [edge.to_debug_dict() for edge in self.candidate_edges],
            "final_groups": [group.to_debug_dict() for group in self.groups],
        }


@dataclass
class SpanSegmentEvaluation:
    start_index: int
    end_index: int
    source_cluster_indices: list[int]
    points: np.ndarray
    feature: ClusterFeature
    accepted_edge_indices: list[int]
    valid: bool
    status: str
    cost: float
    point_count: int
    cluster_count: int
    baseline_local_rmse_m: float
    rmse_increase_m: float | None
    rmse_ratio: float | None
    shared_track_center_v_m: float
    track_spread_m: float
    track_rmse_m: float
    max_cluster_track_deviation_m: float
    max_cluster_track_deviation_norm: float
    total_internal_gap_m: float
    max_internal_gap_m: float
    short_span_penalty: float

    def to_validation_dict(self, *, validation_reason: str, sequence_indices: Sequence[int]) -> dict[str, Any]:
        return {
            "accepted": bool(self.valid),
            "status": self.status,
            "reason": validation_reason,
            "partition_method": "ordered_dynamic_programming",
            "ordered_sequence_indices": [int(value) for value in sequence_indices],
            "segment_start_index": int(self.start_index),
            "segment_end_index": int(self.end_index),
            "segment_cost": float(self.cost),
            "plane_rms_m": float(self.feature.plane_rms_m),
            "catenary_status": self.feature.catenary_fit.status,
            "catenary_rmse_m": _maybe_float(self.feature.catenary_fit.rmse_m),
            "baseline_local_rmse_m": float(self.baseline_local_rmse_m),
            "rmse_increase_m": _maybe_float(self.rmse_increase_m),
            "rmse_ratio": _maybe_float(self.rmse_ratio),
            "point_count": int(self.point_count),
            "source_cluster_count": int(self.cluster_count),
            "accepted_edge_count": int(len(self.accepted_edge_indices)),
            "shared_track_center_v_m": float(self.shared_track_center_v_m),
            "track_spread_m": float(self.track_spread_m),
            "track_rmse_m": float(self.track_rmse_m),
            "track_p95_abs_m": float(self.feature.track_p95_abs_m),
            "max_cluster_track_deviation_m": float(self.max_cluster_track_deviation_m),
            "max_cluster_track_deviation_norm": float(self.max_cluster_track_deviation_norm),
            "total_internal_gap_m": float(self.total_internal_gap_m),
            "max_internal_gap_m": float(self.max_internal_gap_m),
            "short_span_penalty": float(self.short_span_penalty),
        }


def stitch_wire_clusters(
    cluster_points: Sequence[np.ndarray],
    params: WireStitchingParameters | None = None,
) -> WireStitchingResult:
    params = params or WireStitchingParameters()
    normalized_clusters = [np.asarray(points, dtype=np.float64) for points in cluster_points]
    features = [
        _compute_cluster_feature(cluster_index=index, points=points, params=params)
        for index, points in enumerate(normalized_clusters)
        if points.size
    ]
    if not features:
        return WireStitchingResult(
            groups=[],
            cluster_features=[],
            candidate_edges=[],
            parameters=params,
            initial_components=[],
            candidate_tracks=[],
            gate_failures={},
        )

    candidate_edges, gate_failures = _build_candidate_edges(features, params)
    accepted_edges = [edge for edge in candidate_edges if edge.accepted_for_graph]
    initial_components = _connected_components(
        node_ids=[feature.cluster_index for feature in features],
        edges=accepted_edges,
    )

    feature_map = {feature.cluster_index: feature for feature in features}
    candidate_tracks: list[list[int]] = []
    for component in initial_components:
        component_edges = [
            edge
            for edge in accepted_edges
            if edge.cluster_a in component and edge.cluster_b in component
        ]
        candidate_tracks.extend(
            _refine_track_component(
                node_ids=component,
                component_edges=component_edges,
                feature_map=feature_map,
                params=params,
            )
        )

    final_groups: list[StitchedWireGroup] = []
    for component in candidate_tracks:
        component_edges = [
            edge
            for edge in accepted_edges
            if edge.cluster_a in component and edge.cluster_b in component
        ]
        final_groups.extend(
            _partition_track_component(
                node_ids=component,
                component_edges=component_edges,
                feature_map=feature_map,
                params=params,
            )
        )

    final_groups.sort(key=lambda group: (min(group.source_cluster_indices), group.group_index))
    for group_index, group in enumerate(final_groups):
        group.group_index = group_index

    return WireStitchingResult(
        groups=final_groups,
        cluster_features=features,
        candidate_edges=candidate_edges,
        parameters=params,
        initial_components=initial_components,
        candidate_tracks=[sorted(int(node_id) for node_id in component) for component in candidate_tracks],
        gate_failures=gate_failures,
    )


def _build_candidate_edges(
    features: Sequence[ClusterFeature],
    params: WireStitchingParameters,
) -> tuple[list[CandidateEdge], dict[str, int]]:
    candidate_edges: list[CandidateEdge] = []
    gate_failures: dict[str, int] = {}
    edge_index = 0

    for feature_a, feature_b in combinations(features, 2):
        edge, rejection_reason = _evaluate_pair(feature_a, feature_b, params, edge_index=edge_index)
        if edge is not None:
            candidate_edges.append(edge)
            edge_index += 1
            continue
        gate_failures[rejection_reason] = gate_failures.get(rejection_reason, 0) + 1

    candidate_edges.sort(key=lambda edge: (edge.cost, edge.cluster_a, edge.cluster_b))
    for new_edge_index, edge in enumerate(candidate_edges):
        edge.edge_index = new_edge_index
    return candidate_edges, gate_failures


def _evaluate_pair(
    feature_a: ClusterFeature,
    feature_b: ClusterFeature,
    params: WireStitchingParameters,
    *,
    edge_index: int,
) -> tuple[CandidateEdge | None, str]:
    centroid_distance = float(np.linalg.norm(feature_a.centroid - feature_b.centroid))
    if centroid_distance > params.max_centroid_distance_m:
        return None, "centroid_distance"

    mutual_plane_distance = max(
        _point_to_plane_distance(feature_b.centroid, feature_a.plane_point, feature_a.plane_normal),
        _point_to_plane_distance(feature_a.centroid, feature_b.plane_point, feature_b.plane_normal),
    )
    if mutual_plane_distance > params.max_centroid_to_plane_distance_m:
        return None, "plane_distance"

    direction_similarity = float(abs(np.dot(feature_a.span_direction, feature_b.span_direction)))
    if direction_similarity < params.min_direction_similarity:
        return None, "direction_similarity"

    pair_axis = _average_direction(feature_a.span_direction, feature_b.span_direction)
    pair_lateral_axis = _cross_track_direction(pair_axis)
    pair_origin = 0.5 * (feature_a.centroid + feature_b.centroid)
    pair_u_a, pair_v_a, _ = _project_points(feature_a.points, pair_origin, pair_axis, pair_lateral_axis)
    pair_u_b, pair_v_b, _ = _project_points(feature_b.points, pair_origin, pair_axis, pair_lateral_axis)
    interval_a, start_a, end_a = _interval_and_endpoints(feature_a.points, pair_origin, pair_axis, params.endpoint_sample_size)
    interval_b, start_b, end_b = _interval_and_endpoints(feature_b.points, pair_origin, pair_axis, params.endpoint_sample_size)
    delta_v_m = float(abs(_robust_center(pair_v_a) - _robust_center(pair_v_b)))
    delta_v_norm = float(
        delta_v_m / max(_robust_spread(pair_v_a), _robust_spread(pair_v_b), params.min_baseline_rmse_m, _EPS)
    )
    u_overlap_m = _interval_overlap(interval_a, interval_b)
    overlap_denominator = max(
        min(interval_a[1] - interval_a[0], interval_b[1] - interval_b[0]),
        _EPS,
    )
    u_overlap_ratio = float(np.clip(u_overlap_m / overlap_denominator, 0.0, 1.0))
    projected_gap_m, endpoint_distance_m = _pair_gap_and_endpoint_distance(interval_a, interval_b, start_a, end_a, start_b, end_b)
    if projected_gap_m > params.max_projected_gap_m:
        return None, "projected_gap"
    if endpoint_distance_m > params.max_endpoint_distance_m:
        return None, "endpoint_distance"
    if delta_v_m > params.max_lateral_offset_m:
        return None, "lateral_offset"
    if delta_v_norm > params.max_lateral_offset_norm:
        return None, "lateral_offset_norm"
    if (
        u_overlap_ratio >= params.min_side_by_side_overlap_ratio
        and u_overlap_m >= params.min_side_by_side_overlap_m
        and delta_v_m >= params.min_side_by_side_delta_v_m
        and delta_v_norm >= params.min_side_by_side_delta_v_norm
    ):
        return None, "side_by_side_parallel"

    vertical_offset_m = float(abs(feature_a.centroid[2] - feature_b.centroid[2]))
    if vertical_offset_m > params.max_vertical_offset_m:
        return None, "vertical_offset"

    pair_feature = _compute_cluster_feature(
        cluster_index=-1,
        points=np.vstack((feature_a.points, feature_b.points)),
        params=params,
        forced_span_direction=pair_axis,
    )
    if pair_feature.catenary_fit.status != "ok":
        return None, "pair_catenary_fit"

    pair_plane_rms_m = float(pair_feature.plane_rms_m)
    if pair_plane_rms_m > params.max_pair_plane_rms_m:
        return None, "pair_plane_rms"

    pair_rmse_m = float(pair_feature.catenary_fit.rmse_m or math.inf)
    if pair_rmse_m > params.max_pair_catenary_rmse_m:
        return None, "pair_catenary_rmse"

    baseline_rmse_m = _baseline_rmse([feature_a, feature_b], params)
    pair_rmse_increase_m = float(pair_rmse_m - baseline_rmse_m)
    if pair_rmse_increase_m > params.max_pair_rmse_increase_m:
        return None, "pair_rmse_increase"

    pair_rmse_ratio = float(pair_rmse_m / max(baseline_rmse_m, params.min_baseline_rmse_m))
    if pair_rmse_ratio > params.max_pair_rmse_ratio:
        return None, "pair_rmse_ratio"

    pair_catenary_log_a_shift = _catenary_log_a_shift(pair_feature.catenary_fit, [feature_a, feature_b])
    if pair_catenary_log_a_shift > params.max_pair_catenary_log_a_shift:
        return None, "pair_catenary_shift"
    pair_track_spread_m = float(pair_feature.v_spread_m)
    pair_track_rmse_m = float(pair_feature.track_rmse_m)
    if (
        u_overlap_ratio >= params.min_side_by_side_overlap_ratio
        and pair_track_spread_m >= params.max_component_track_spread_m
    ):
        return None, "pair_multi_track"

    cost = _weighted_normalized_cost(
        values={
            "plane": mutual_plane_distance / params.max_centroid_to_plane_distance_m,
            "direction": (1.0 - direction_similarity) / max(1.0 - params.min_direction_similarity, _EPS),
            "lateral": delta_v_m / params.max_lateral_offset_m,
            "lateral_norm": delta_v_norm / params.max_lateral_offset_norm,
            "overlap": u_overlap_ratio,
            "gap": projected_gap_m / params.max_projected_gap_m,
            "endpoint": endpoint_distance_m / params.max_endpoint_distance_m,
            "vertical": vertical_offset_m / params.max_vertical_offset_m,
            "pair_plane": pair_plane_rms_m / params.max_pair_plane_rms_m,
            "pair_rmse": pair_rmse_m / params.max_pair_catenary_rmse_m,
            "pair_rmse_increase": max(pair_rmse_increase_m, 0.0) / params.max_pair_rmse_increase_m,
            "pair_catenary_shift": pair_catenary_log_a_shift / params.max_pair_catenary_log_a_shift,
        },
        weights={
            "plane": params.plane_cost_weight,
            "direction": params.direction_cost_weight,
            "lateral": params.lateral_cost_weight,
            "lateral_norm": params.lateral_norm_cost_weight,
            "overlap": params.overlap_cost_weight,
            "gap": params.gap_cost_weight,
            "endpoint": params.endpoint_cost_weight,
            "vertical": params.vertical_cost_weight,
            "pair_plane": params.pair_plane_cost_weight,
            "pair_rmse": params.pair_rmse_cost_weight,
            "pair_rmse_increase": params.pair_rmse_increase_weight,
            "pair_catenary_shift": params.pair_catenary_shift_weight,
        },
    )
    accepted_for_graph = cost <= params.edge_accept_cost
    decision_reason = "accepted" if accepted_for_graph else "cost_threshold"
    score = 1.0 / (1.0 + max(cost, 0.0))

    return CandidateEdge(
        edge_index=edge_index,
        cluster_a=feature_a.cluster_index,
        cluster_b=feature_b.cluster_index,
        cost=float(cost),
        score=float(score),
        accepted_for_graph=accepted_for_graph,
        decision_reason=decision_reason,
        centroid_distance_m=centroid_distance,
        mutual_plane_distance_m=mutual_plane_distance,
        direction_similarity=direction_similarity,
        delta_v_m=delta_v_m,
        delta_v_norm=delta_v_norm,
        u_overlap_m=u_overlap_m,
        u_overlap_ratio=u_overlap_ratio,
        projected_gap_m=projected_gap_m,
        vertical_offset_m=vertical_offset_m,
        endpoint_distance_m=endpoint_distance_m,
        pair_plane_rms_m=pair_plane_rms_m,
        pair_catenary_rmse_m=pair_rmse_m,
        pair_rmse_increase_m=pair_rmse_increase_m,
        pair_rmse_ratio=pair_rmse_ratio,
        pair_catenary_log_a_shift=pair_catenary_log_a_shift,
        pair_track_spread_m=pair_track_spread_m,
        pair_track_rmse_m=pair_track_rmse_m,
    ), decision_reason


def _refine_track_component(
    node_ids: Sequence[int],
    component_edges: Sequence[CandidateEdge],
    feature_map: dict[int, ClusterFeature],
    params: WireStitchingParameters,
) -> list[list[int]]:
    ordered_node_ids = sorted(int(node_id) for node_id in node_ids)
    if len(ordered_node_ids) <= 1:
        return [ordered_node_ids]

    track_feature, track_validation = _evaluate_track_component(ordered_node_ids, feature_map, params)
    if bool(track_validation.get("accepted")):
        return [ordered_node_ids]

    suggested_track_split = track_validation.get("suggested_track_split")
    if isinstance(suggested_track_split, list) and len(suggested_track_split) >= 2:
        refined_tracks: list[list[int]] = []
        for component in suggested_track_split:
            if not component:
                continue
            inner_edges = [
                edge
                for edge in component_edges
                if edge.cluster_a in component and edge.cluster_b in component
            ]
            refined_tracks.extend(
                _refine_track_component(
                    node_ids=component,
                    component_edges=inner_edges,
                    feature_map=feature_map,
                    params=params,
                )
            )
        if refined_tracks:
            return refined_tracks

    remaining_edges = list(component_edges)
    while remaining_edges:
        weakest_edge = max(remaining_edges, key=lambda edge: (edge.cost, edge.cluster_a, edge.cluster_b))
        remaining_edges.remove(weakest_edge)
        split_components = _connected_components(node_ids=ordered_node_ids, edges=remaining_edges)
        if len(split_components) == 1 and len(split_components[0]) == len(ordered_node_ids):
            continue

        refined_tracks: list[list[int]] = []
        for component in split_components:
            inner_edges = [
                edge
                for edge in remaining_edges
                if edge.cluster_a in component and edge.cluster_b in component
            ]
            refined_tracks.extend(
                _refine_track_component(
                    node_ids=component,
                    component_edges=inner_edges,
                    feature_map=feature_map,
                    params=params,
                )
            )
        return refined_tracks

    return [[cluster_index] for cluster_index in ordered_node_ids]


def _evaluate_track_component(
    node_ids: Sequence[int],
    feature_map: dict[int, ClusterFeature],
    params: WireStitchingParameters,
) -> tuple[ClusterFeature, dict[str, Any]]:
    ordered_indices = sorted(int(index) for index in node_ids)
    group_points = np.vstack([feature_map[index].points for index in ordered_indices])
    group_feature = _compute_cluster_feature(cluster_index=-1, points=group_points, params=params)
    cluster_track_stats = _project_cluster_track_stats(
        ordered_indices,
        feature_map,
        group_feature.plane_point,
        group_feature.span_direction,
        group_feature.lateral_direction,
    )
    cluster_track_centers = [float(entry["v_center_m"]) for entry in cluster_track_stats]
    cluster_track_spreads = [float(entry["v_spread_m"]) for entry in cluster_track_stats]
    shared_track_center_v = float(np.median(cluster_track_centers)) if cluster_track_centers else 0.0
    max_cluster_track_deviation_m = float(
        max((abs(value - shared_track_center_v) for value in cluster_track_centers), default=0.0)
    )
    max_cluster_track_deviation_norm = float(
        max(
            (
                abs(value - shared_track_center_v) / max(spread, params.min_baseline_rmse_m, _EPS)
                for value, spread in zip(cluster_track_centers, cluster_track_spreads)
            ),
            default=0.0,
        )
    )
    bimodality = _detect_v_bimodality(
        group_feature.points,
        group_feature.plane_point,
        group_feature.span_direction,
        group_feature.lateral_direction,
        params,
        ordered_indices=ordered_indices,
        feature_map=feature_map,
    )
    suggested_track_split = bimodality.get("cluster_split")
    accepted = (
        group_feature.plane_rms_m <= params.max_component_plane_rms_m
        and group_feature.v_spread_m <= params.max_component_track_spread_m
        and group_feature.track_rmse_m <= params.max_component_track_rmse_m
        and max_cluster_track_deviation_m <= params.max_component_cluster_track_deviation_m
        and max_cluster_track_deviation_norm <= params.max_component_cluster_track_deviation_norm
        and not bool(bimodality.get("is_bimodal"))
    )
    if accepted:
        status = "accepted_track"
    elif bool(bimodality.get("is_bimodal")):
        status = "rejected_multi_track"
    elif max_cluster_track_deviation_m > params.max_component_cluster_track_deviation_m:
        status = "rejected_track_center"
    elif group_feature.v_spread_m > params.max_component_track_spread_m:
        status = "rejected_track_spread"
    elif group_feature.plane_rms_m > params.max_component_plane_rms_m:
        status = "rejected_track_plane"
    else:
        status = "rejected_track"

    return group_feature, {
        "accepted": bool(accepted),
        "status": status,
        "shared_track_center_v_m": shared_track_center_v,
        "track_spread_m": float(group_feature.v_spread_m),
        "track_rmse_m": float(group_feature.track_rmse_m),
        "max_cluster_track_deviation_m": max_cluster_track_deviation_m,
        "max_cluster_track_deviation_norm": max_cluster_track_deviation_norm,
        "bimodality": bimodality,
        "suggested_track_split": suggested_track_split,
    }


def _partition_track_component(
    node_ids: Sequence[int],
    component_edges: Sequence[CandidateEdge],
    feature_map: dict[int, ClusterFeature],
    params: WireStitchingParameters,
) -> list[StitchedWireGroup]:
    ordered_node_ids = sorted(int(node_id) for node_id in node_ids)
    if not ordered_node_ids:
        return []

    track_points = np.vstack([feature_map[index].points for index in ordered_node_ids])
    track_feature = _compute_cluster_feature(cluster_index=-1, points=track_points, params=params)
    ordered_cluster_stats = _ordered_track_cluster_stats(
        ordered_node_ids,
        feature_map,
        track_feature.plane_point,
        track_feature.span_direction,
        track_feature.lateral_direction,
    )
    segment_table = _precompute_segment_evaluations(
        ordered_cluster_stats,
        component_edges,
        feature_map,
        track_feature,
        params,
    )
    partition = _solve_span_partition(segment_table)

    if not partition:
        partition = [
            _evaluate_span_segment(
                ordered_cluster_stats,
                start_index=index,
                end_index=index,
                component_edges=component_edges,
                feature_map=feature_map,
                track_feature=track_feature,
                params=params,
            )
            for index in range(len(ordered_cluster_stats))
        ]

    sequence_indices = [int(entry["cluster_index"]) for entry in ordered_cluster_stats]
    groups = [
        _build_group_from_segment_evaluation(
            evaluation,
            validation_reason="dp_partition_selected",
            sequence_indices=sequence_indices,
        )
        for evaluation in partition
    ]
    return groups


def _ordered_track_cluster_stats(
    node_ids: Sequence[int],
    feature_map: dict[int, ClusterFeature],
    origin: np.ndarray,
    span_direction: np.ndarray,
    lateral_direction: np.ndarray,
) -> list[dict[str, float | int]]:
    stats: list[dict[str, float | int]] = []
    for node_id in node_ids:
        feature = feature_map[int(node_id)]
        u_values, v_values, _ = _project_points(
            feature.points,
            origin,
            span_direction,
            lateral_direction,
        )
        stats.append(
            {
                "cluster_index": int(node_id),
                "u_min_m": float(np.min(u_values)) if u_values.size else 0.0,
                "u_max_m": float(np.max(u_values)) if u_values.size else 0.0,
                "u_center_m": float(_robust_center(u_values)) if u_values.size else 0.0,
                "v_center_m": float(_robust_center(v_values)) if v_values.size else 0.0,
                "v_spread_m": float(_robust_spread(v_values)) if v_values.size else 0.0,
            }
        )
    stats.sort(key=lambda entry: (float(entry["u_center_m"]), int(entry["cluster_index"])))
    return stats


def _precompute_segment_evaluations(
    ordered_cluster_stats: Sequence[dict[str, float | int]],
    component_edges: Sequence[CandidateEdge],
    feature_map: dict[int, ClusterFeature],
    track_feature: ClusterFeature,
    params: WireStitchingParameters,
) -> list[list[SpanSegmentEvaluation | None]]:
    cluster_count = len(ordered_cluster_stats)
    table: list[list[SpanSegmentEvaluation | None]] = [
        [None for _ in range(cluster_count)]
        for _ in range(cluster_count)
    ]
    for start_index in range(cluster_count):
        max_end = min(cluster_count, start_index + params.max_clusters_per_span_candidate)
        for end_index in range(start_index, max_end):
            table[start_index][end_index] = _evaluate_span_segment(
                ordered_cluster_stats,
                start_index=start_index,
                end_index=end_index,
                component_edges=component_edges,
                feature_map=feature_map,
                track_feature=track_feature,
                params=params,
            )
    return table


def _evaluate_span_segment(
    ordered_cluster_stats: Sequence[dict[str, float | int]],
    *,
    start_index: int,
    end_index: int,
    component_edges: Sequence[CandidateEdge],
    feature_map: dict[int, ClusterFeature],
    track_feature: ClusterFeature,
    params: WireStitchingParameters,
) -> SpanSegmentEvaluation:
    segment_stats = list(ordered_cluster_stats[start_index : end_index + 1])
    segment_cluster_indices = [int(entry["cluster_index"]) for entry in segment_stats]
    segment_points = np.vstack([feature_map[index].points for index in segment_cluster_indices])
    segment_feature = _compute_cluster_feature(
        cluster_index=-1,
        points=segment_points,
        params=params,
        forced_span_direction=track_feature.span_direction,
    )
    baseline_rmse_m = _baseline_rmse([feature_map[index] for index in segment_cluster_indices], params)
    segment_rmse_m = float(segment_feature.catenary_fit.rmse_m or math.inf)
    rmse_increase_m = float(segment_rmse_m - baseline_rmse_m) if math.isfinite(segment_rmse_m) else math.inf
    rmse_ratio = (
        float(segment_rmse_m / max(baseline_rmse_m, params.min_baseline_rmse_m))
        if math.isfinite(segment_rmse_m)
        else math.inf
    )

    cluster_track_centers = [float(entry["v_center_m"]) for entry in segment_stats]
    cluster_track_spreads = [float(entry["v_spread_m"]) for entry in segment_stats]
    shared_track_center_v = float(np.median(cluster_track_centers)) if cluster_track_centers else 0.0
    max_cluster_track_deviation_m = float(
        max((abs(value - shared_track_center_v) for value in cluster_track_centers), default=0.0)
    )
    max_cluster_track_deviation_norm = float(
        max(
            (
                abs(value - shared_track_center_v) / max(spread, params.min_baseline_rmse_m, _EPS)
                for value, spread in zip(cluster_track_centers, cluster_track_spreads)
            ),
            default=0.0,
        )
    )

    gap_values = [
        max(0.0, float(segment_stats[offset + 1]["u_min_m"]) - float(segment_stats[offset]["u_max_m"]))
        for offset in range(len(segment_stats) - 1)
    ]
    total_internal_gap_m = float(sum(gap_values))
    max_internal_gap_m = float(max(gap_values, default=0.0))
    short_span_penalty = float(
        max(params.min_span_for_catenary_m - segment_feature.projected_span_length_m, 0.0)
        / max(params.min_span_for_catenary_m, _EPS)
    )

    track_term = _weighted_normalized_cost(
        values={
            "track_spread": segment_feature.v_spread_m / max(params.max_span_track_spread_m, _EPS),
            "track_rmse": segment_feature.track_rmse_m / max(params.max_span_track_rmse_m, _EPS),
            "track_dev": max_cluster_track_deviation_m / max(params.max_span_cluster_track_deviation_m, _EPS),
            "track_dev_norm": max_cluster_track_deviation_norm / max(params.max_span_cluster_track_deviation_norm, _EPS),
        },
        weights={
            "track_spread": 1.0,
            "track_rmse": 1.0,
            "track_dev": 1.0,
            "track_dev_norm": 1.0,
        },
    )

    cost = float(params.span_partition_base_cost)
    if len(segment_cluster_indices) == 1:
        cost += float(params.singleton_span_extra_cost)

    if segment_feature.catenary_fit.status == "ok" and math.isfinite(segment_rmse_m):
        cost += params.segment_rmse_cost_weight * (
            segment_rmse_m / max(params.max_span_catenary_rmse_m, _EPS)
        )
        cost += params.segment_rmse_increase_cost_weight * (
            max(rmse_increase_m, 0.0) / max(params.max_span_rmse_increase_m, _EPS)
        )
    else:
        cost += params.segment_rmse_cost_weight

    cost += params.segment_gap_cost_weight * (
        total_internal_gap_m / max(params.max_projected_gap_m, _EPS)
    )
    cost += params.segment_short_span_cost_weight * short_span_penalty
    cost += params.segment_track_cost_weight * track_term

    valid = (
        segment_feature.catenary_fit.status == "ok"
        and segment_feature.plane_rms_m <= params.max_component_plane_rms_m
        and segment_rmse_m <= params.max_span_catenary_rmse_m
        and rmse_increase_m <= params.max_span_rmse_increase_m
        and rmse_ratio <= params.max_span_rmse_ratio
        and segment_feature.v_spread_m <= params.max_span_track_spread_m
        and segment_feature.track_rmse_m <= params.max_span_track_rmse_m
        and max_cluster_track_deviation_m <= params.max_span_cluster_track_deviation_m
        and max_cluster_track_deviation_norm <= params.max_span_cluster_track_deviation_norm
    )

    status = "accepted_span_candidate"
    if len(segment_cluster_indices) == 1 and not valid:
        valid = True
        cost += float(params.singleton_invalid_fit_cost)
        status = "singleton_fallback"
    elif not valid:
        if segment_feature.catenary_fit.status != "ok":
            status = "invalid_catenary_fit"
        elif segment_rmse_m > params.max_span_catenary_rmse_m:
            status = "invalid_catenary_rmse"
        elif rmse_increase_m > params.max_span_rmse_increase_m:
            status = "invalid_rmse_increase"
        elif rmse_ratio > params.max_span_rmse_ratio:
            status = "invalid_rmse_ratio"
        elif max_cluster_track_deviation_m > params.max_span_cluster_track_deviation_m:
            status = "invalid_track_center"
        elif segment_feature.v_spread_m > params.max_span_track_spread_m:
            status = "invalid_track_spread"
        else:
            status = "invalid_span_candidate"

    accepted_edge_indices = sorted(
        edge.edge_index
        for edge in component_edges
        if edge.cluster_a in segment_cluster_indices and edge.cluster_b in segment_cluster_indices
    )

    return SpanSegmentEvaluation(
        start_index=start_index,
        end_index=end_index,
        source_cluster_indices=segment_cluster_indices,
        points=segment_points,
        feature=segment_feature,
        accepted_edge_indices=accepted_edge_indices,
        valid=bool(valid),
        status=status,
        cost=float(cost),
        point_count=int(segment_points.shape[0]),
        cluster_count=int(len(segment_cluster_indices)),
        baseline_local_rmse_m=float(baseline_rmse_m),
        rmse_increase_m=_maybe_float(rmse_increase_m),
        rmse_ratio=_maybe_float(rmse_ratio),
        shared_track_center_v_m=float(shared_track_center_v),
        track_spread_m=float(segment_feature.v_spread_m),
        track_rmse_m=float(segment_feature.track_rmse_m),
        max_cluster_track_deviation_m=float(max_cluster_track_deviation_m),
        max_cluster_track_deviation_norm=float(max_cluster_track_deviation_norm),
        total_internal_gap_m=float(total_internal_gap_m),
        max_internal_gap_m=float(max_internal_gap_m),
        short_span_penalty=float(short_span_penalty),
    )


def _solve_span_partition(
    segment_table: Sequence[Sequence[SpanSegmentEvaluation | None]],
) -> list[SpanSegmentEvaluation]:
    cluster_count = len(segment_table)
    if cluster_count == 0:
        return []

    dp_cost = [math.inf] * (cluster_count + 1)
    dp_segments = [math.inf] * (cluster_count + 1)
    previous_index: list[int | None] = [None] * (cluster_count + 1)
    previous_segment: list[SpanSegmentEvaluation | None] = [None] * (cluster_count + 1)
    dp_cost[0] = 0.0
    dp_segments[0] = 0

    for end_index in range(1, cluster_count + 1):
        for start_index in range(end_index):
            segment = segment_table[start_index][end_index - 1]
            if segment is None or not segment.valid or not math.isfinite(segment.cost):
                continue
            if not math.isfinite(dp_cost[start_index]):
                continue
            candidate_cost = dp_cost[start_index] + segment.cost
            candidate_segment_count = dp_segments[start_index] + 1
            current_key = (dp_cost[end_index], dp_segments[end_index], -(previous_index[end_index] or 0))
            candidate_key = (candidate_cost, candidate_segment_count, -start_index)
            if candidate_key < current_key:
                dp_cost[end_index] = candidate_cost
                dp_segments[end_index] = candidate_segment_count
                previous_index[end_index] = start_index
                previous_segment[end_index] = segment

    if previous_segment[cluster_count] is None:
        return []

    partition: list[SpanSegmentEvaluation] = []
    cursor = cluster_count
    while cursor > 0:
        segment = previous_segment[cursor]
        start_index = previous_index[cursor]
        if segment is None or start_index is None:
            return []
        partition.append(segment)
        cursor = start_index
    partition.reverse()
    return partition


def _build_group_from_segment_evaluation(
    segment: SpanSegmentEvaluation,
    *,
    validation_reason: str,
    sequence_indices: Sequence[int],
) -> StitchedWireGroup:
    return StitchedWireGroup(
        group_index=-1,
        source_cluster_indices=list(segment.source_cluster_indices),
        points=segment.points,
        feature=segment.feature,
        validation=segment.to_validation_dict(
            validation_reason=validation_reason,
            sequence_indices=sequence_indices,
        ),
        accepted_edge_indices=list(segment.accepted_edge_indices),
    )


def _compute_cluster_feature(
    cluster_index: int,
    points: np.ndarray,
    params: WireStitchingParameters,
    forced_span_direction: np.ndarray | None = None,
) -> ClusterFeature:
    clean_points = np.asarray(points, dtype=np.float64)
    if clean_points.ndim != 2 or clean_points.shape[1] != 3:
        raise ValueError("Cluster points must have shape (N, 3).")

    centroid = clean_points.mean(axis=0)
    bbox_min = clean_points.min(axis=0)
    bbox_max = clean_points.max(axis=0)
    principal_direction = _principal_direction(clean_points)
    span_direction = (
        _canonicalize_direction(np.asarray(forced_span_direction, dtype=np.float64))
        if forced_span_direction is not None
        else _horizontal_span_direction(principal_direction, clean_points)
    )
    plane_normal = _plane_normal_from_span(span_direction)
    lateral_direction = plane_normal
    vertical_direction = _vertical_in_plane_direction(span_direction, lateral_direction)
    along, lateral, _ = _project_points(clean_points, centroid, span_direction, lateral_direction)
    plane_distances = lateral
    plane_rms_m = float(np.sqrt(np.mean(np.square(plane_distances)))) if clean_points.shape[0] else 0.0

    projected_span_length_m = float(np.max(along) - np.min(along)) if along.size else 0.0
    order = np.argsort(along)
    endpoint_count = max(1, min(params.endpoint_sample_size, clean_points.shape[0]))
    start_point = clean_points[order[:endpoint_count]].mean(axis=0)
    end_point = clean_points[order[-endpoint_count:]].mean(axis=0)
    u_min_m = float(np.min(along)) if along.size else 0.0
    u_max_m = float(np.max(along)) if along.size else 0.0
    v_center_m = float(_robust_center(lateral)) if lateral.size else 0.0
    v_mad_m = float(_robust_mad(lateral)) if lateral.size else 0.0
    v_spread_m = float(_robust_spread(lateral)) if lateral.size else 0.0
    centered_lateral = lateral - v_center_m
    track_rmse_m = float(np.sqrt(np.mean(np.square(centered_lateral)))) if lateral.size else 0.0
    track_p95_abs_m = float(np.percentile(np.abs(centered_lateral), 95.0)) if lateral.size else 0.0
    catenary_fit = _fit_catenary_from_points(
        clean_points,
        centroid,
        span_direction,
        params,
        lateral_direction=lateral_direction,
    )

    return ClusterFeature(
        cluster_index=cluster_index,
        points=clean_points,
        centroid=centroid,
        point_count=int(clean_points.shape[0]),
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        principal_direction=principal_direction,
        span_direction=span_direction,
        lateral_direction=lateral_direction,
        vertical_direction=vertical_direction,
        plane_point=centroid,
        plane_normal=plane_normal,
        plane_rms_m=plane_rms_m,
        projected_span_length_m=projected_span_length_m,
        start_point=start_point,
        end_point=end_point,
        u_min_m=u_min_m,
        u_max_m=u_max_m,
        v_center_m=v_center_m,
        v_spread_m=v_spread_m,
        v_mad_m=v_mad_m,
        track_rmse_m=track_rmse_m,
        track_p95_abs_m=track_p95_abs_m,
        catenary_fit=catenary_fit,
    )


def _fit_catenary_from_points(
    points: np.ndarray,
    origin: np.ndarray,
    span_direction: np.ndarray,
    params: WireStitchingParameters,
    lateral_direction: np.ndarray | None = None,
) -> CatenaryFitResult:
    if points.shape[0] < params.min_points_for_feature:
        return CatenaryFitResult(
            status="too_few_points",
            horizontal_span_m=0.0,
            point_count=int(points.shape[0]),
        )

    local_lateral = (
        _canonicalize_direction(np.asarray(lateral_direction, dtype=np.float64))
        if lateral_direction is not None
        else _cross_track_direction(span_direction)
    )
    along, _, vertical = _project_points(points, origin, span_direction, local_lateral)
    span_min_m = float(np.min(along))
    shifted_along = along - span_min_m
    horizontal_span_m = float(np.max(shifted_along) - np.min(shifted_along))
    if horizontal_span_m < params.min_span_for_catenary_m:
        return CatenaryFitResult(
            status="span_too_small",
            horizontal_span_m=horizontal_span_m,
            point_count=int(points.shape[0]),
            along_min_m=span_min_m,
        )

    local_vertical = np.asarray(vertical, dtype=np.float64)
    fit = _fit_shifted_catenary(shifted_along, local_vertical)
    if fit is None:
        return CatenaryFitResult(
            status="fit_failed",
            horizontal_span_m=horizontal_span_m,
            point_count=int(points.shape[0]),
            along_min_m=span_min_m,
        )

    a_m, x0_m, z0_m = fit
    predicted = _evaluate_shifted_catenary(shifted_along, a_m, x0_m, z0_m)
    residuals = local_vertical - predicted
    rmse_m = float(np.sqrt(np.mean(np.square(residuals))))
    residual_mean_abs_m = float(np.mean(np.abs(residuals)))

    return CatenaryFitResult(
        status="ok",
        horizontal_span_m=horizontal_span_m,
        point_count=int(points.shape[0]),
        rmse_m=rmse_m,
        residual_mean_abs_m=residual_mean_abs_m,
        a_m=float(a_m),
        x0_m=float(x0_m),
        z0_m=float(z0_m),
        along_min_m=span_min_m,
    )


def _fit_shifted_catenary(along: np.ndarray, z: np.ndarray) -> tuple[float, float, float] | None:
    x = np.asarray(along, dtype=np.float64)
    z_values = np.asarray(z, dtype=np.float64)
    if x.ndim != 1 or z_values.ndim != 1 or x.size != z_values.size or x.size < 6:
        return None

    order = np.argsort(x)
    x = x[order]
    z_values = z_values[order]
    horizontal_span_m = float(np.max(x) - np.min(x))
    if horizontal_span_m < _EPS:
        return None

    idx_min = int(np.argmin(z_values))
    x0_guess = float(np.clip(x[idx_min], 0.0, horizontal_span_m))
    support_z_guess = float(np.mean([z_values[0], z_values[-1]]))
    observed_sag_m = max(0.10, support_z_guess - float(np.min(z_values)))
    a_guess = max(horizontal_span_m / 2.0, (horizontal_span_m * horizontal_span_m) / (8.0 * observed_sag_m))
    z0_guess = float(np.min(z_values) - a_guess)

    try:
        params, _ = curve_fit(
            _evaluate_shifted_catenary,
            x,
            z_values,
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


def _evaluate_shifted_catenary(x_axis: np.ndarray, a_m: float, x0_m: float, z0_m: float) -> np.ndarray:
    safe_a = max(float(a_m), _EPS)
    arg = np.clip((np.asarray(x_axis, dtype=np.float64) - float(x0_m)) / safe_a, -50.0, 50.0)
    return safe_a * np.cosh(arg) + float(z0_m)


def _baseline_rmse(features: Iterable[ClusterFeature], params: WireStitchingParameters) -> float:
    rmses = [
        float(feature.catenary_fit.rmse_m)
        for feature in features
        if feature.catenary_fit.status == "ok" and feature.catenary_fit.rmse_m is not None
    ]
    if not rmses:
        return params.min_baseline_rmse_m
    return max(float(np.mean(rmses)), params.min_baseline_rmse_m)


def _catenary_log_a_shift(candidate_fit: CatenaryFitResult, features: Sequence[ClusterFeature]) -> float:
    if candidate_fit.a_m is None or candidate_fit.a_m <= _EPS:
        return math.inf
    local_a_values = [
        float(feature.catenary_fit.a_m)
        for feature in features
        if feature.catenary_fit.status == "ok" and feature.catenary_fit.a_m is not None and feature.catenary_fit.a_m > _EPS
    ]
    if not local_a_values:
        return 0.0
    baseline_a = float(np.mean(local_a_values))
    if baseline_a <= _EPS:
        return 0.0
    return float(abs(math.log(candidate_fit.a_m / baseline_a)))


def _principal_direction(points: np.ndarray) -> np.ndarray:
    centered = points - points.mean(axis=0)
    if centered.shape[0] < 2:
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)
    covariance = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    principal_vector = eigenvectors[:, int(np.argmax(eigenvalues))]
    return _canonicalize_direction(principal_vector)


def _horizontal_span_direction(principal_direction: np.ndarray, points: np.ndarray) -> np.ndarray:
    horizontal = np.array([principal_direction[0], principal_direction[1], 0.0], dtype=np.float64)
    if np.linalg.norm(horizontal) > _EPS:
        return _canonicalize_direction(horizontal)

    xy = points[:, :2] - points[:, :2].mean(axis=0)
    if xy.shape[0] >= 2 and not np.allclose(xy, 0.0):
        covariance = np.cov(xy, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        axis_xy = eigenvectors[:, int(np.argmax(eigenvalues))]
        return _canonicalize_direction(np.array([axis_xy[0], axis_xy[1], 0.0], dtype=np.float64))

    return np.array([1.0, 0.0, 0.0], dtype=np.float64)


def _plane_normal_from_span(span_direction: np.ndarray) -> np.ndarray:
    return _cross_track_direction(span_direction)


def _cross_track_direction(span_direction: np.ndarray) -> np.ndarray:
    lateral = np.cross(span_direction, _WORLD_UP)
    if np.linalg.norm(lateral) <= _EPS:
        lateral = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    return _canonicalize_direction(lateral)


def _vertical_in_plane_direction(span_direction: np.ndarray, lateral_direction: np.ndarray) -> np.ndarray:
    vertical = np.cross(lateral_direction, span_direction)
    if np.linalg.norm(vertical) <= _EPS:
        vertical = np.array(_WORLD_UP, copy=True)
    vertical = vertical / max(np.linalg.norm(vertical), _EPS)
    if float(np.dot(vertical, _WORLD_UP)) < 0.0:
        vertical = -vertical
    return vertical


def _project_points(
    points: np.ndarray,
    origin: np.ndarray,
    span_direction: np.ndarray,
    lateral_direction: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    centered = np.asarray(points, dtype=np.float64) - np.asarray(origin, dtype=np.float64)
    span_axis = _canonicalize_direction(span_direction)
    lateral_axis = _canonicalize_direction(lateral_direction)
    vertical_axis = _vertical_in_plane_direction(span_axis, lateral_axis)
    u = centered @ span_axis
    v = centered @ lateral_axis
    w = centered @ vertical_axis
    return u, v, w


def _robust_center(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    return float(np.median(np.asarray(values, dtype=np.float64)))


def _robust_mad(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    centered = np.abs(np.asarray(values, dtype=np.float64) - np.median(values))
    return float(np.median(centered))


def _robust_spread(values: np.ndarray) -> float:
    array = np.asarray(values, dtype=np.float64)
    if array.size <= 1:
        return 0.0
    mad_sigma = 1.4826 * _robust_mad(array)
    std_sigma = float(np.std(array))
    return float(max(mad_sigma, std_sigma))


def _interval_overlap(interval_a: tuple[float, float], interval_b: tuple[float, float]) -> float:
    return float(max(0.0, min(interval_a[1], interval_b[1]) - max(interval_a[0], interval_b[0])))


def _project_cluster_track_stats(
    node_ids: Sequence[int],
    feature_map: dict[int, ClusterFeature],
    origin: np.ndarray,
    span_direction: np.ndarray,
    lateral_direction: np.ndarray,
) -> list[dict[str, float | int]]:
    stats: list[dict[str, float | int]] = []
    for node_id in node_ids:
        feature = feature_map[int(node_id)]
        u_values, v_values, _ = _project_points(
            feature.points,
            origin,
            span_direction,
            lateral_direction,
        )
        stats.append(
            {
                "cluster_index": int(node_id),
                "u_min_m": float(np.min(u_values)) if u_values.size else 0.0,
                "u_max_m": float(np.max(u_values)) if u_values.size else 0.0,
                "v_center_m": float(_robust_center(v_values)) if v_values.size else 0.0,
                "v_spread_m": float(_robust_spread(v_values)) if v_values.size else 0.0,
                "v_mad_m": float(_robust_mad(v_values)) if v_values.size else 0.0,
            }
        )
    return stats


def _detect_v_bimodality(
    points: np.ndarray,
    origin: np.ndarray,
    span_direction: np.ndarray,
    lateral_direction: np.ndarray,
    params: WireStitchingParameters,
    *,
    ordered_indices: Sequence[int] | None = None,
    feature_map: dict[int, ClusterFeature] | None = None,
) -> dict[str, Any]:
    _, v_values, _ = _project_points(points, origin, span_direction, lateral_direction)
    if v_values.size < params.min_points_for_bimodality:
        return {
            "is_bimodal": False,
            "largest_gap_m": 0.0,
            "center_separation_m": 0.0,
            "cluster_split": None,
        }

    sorted_v = np.sort(np.asarray(v_values, dtype=np.float64))
    gaps = np.diff(sorted_v)
    if gaps.size == 0:
        return {
            "is_bimodal": False,
            "largest_gap_m": 0.0,
            "center_separation_m": 0.0,
            "cluster_split": None,
        }

    gap_index = int(np.argmax(gaps))
    largest_gap_m = float(gaps[gap_index])
    threshold_v = float(0.5 * (sorted_v[gap_index] + sorted_v[gap_index + 1]))
    left_values = sorted_v[: gap_index + 1]
    right_values = sorted_v[gap_index + 1 :]
    if left_values.size < 3 or right_values.size < 3:
        return {
            "is_bimodal": False,
            "largest_gap_m": largest_gap_m,
            "center_separation_m": 0.0,
            "threshold_v_m": threshold_v,
            "cluster_split": None,
        }

    left_center = _robust_center(left_values)
    right_center = _robust_center(right_values)
    left_spread = _robust_spread(left_values)
    right_spread = _robust_spread(right_values)
    center_separation_m = float(abs(right_center - left_center))
    gap_scale_threshold = params.bimodal_gap_scale * max(left_spread, right_spread, params.min_baseline_rmse_m, _EPS)
    is_bimodal = bool(
        largest_gap_m >= params.bimodal_min_gap_m
        and center_separation_m >= params.bimodal_min_center_separation_m
        and largest_gap_m >= gap_scale_threshold
    )

    cluster_split: list[list[int]] | None = None
    if is_bimodal and ordered_indices is not None and feature_map is not None:
        left_nodes: list[int] = []
        right_nodes: list[int] = []
        for node_id in ordered_indices:
            feature = feature_map[int(node_id)]
            _, cluster_v, _ = _project_points(feature.points, origin, span_direction, lateral_direction)
            cluster_center = _robust_center(cluster_v)
            if cluster_center <= threshold_v:
                left_nodes.append(int(node_id))
            else:
                right_nodes.append(int(node_id))
        if left_nodes and right_nodes:
            cluster_split = [sorted(left_nodes), sorted(right_nodes)]
        else:
            is_bimodal = False

    return {
        "is_bimodal": is_bimodal,
        "largest_gap_m": largest_gap_m,
        "center_separation_m": center_separation_m,
        "threshold_v_m": threshold_v,
        "left_center_v_m": float(left_center),
        "right_center_v_m": float(right_center),
        "left_spread_m": float(left_spread),
        "right_spread_m": float(right_spread),
        "cluster_split": cluster_split,
    }


def _connected_components(node_ids: Sequence[int], edges: Sequence[CandidateEdge]) -> list[list[int]]:
    adjacency: dict[int, set[int]] = {int(node_id): set() for node_id in node_ids}
    for edge in edges:
        adjacency[int(edge.cluster_a)].add(int(edge.cluster_b))
        adjacency[int(edge.cluster_b)].add(int(edge.cluster_a))

    components: list[list[int]] = []
    visited: set[int] = set()
    for node_id in sorted(adjacency):
        if node_id in visited:
            continue
        stack = [node_id]
        component: list[int] = []
        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)
            component.append(current)
            stack.extend(sorted(adjacency[current] - visited, reverse=True))
        components.append(sorted(component))
    return components


def _interval_and_endpoints(
    points: np.ndarray,
    origin: np.ndarray,
    axis: np.ndarray,
    endpoint_sample_size: int,
) -> tuple[tuple[float, float], np.ndarray, np.ndarray]:
    along = (points - origin) @ axis
    order = np.argsort(along)
    endpoint_count = max(1, min(endpoint_sample_size, points.shape[0]))
    start_point = points[order[:endpoint_count]].mean(axis=0)
    end_point = points[order[-endpoint_count:]].mean(axis=0)
    return (float(np.min(along)), float(np.max(along))), start_point, end_point


def _pair_gap_and_endpoint_distance(
    interval_a: tuple[float, float],
    interval_b: tuple[float, float],
    start_a: np.ndarray,
    end_a: np.ndarray,
    start_b: np.ndarray,
    end_b: np.ndarray,
) -> tuple[float, float]:
    if interval_a[1] <= interval_b[0]:
        gap = float(interval_b[0] - interval_a[1])
        endpoint_distance = float(np.linalg.norm(end_a - start_b))
        return gap, endpoint_distance
    if interval_b[1] <= interval_a[0]:
        gap = float(interval_a[0] - interval_b[1])
        endpoint_distance = float(np.linalg.norm(end_b - start_a))
        return gap, endpoint_distance

    endpoint_distance = min(
        float(np.linalg.norm(start_a - start_b)),
        float(np.linalg.norm(start_a - end_b)),
        float(np.linalg.norm(end_a - start_b)),
        float(np.linalg.norm(end_a - end_b)),
    )
    return 0.0, endpoint_distance


def _average_direction(direction_a: np.ndarray, direction_b: np.ndarray) -> np.ndarray:
    aligned_b = _align_direction(direction_b, direction_a)
    combined = direction_a + aligned_b
    if np.linalg.norm(combined) <= _EPS:
        return _canonicalize_direction(direction_a)
    return _canonicalize_direction(combined)


def _align_direction(direction: np.ndarray, reference: np.ndarray) -> np.ndarray:
    return np.asarray(direction, dtype=np.float64) if np.dot(direction, reference) >= 0.0 else -np.asarray(direction, dtype=np.float64)


def _canonicalize_direction(direction: np.ndarray) -> np.ndarray:
    vector = np.asarray(direction, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm <= _EPS:
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)
    unit = vector / norm
    for component in unit:
        if abs(component) > 1e-8:
            if component < 0.0:
                unit = -unit
            break
    return unit


def _point_to_plane_distance(point: np.ndarray, plane_point: np.ndarray, plane_normal: np.ndarray) -> float:
    return float(abs(np.dot(point - plane_point, plane_normal)))


def _weighted_normalized_cost(values: dict[str, float], weights: dict[str, float]) -> float:
    total_weight = 0.0
    total_cost = 0.0
    for key, value in values.items():
        weight = float(weights.get(key, 0.0))
        if weight <= 0.0:
            continue
        total_cost += weight * max(float(value), 0.0)
        total_weight += weight
    return float(total_cost / max(total_weight, _EPS))


def _vector_to_list(vector: np.ndarray) -> list[float]:
    return [float(value) for value in np.asarray(vector, dtype=np.float64).tolist()]


def _maybe_float(value: float | None) -> float | None:
    if value is None:
        return None
    if not math.isfinite(float(value)):
        return None
    return float(value)


def _to_serializable(value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): _to_serializable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_serializable(item) for item in value]
    return str(value)
