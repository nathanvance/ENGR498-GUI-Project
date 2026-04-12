from __future__ import annotations

import numpy as np

from wire_extraction.stitching import WireStitchingParameters, stitch_wire_clusters


def _make_catenary_segment(
    x_start: float,
    x_stop: float,
    *,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
    sample_count: int = 40,
    a_m: float = 90.0,
    x0_m: float = 14.0,
    z0_m: float = -85.0,
) -> np.ndarray:
    x = np.linspace(x_start, x_stop, sample_count, dtype=np.float64)
    y = np.full_like(x, y_offset)
    z = (a_m * np.cosh((x - x0_m) / a_m)) + z0_m + z_offset
    return np.column_stack((x, y, z))


def _make_supported_span_fragment(
    x_start: float,
    x_stop: float,
    *,
    support_start: float,
    support_end: float,
    y_offset: float = 0.0,
    support_height: float = 0.0,
    sample_count: int = 40,
    a_m: float = 22.0,
) -> np.ndarray:
    span_mid = 0.5 * (support_start + support_end)
    support_offset = support_height - (a_m * np.cosh((support_end - span_mid) / a_m))
    x = np.linspace(x_start, x_stop, sample_count, dtype=np.float64)
    y = np.full_like(x, y_offset)
    z = (a_m * np.cosh((x - span_mid) / a_m)) + support_offset
    return np.column_stack((x, y, z))


def test_stitches_same_wire_fragments_and_rejects_parallel_wire() -> None:
    fragment_a = _make_catenary_segment(0.0, 9.0)
    fragment_b = _make_catenary_segment(10.5, 21.0)
    nearby_parallel_wire = _make_catenary_segment(0.0, 21.0, y_offset=3.0)

    result = stitch_wire_clusters([fragment_a, fragment_b, nearby_parallel_wire], WireStitchingParameters())

    source_groups = sorted(tuple(group.source_cluster_indices) for group in result.groups)
    assert source_groups == [(0, 1), (2,)]

    accepted_pairs = {
        tuple(sorted((edge.cluster_a, edge.cluster_b)))
        for edge in result.candidate_edges
        if edge.accepted_for_graph
    }
    assert accepted_pairs == {(0, 1)}


def test_large_gap_prevents_stitch_even_when_direction_matches() -> None:
    fragment_a = _make_catenary_segment(0.0, 8.0)
    fragment_b = _make_catenary_segment(24.0, 32.0)

    params = WireStitchingParameters(max_projected_gap_m=6.0)
    result = stitch_wire_clusters([fragment_a, fragment_b], params)

    source_groups = sorted(tuple(group.source_cluster_indices) for group in result.groups)
    assert source_groups == [(0,), (1,)]
    assert all(not edge.accepted_for_graph for edge in result.candidate_edges)


def test_close_parallel_wires_do_not_cross_merge_when_tracks_differ() -> None:
    wire_a_0 = _make_catenary_segment(0.0, 9.0, y_offset=0.00)
    wire_a_1 = _make_catenary_segment(10.5, 21.0, y_offset=0.00)
    wire_b_0 = _make_catenary_segment(0.0, 9.0, y_offset=0.35)
    wire_b_1 = _make_catenary_segment(10.5, 21.0, y_offset=0.35)

    result = stitch_wire_clusters([wire_a_0, wire_a_1, wire_b_0, wire_b_1], WireStitchingParameters())

    source_groups = sorted(tuple(group.source_cluster_indices) for group in result.groups)
    assert source_groups == [(0, 1), (2, 3)]

    accepted_pairs = {
        tuple(sorted((edge.cluster_a, edge.cluster_b)))
        for edge in result.candidate_edges
        if edge.accepted_for_graph
    }
    assert accepted_pairs == {(0, 1), (2, 3)}


def test_group_level_track_bimodality_splits_multi_track_component() -> None:
    wire_a_0 = _make_catenary_segment(0.0, 9.0, y_offset=0.00)
    wire_a_1 = _make_catenary_segment(10.5, 21.0, y_offset=0.00)
    wire_b_0 = _make_catenary_segment(0.0, 9.0, y_offset=0.32)
    wire_b_1 = _make_catenary_segment(10.5, 21.0, y_offset=0.32)

    params = WireStitchingParameters(
        max_lateral_offset_m=1.0,
        max_lateral_offset_norm=12.0,
        min_side_by_side_overlap_ratio=1.1,
        min_side_by_side_overlap_m=999.0,
        min_side_by_side_delta_v_m=999.0,
        min_side_by_side_delta_v_norm=999.0,
    )
    result = stitch_wire_clusters([wire_a_0, wire_a_1, wire_b_0, wire_b_1], params)

    source_groups = sorted(tuple(group.source_cluster_indices) for group in result.groups)
    assert source_groups == [(0, 1), (2, 3)]
    assert all(bool(group.validation["accepted"]) for group in result.groups)


def test_dp_partition_keeps_adjacent_support_spans_separate_on_same_track() -> None:
    left_0 = _make_supported_span_fragment(0.0, 7.5, support_start=0.0, support_end=10.0)
    left_1 = _make_supported_span_fragment(8.0, 9.95, support_start=0.0, support_end=10.0)
    right_0 = _make_supported_span_fragment(10.05, 12.0, support_start=10.0, support_end=20.0)
    right_1 = _make_supported_span_fragment(12.5, 20.0, support_start=10.0, support_end=20.0)

    params = WireStitchingParameters(
        max_pair_catenary_rmse_m=1.0,
        max_pair_rmse_increase_m=1.0,
        max_pair_rmse_ratio=20.0,
        edge_accept_cost=2.0,
        max_clusters_per_span_candidate=4,
    )
    result = stitch_wire_clusters([left_0, left_1, right_0, right_1], params)

    candidate_tracks = sorted(tuple(track) for track in result.candidate_tracks)
    assert candidate_tracks == [(0, 1, 2, 3)]

    source_groups = sorted(tuple(group.source_cluster_indices) for group in result.groups)
    assert source_groups == [(0, 1), (2, 3)]

    partitions = [
        (
            tuple(group.validation["ordered_sequence_indices"]),
            group.validation["segment_start_index"],
            group.validation["segment_end_index"],
        )
        for group in result.groups
    ]
    assert partitions == [((0, 1, 2, 3), 0, 1), ((0, 1, 2, 3), 2, 3)]
