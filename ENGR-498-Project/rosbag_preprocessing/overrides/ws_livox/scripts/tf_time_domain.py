"""Helpers for converting between query/header time and raw TF time."""

from __future__ import annotations


def derive_lookup_time_offset_sec(first_tf_sec: float | None, first_clock_sec: float | None) -> float:
    """Return the query/header -> TF offset when the two domains obviously differ."""
    if first_tf_sec is None or first_clock_sec is None:
        return 0.0
    if abs(float(first_clock_sec) - float(first_tf_sec)) > 60.0:
        return float(first_clock_sec) - float(first_tf_sec)
    return 0.0


def map_query_time_to_tf_domain(t_query_sec: float, lookup_time_offset_sec: float) -> float:
    """Convert a query/header timestamp into the TF lookup domain."""
    if lookup_time_offset_sec:
        return max(float(t_query_sec) - float(lookup_time_offset_sec), 0.0)
    return float(t_query_sec)


def map_tf_time_to_query_domain(t_tf_sec: float, lookup_time_offset_sec: float) -> float:
    """Convert a raw TF timestamp into the query/header timestamp domain."""
    if lookup_time_offset_sec:
        return float(t_tf_sec) + float(lookup_time_offset_sec)
    return float(t_tf_sec)
