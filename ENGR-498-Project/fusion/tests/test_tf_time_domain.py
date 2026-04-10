from __future__ import annotations

import sys
from pathlib import Path

import pytest

SCRIPT_DIR = (
    Path(__file__).resolve().parents[2]
    / "rosbag_preprocessing"
    / "overrides"
    / "ws_livox"
    / "scripts"
)
sys.path.insert(0, str(SCRIPT_DIR))

from tf_time_domain import (  # noqa: E402
    derive_lookup_time_offset_sec,
    map_query_time_to_tf_domain,
    map_tf_time_to_query_domain,
)


def test_no_lookup_offset_when_domains_already_match():
    assert derive_lookup_time_offset_sec(1718656564.5, 1718656564.6) == 0.0


def test_lookup_offset_detects_clock_vs_tf_domain_shift():
    offset = derive_lookup_time_offset_sec(407.400700825, 1718656564.647477)
    assert offset > 1e9


def test_query_time_maps_into_tf_domain_with_offset():
    lookup_offset = 1718656157.246776104
    tf_time = map_query_time_to_tf_domain(1718656565.0778642, lookup_offset)
    assert tf_time == pytest.approx(407.831088096064)


def test_tf_time_maps_back_into_query_domain_with_offset():
    lookup_offset = 1718656157.246776104
    query_time = map_tf_time_to_query_domain(407.831088096064, lookup_offset)
    assert query_time == 1718656565.0778642
