"""Unit tests for DenseTrajectory loading, sign continuity, SLERP, and query."""
from __future__ import annotations

import io
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest

# Allow importing from fusion/ without installing the package.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fuse_masks_to_slam import (
    DenseTrajectory,
    _enforce_quaternion_sign_continuity,
    _slerp_quaternion_xyzw,
    load_dense_trajectory,
    query_dense_trajectory,
    validate_dense_trajectory_time_range,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_traj(n: int = 10, dt: float = 0.010, t0: float = 1000.0) -> DenseTrajectory:
    """Synthetic straight-line trajectory with constant yaw."""
    ts = np.arange(n, dtype=np.float64) * dt + t0
    trans = np.zeros((n, 3), dtype=np.float64)
    trans[:, 0] = np.arange(n, dtype=np.float64) * 0.1  # 10 cm/sample forward in X
    # Identity quaternion for all poses.
    quats = np.tile([0.0, 0.0, 0.0, 1.0], (n, 1)).astype(np.float64)
    return DenseTrajectory(timestamps=ts, translations=trans, quaternions=quats)


def _csv_text(rows: list[dict]) -> str:
    header = "timestamp_sec,x,y,z,qx,qy,qz,qw,status"
    lines = [header]
    for r in rows:
        lines.append(
            f"{r['t']},{r.get('x',0)},{r.get('y',0)},{r.get('z',0)},"
            f"{r.get('qx',0)},{r.get('qy',0)},{r.get('qz',0)},{r.get('qw',1)},{r.get('status','OK')}"
        )
    return "\n".join(lines) + "\n"


def _write_csv(tmp_path: Path, rows: list[dict]) -> Path:
    p = tmp_path / "dense.csv"
    p.write_text(_csv_text(rows), encoding="utf-8")
    return p


# ---------------------------------------------------------------------------
# sign continuity
# ---------------------------------------------------------------------------


def test_sign_continuity_no_flips_needed():
    q = np.array([[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]], dtype=np.float64)
    _enforce_quaternion_sign_continuity(q)
    assert np.all(q[:, 3] > 0)


def test_sign_continuity_all_flipped():
    """Alternating sign flips should all be corrected."""
    q = np.array(
        [
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, -1.0],  # flipped
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, -1.0],  # flipped
        ],
        dtype=np.float64,
    )
    _enforce_quaternion_sign_continuity(q)
    # After correction every consecutive dot product should be >= 0.
    for i in range(1, len(q)):
        assert float(np.dot(q[i - 1], q[i])) >= 0.0, f"Sign flip remains at index {i}"


def test_sign_continuity_single_row():
    q = np.array([[0.0, 0.0, 0.0, 1.0]], dtype=np.float64)
    _enforce_quaternion_sign_continuity(q)  # Should not crash on single row.
    assert q[0, 3] == pytest.approx(1.0)


def test_sign_continuity_180_degree_rotation():
    """180 deg apart quaternions: after fixup dot product is still 0 but no negative."""
    # q1 = 180 deg rotation around Z: (0, 0, 1, 0)
    q = np.array([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, -1.0, 0.0]], dtype=np.float64)
    _enforce_quaternion_sign_continuity(q)
    # dot(q[0], q[1]) should be >= 0
    assert float(np.dot(q[0], q[1])) >= 0.0


# ---------------------------------------------------------------------------
# SLERP
# ---------------------------------------------------------------------------


def _norm(q: np.ndarray) -> float:
    return float(np.linalg.norm(q))


def test_slerp_alpha_zero():
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    q1 = np.array([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    result = _slerp_quaternion_xyzw(q0, q1, 0.0)
    np.testing.assert_allclose(result, q0, atol=1e-9)


def test_slerp_alpha_one():
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    q1 = np.array([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    result = _slerp_quaternion_xyzw(q0, q1, 1.0)
    np.testing.assert_allclose(result, q1, atol=1e-9)


def test_slerp_alpha_half_identity():
    """SLERP between identity and 90 deg around Z at alpha=0.5 to 45 deg around Z."""
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    angle = math.pi / 2
    q1 = np.array([0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])
    result = _slerp_quaternion_xyzw(q0, q1, 0.5)
    assert _norm(result) == pytest.approx(1.0, abs=1e-9)
    # Should be 45 deg rotation around Z.
    expected = np.array([0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8)])
    np.testing.assert_allclose(result, expected, atol=1e-9)


def test_slerp_identical_quaternions():
    q = np.array([0.0, 0.0, math.sin(0.3), math.cos(0.3)])
    result = _slerp_quaternion_xyzw(q, q, 0.5)
    np.testing.assert_allclose(result, q / np.linalg.norm(q), atol=1e-9)


def test_slerp_unit_norm_output():
    q0 = np.array([0.1, 0.2, 0.3, 0.9])
    q0 /= np.linalg.norm(q0)
    q1 = np.array([0.4, -0.1, 0.1, 0.9])
    q1 /= np.linalg.norm(q1)
    for alpha in (0.0, 0.25, 0.5, 0.75, 1.0):
        result = _slerp_quaternion_xyzw(q0, q1, alpha)
        assert _norm(result) == pytest.approx(1.0, abs=1e-9), f"norm != 1 at alpha={alpha}"


def test_slerp_sign_flipped_q1():
    """SLERP should still produce a correct short-arc result when q1 is sign-flipped."""
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    q1 = np.array([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    result_normal = _slerp_quaternion_xyzw(q0, q1, 0.5)
    result_flipped = _slerp_quaternion_xyzw(q0, -q1, 0.5)
    # The slerp function handles sign internally; results should be equivalent rotations.
    dot = abs(float(np.dot(result_normal, result_flipped)))
    assert dot == pytest.approx(1.0, abs=1e-9), "SLERP with flipped q1 gave different rotation"


# ---------------------------------------------------------------------------
# query_dense_trajectory
# ---------------------------------------------------------------------------


def test_query_before_start():
    traj = _make_traj()
    pose, details = query_dense_trajectory(traj, traj.timestamps[0] - 1.0)
    assert pose is None
    assert "drop_reason" in details


def test_query_after_end():
    traj = _make_traj()
    pose, details = query_dense_trajectory(traj, traj.timestamps[-1] + 1.0)
    assert pose is None
    assert "drop_reason" in details


def test_query_exactly_at_first():
    traj = _make_traj()
    pose, details = query_dense_trajectory(traj, float(traj.timestamps[0]))
    assert pose is not None
    np.testing.assert_allclose(pose.translation, traj.translations[0], atol=1e-12)


def test_query_exactly_at_last():
    traj = _make_traj()
    pose, details = query_dense_trajectory(traj, float(traj.timestamps[-1]))
    assert pose is not None
    np.testing.assert_allclose(pose.translation, traj.translations[-1], atol=1e-12)


def test_query_midpoint_translation():
    """At midpoint between two samples, translation should be the exact midpoint."""
    traj = _make_traj(n=3)
    t_mid = float((traj.timestamps[0] + traj.timestamps[1]) / 2)
    pose, details = query_dense_trajectory(traj, t_mid)
    assert pose is not None
    expected = (traj.translations[0] + traj.translations[1]) / 2
    np.testing.assert_allclose(pose.translation, expected, atol=1e-12)


def test_query_midpoint_quaternion_unit_norm():
    traj = _make_traj(n=5)
    t_mid = float((traj.timestamps[1] + traj.timestamps[2]) / 2)
    pose, details = query_dense_trajectory(traj, t_mid)
    assert pose is not None
    assert _norm(pose.quaternion_xyzw) == pytest.approx(1.0, abs=1e-9)


def test_query_diagnostics_schema():
    """Diagnostics dict must have the same keys as interpolate_pose_record output."""
    expected_keys = {"match_mode", "pose_index_lo", "pose_index_hi", "pose_time_lo", "pose_time_hi", "interp_alpha"}
    traj = _make_traj(n=5)
    t_mid = float((traj.timestamps[1] + traj.timestamps[2]) / 2)
    _, details = query_dense_trajectory(traj, t_mid)
    assert expected_keys.issubset(set(details.keys()))


def test_query_empty_trajectory():
    traj = DenseTrajectory(
        timestamps=np.empty(0, dtype=np.float64),
        translations=np.empty((0, 3), dtype=np.float64),
        quaternions=np.empty((0, 4), dtype=np.float64),
    )
    pose, details = query_dense_trajectory(traj, 1000.0)
    assert pose is None
    assert "drop_reason" in details


# ---------------------------------------------------------------------------
# load_dense_trajectory
# ---------------------------------------------------------------------------


def test_load_basic(tmp_path: Path):
    rows = [
        {"t": 1000.0, "x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "status": "OK"},
        {"t": 1000.01, "x": 0.1, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "status": "OK"},
        {"t": 1000.02, "x": 0.2, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "status": "OK"},
    ]
    p = _write_csv(tmp_path, rows)
    traj = load_dense_trajectory(p)
    assert traj.timestamps.size == 3
    assert float(traj.timestamps[0]) == pytest.approx(1000.0)
    assert float(traj.translations[1, 0]) == pytest.approx(0.1)


def test_load_skips_non_ok(tmp_path: Path):
    rows = [
        {"t": 1000.0, "status": "OK"},
        {"t": 1000.01, "status": "NO_TF"},
        {"t": 1000.02, "status": "EXTRAPOLATION"},
        {"t": 1000.03, "status": "OK"},
    ]
    p = _write_csv(tmp_path, rows)
    traj = load_dense_trajectory(p)
    assert traj.timestamps.size == 2


def test_load_skips_invalid_quaternion(tmp_path: Path):
    rows = [
        {"t": 1000.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "status": "OK"},
        {"t": 1000.01, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0, "status": "OK"},  # zero-norm
        {"t": 1000.02, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "status": "OK"},
    ]
    p = _write_csv(tmp_path, rows)
    traj = load_dense_trajectory(p)
    assert traj.timestamps.size == 2


def test_load_deduplicates(tmp_path: Path):
    rows = [
        {"t": 1000.0, "status": "OK"},
        {"t": 1000.0, "status": "OK"},  # duplicate
        {"t": 1000.01, "status": "OK"},
    ]
    p = _write_csv(tmp_path, rows)
    traj = load_dense_trajectory(p)
    assert traj.timestamps.size == 2


def test_load_sorts_timestamps(tmp_path: Path):
    rows = [
        {"t": 1000.02, "x": 2.0, "status": "OK"},
        {"t": 1000.00, "x": 0.0, "status": "OK"},
        {"t": 1000.01, "x": 1.0, "status": "OK"},
    ]
    p = _write_csv(tmp_path, rows)
    traj = load_dense_trajectory(p)
    assert list(traj.timestamps) == pytest.approx([1000.00, 1000.01, 1000.02])


def test_load_normalizes_quaternion(tmp_path: Path):
    """Quaternions that are close to unit length (within [0.5, 2.0] norm_sq) are accepted
    and then normalized to exactly unit length.  Wildly unnormalized values are rejected."""
    # Slightly off-unit (norm_sq ~ 1.002) — should be accepted and normalized.
    qw = math.sqrt(1.002)
    rows = [
        {"t": 1000.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": round(qw, 10), "status": "OK"},
    ]
    p = _write_csv(tmp_path, rows)
    traj = load_dense_trajectory(p)
    assert traj.timestamps.size == 1
    assert _norm(traj.quaternions[0]) == pytest.approx(1.0, abs=1e-9)


def test_load_enforces_sign_continuity(tmp_path: Path):
    rows = [
        {"t": 1000.00, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "status": "OK"},
        {"t": 1000.01, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": -1.0, "status": "OK"},  # flipped
        {"t": 1000.02, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "status": "OK"},
    ]
    p = _write_csv(tmp_path, rows)
    traj = load_dense_trajectory(p)
    for i in range(1, len(traj.quaternions)):
        assert float(np.dot(traj.quaternions[i - 1], traj.quaternions[i])) >= 0.0


def test_load_empty_csv_raises(tmp_path: Path):
    p = _write_csv(tmp_path, [])
    with pytest.raises(ValueError):
        load_dense_trajectory(p)


def test_load_all_non_ok_raises(tmp_path: Path):
    rows = [
        {"t": 1000.0, "status": "NO_TF"},
        {"t": 1000.01, "status": "EXTRAPOLATION"},
    ]
    p = _write_csv(tmp_path, rows)
    with pytest.raises(ValueError):
        load_dense_trajectory(p)


# ---------------------------------------------------------------------------
# Integration: load to query round-trip
# ---------------------------------------------------------------------------


def test_roundtrip_interpolation(tmp_path: Path):
    """Load a synthetic CSV and verify that querying at a known midpoint gives the right translation."""
    rows = [
        {"t": 0.000, "x": 0.0, "y": 0.0, "z": 0.0, "qx": 0, "qy": 0, "qz": 0, "qw": 1, "status": "OK"},
        {"t": 0.010, "x": 1.0, "y": 0.0, "z": 0.0, "qx": 0, "qy": 0, "qz": 0, "qw": 1, "status": "OK"},
        {"t": 0.020, "x": 2.0, "y": 0.0, "z": 0.0, "qx": 0, "qy": 0, "qz": 0, "qw": 1, "status": "OK"},
    ]
    p = _write_csv(tmp_path, rows)
    traj = load_dense_trajectory(p)
    pose, details = query_dense_trajectory(traj, 0.005)
    assert pose is not None
    assert float(pose.translation[0]) == pytest.approx(0.5, abs=1e-9)
    assert details["interp_alpha"] == pytest.approx(0.5, abs=1e-9)


def test_roundtrip_slerp_yaw(tmp_path: Path):
    """Verify SLERP of a 90 deg yaw rotation interpolates to 45 deg at alpha=0.5."""
    angle_half = math.pi / 4  # 90 deg rotation needs half-angle
    rows = [
        {"t": 0.000, "x": 0, "y": 0, "z": 0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "status": "OK"},
        {
            "t": 0.010,
            "x": 0,
            "y": 0,
            "z": 0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": round(math.sin(angle_half), 10),
            "qw": round(math.cos(angle_half), 10),
            "status": "OK",
        },
    ]
    p = _write_csv(tmp_path, rows)
    traj = load_dense_trajectory(p)
    pose, _ = query_dense_trajectory(traj, 0.005)
    assert pose is not None
    expected_qz = math.sin(math.pi / 8)  # 45 deg half-angle
    expected_qw = math.cos(math.pi / 8)
    assert float(pose.quaternion_xyzw[2]) == pytest.approx(expected_qz, abs=1e-6)
    assert float(pose.quaternion_xyzw[3]) == pytest.approx(expected_qw, abs=1e-6)


def test_validate_dense_trajectory_time_range_accepts_unix_overlap(tmp_path: Path):
    traj = _make_traj(n=40, dt=0.01, t0=1718656564.60)
    frame_times = np.asarray([1718656564.70, 1718656564.80, 1718656564.90], dtype=np.float64)
    validate_dense_trajectory_time_range(traj, frame_times, dense_path=tmp_path / "dense.csv")


def test_validate_dense_trajectory_time_range_rejects_tf_domain_dense(tmp_path: Path):
    traj = _make_traj(n=5, dt=0.01, t0=407.40)
    frame_times = np.asarray([1718656564.70, 1718656564.80, 1718656564.90], dtype=np.float64)
    with pytest.raises(ValueError, match="Regenerate pose recovery outputs"):
        validate_dense_trajectory_time_range(traj, frame_times, dense_path=tmp_path / "dense.csv")


def test_validate_dense_trajectory_time_range_rejects_non_overlapping_unix_ranges(tmp_path: Path):
    traj = _make_traj(n=5, dt=0.01, t0=1718656564.60)
    frame_times = np.asarray([1718656664.70, 1718656664.80, 1718656664.90], dtype=np.float64)
    with pytest.raises(ValueError, match="do not overlap"):
        validate_dense_trajectory_time_range(traj, frame_times, dense_path=tmp_path / "dense.csv")
