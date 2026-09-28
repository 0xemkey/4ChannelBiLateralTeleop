#!/usr/bin/env python3
"""Phase 2 (2026-09-09): dedicated hardware-independent test for C2/C3
absolute-mode frame symmetry. Pure Python, no rclpy/hardware dependency.

    C3: slave_target  = master_position   (for absolute axes)
    C2: master_target = slave_position    (for absolute axes)

Neither must depend on left_anchor/right_anchor for absolute axes, even
when the two anchors are deliberately set tens of mm apart. Also checks
mixed absolute/relative configuration (only some axes absolute).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from bridge_master_slave import compute_c2_target, compute_c3_target  # noqa: E402

IDENTITY_AXIS_MAP = [(0, 1.0), (1, 1.0), (2, 1.0)]
AXIS_SCALE = [1.0, 1.0, 1.0]
MAX_OFFSET = [0.12, 0.19, 0.15]
LEFT_MAX_OFFSET_M = 0.08
DEADBAND_M = 0.0002

# Deliberately very different anchors, ||A-B|| > 50mm on every axis.
LEFT_ANCHOR = [0.10, -0.20, 0.05]
RIGHT_ANCHOR = [-0.05, 0.10, -0.15]
ANCHOR_GAP = math.sqrt(sum((LEFT_ANCHOR[i] - RIGHT_ANCHOR[i]) ** 2 for i in range(3)))
assert ANCHOR_GAP > 0.050, f"test fixture must use a >50mm anchor gap, got {ANCHOR_GAP*1000:.1f}mm"

MASTER_POS = [0.02, -0.03, 0.01]
SLAVE_POS = [0.04, 0.01, -0.02]


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def test_c3_absolute_axes_independent_of_anchor_mismatch():
    """C3: with all axes absolute, slave_target must equal master_pos
    exactly, regardless of the (deliberately huge, >50mm) anchor gap."""
    target, limit_flag = compute_c3_target(
        MASTER_POS, LEFT_ANCHOR, RIGHT_ANCHOR, IDENTITY_AXIS_MAP, AXIS_SCALE,
        DEADBAND_M, MAX_OFFSET, "axis", absolute_axes=[0, 1, 2], absolute_trim=[0.0, 0.0, 0.0],
    )
    err = norm([target[i] - MASTER_POS[i] for i in range(3)])
    print(f"C3 (all absolute): target={target} master_pos={MASTER_POS} err={err*1000:.6f}mm "
          f"(anchor gap was {ANCHOR_GAP*1000:.1f}mm)")
    assert err < 1e-9, f"C3 absolute-axes target must equal master_pos exactly, got {err*1000:.4f}mm error"
    assert not limit_flag
    print("PASS\n")


def test_c2_absolute_axes_independent_of_anchor_mismatch():
    """C2: with all axes absolute, master_target must equal slave_pos
    exactly, regardless of the (deliberately huge, >50mm) anchor gap."""
    target, limit_flag = compute_c2_target(
        SLAVE_POS, RIGHT_ANCHOR, LEFT_ANCHOR, IDENTITY_AXIS_MAP, AXIS_SCALE,
        LEFT_MAX_OFFSET_M, absolute_axes=[0, 1, 2], absolute_trim=[0.0, 0.0, 0.0],
    )
    err = norm([target[i] - SLAVE_POS[i] for i in range(3)])
    print(f"C2 (all absolute): target={target} slave_pos={SLAVE_POS} err={err*1000:.6f}mm "
          f"(anchor gap was {ANCHOR_GAP*1000:.1f}mm)")
    assert err < 1e-9, f"C2 absolute-axes target must equal slave_pos exactly, got {err*1000:.4f}mm error"
    assert not limit_flag
    print("PASS\n")


def test_mixed_absolute_relative_axes():
    """absolute_axes = x,z (indices 0,2); y stays anchor-relative. Confirms
    ONLY the listed axes get the absolute override -- y must still show the
    old anchor-relative behavior on both channels."""
    absolute_axes = [0, 2]  # x, z absolute; y relative

    # --- C3 ---
    c3_target, _ = compute_c3_target(
        MASTER_POS, LEFT_ANCHOR, RIGHT_ANCHOR, IDENTITY_AXIS_MAP, AXIS_SCALE,
        DEADBAND_M, MAX_OFFSET, "axis", absolute_axes=absolute_axes, absolute_trim=[0.0, 0.0, 0.0],
    )
    # x, z: must equal master_pos exactly (absolute).
    assert abs(c3_target[0] - MASTER_POS[0]) < 1e-9, "C3 x should be absolute (==master_pos)"
    assert abs(c3_target[2] - MASTER_POS[2]) < 1e-9, "C3 z should be absolute (==master_pos)"
    # y: must be the anchor-relative formula, i.e. right_anchor[1] + (master[1]-left_anchor[1])
    # (deadband is 0.0002m, delta here is 0.02m so it's not zeroed).
    expected_y = RIGHT_ANCHOR[1] + (MASTER_POS[1] - LEFT_ANCHOR[1])
    assert abs(c3_target[1] - expected_y) < 1e-9, (
        f"C3 y should remain anchor-relative, got {c3_target[1]} expected {expected_y}")
    print(f"C3 mixed: target={c3_target} (x,z absolute -> master_pos; y relative -> anchor formula) PASS")

    # --- C2 ---
    c2_target, _ = compute_c2_target(
        SLAVE_POS, RIGHT_ANCHOR, LEFT_ANCHOR, IDENTITY_AXIS_MAP, AXIS_SCALE,
        LEFT_MAX_OFFSET_M, absolute_axes=absolute_axes, absolute_trim=[0.0, 0.0, 0.0],
    )
    assert abs(c2_target[0] - SLAVE_POS[0]) < 1e-9, "C2 x should be absolute (==slave_pos)"
    assert abs(c2_target[2] - SLAVE_POS[2]) < 1e-9, "C2 z should be absolute (==slave_pos)"
    # y stays anchor-relative -- and, importantly, STILL respects
    # left_max_offset_m (0.08m) on that relative axis, unlike the absolute
    # axes which bypass it entirely. Raw relative displacement here is
    # 0.01 - 0.10 = -0.09m, which exceeds the 0.08m limit, so it clamps.
    raw_disp_y = SLAVE_POS[1] - RIGHT_ANCHOR[1]
    clamped_disp_y = max(-LEFT_MAX_OFFSET_M, min(LEFT_MAX_OFFSET_M, raw_disp_y))
    assert abs(raw_disp_y) > LEFT_MAX_OFFSET_M, "test fixture should exercise the clamp; adjust if this no longer holds"
    expected_y_c2 = LEFT_ANCHOR[1] + clamped_disp_y
    assert abs(c2_target[1] - expected_y_c2) < 1e-9, (
        f"C2 y should remain anchor-relative AND clamped, got {c2_target[1]} expected {expected_y_c2}")
    print(f"C2 mixed: target={c2_target} (x,z absolute -> slave_pos; y relative+clamped -> anchor formula) PASS\n")


def test_symmetry_full_round_trip():
    """With all axes absolute and no trim: C3 followed by C2 (i.e. if the
    slave perfectly reached C3's target, what would C2 then command back
    for the master?) must return exactly to the master's own position --
    the round trip must be lossless for absolute axes, independent of
    anchors."""
    c3_target, _ = compute_c3_target(
        MASTER_POS, LEFT_ANCHOR, RIGHT_ANCHOR, IDENTITY_AXIS_MAP, AXIS_SCALE,
        DEADBAND_M, MAX_OFFSET, "axis", absolute_axes=[0, 1, 2], absolute_trim=[0.0, 0.0, 0.0],
    )
    # Assume slave perfectly reached c3_target (steady state).
    c2_target, _ = compute_c2_target(
        c3_target, RIGHT_ANCHOR, LEFT_ANCHOR, IDENTITY_AXIS_MAP, AXIS_SCALE,
        LEFT_MAX_OFFSET_M, absolute_axes=[0, 1, 2], absolute_trim=[0.0, 0.0, 0.0],
    )
    err = norm([c2_target[i] - MASTER_POS[i] for i in range(3)])
    print(f"round trip: master_pos={MASTER_POS} -> c3_target={c3_target} -> c2_target={c2_target}  err={err*1000:.9f}mm")
    assert err < 1e-9, f"round trip must be lossless for absolute axes, got {err*1000:.6f}mm residual"
    print("PASS\n")


if __name__ == "__main__":
    test_c3_absolute_axes_independent_of_anchor_mismatch()
    test_c2_absolute_axes_independent_of_anchor_mismatch()
    test_mixed_absolute_relative_axes()
    test_symmetry_full_round_trip()
    print("ALL TESTS PASSED")
