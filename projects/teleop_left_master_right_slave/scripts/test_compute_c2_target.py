#!/usr/bin/env python3
"""Regression test for the C2/C3 frame-mixing bug found 2026-09-09 during
real-hardware root-cause investigation of a READY->ACTIVE target
discontinuity (session 20260909T100330Z_dob_observe, trials
h2abs_gravcomp_B_070 and h2abs_gravcomp_B2_070).

Pure Python, no rclpy/ROS/hardware dependency -- matches the
"framework-free, unit-testable without rclpy" convention already used for
TeleopStartupAlignment (see startup_alignment.py's own docstring). Run
directly:

    python3 test_compute_c2_target.py

BUG: _update_c2() (C2, slave position -> master target) had NO
absolute_axes handling -- it always used the anchor-relative formula
`left_anchor + (slave_pos - right_anchor)`, while _tick()'s C3 path
(master position -> slave target) DID override to the absolute formula
`target = master_pos` for axes listed in absolute_axes. When
`left_anchor != right_anchor` (captured at bridge startup, from whatever
pose the two devices happened to be resting at -- unrelated to the
operator's reference-pose match at arm time), this mismatch became an
UNCANCELLED constant offset injected into C2's target.

MEASURED ON REAL HARDWARE (trial h2abs_gravcomp_B2_070):
    left_anchor  = [ 0.03659441350635732, -0.0290291903897336, -0.02598308753967285]
    right_anchor = [-0.0015765916532591738, -0.11049050431502506, -0.04055201796481485]
    -> alignment_error_norm = 0.09113 (91.1mm), dominated by Y (81.5mm)
    -> master position_error_clamped = TRUE for 100% of a 19-second trial
       (master's own C2 spring pinned against its 0.03m clamp the entire
       time), even though the operator held the master within ~1mm of the
       intended reference pose throughout.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from bridge_master_slave import apply_axis_map_inverse, compute_c2_target  # noqa: E402

# Real values from the B2 trial's bridge log ("anchors captured: ...").
LEFT_ANCHOR = [0.03659441350635732, -0.0290291903897336, -0.02598308753967285]
RIGHT_ANCHOR = [-0.0015765916532591738, -0.11049050431502506, -0.04055201796481485]
IDENTITY_AXIS_MAP = [(0, 1.0), (1, 1.0), (2, 1.0)]  # axis_map=x,y,z
AXIS_SCALE = [1.0, 1.0, 1.0]
LEFT_MAX_OFFSET_M = 0.08
ABSOLUTE_AXES = [0, 1, 2]  # --absolute-axes x,y,z, as used in trials A/B/B2
ABSOLUTE_TRIM = [0.0, 0.0, 0.0]

# The operator's actual master reference pose at arm time (from the B2
# pre-arm check this session: ||delta_p|| ~= 1.01mm from the fixed
# reference -- i.e. this IS approximately where the master was held).
MASTER_REFERENCE_POSE = [0.008123172760009767, 0.001142270565032959, 0.007907872676849365]


def old_buggy_c2_target(slave_pos, right_anchor, left_anchor, axis_map, axis_scale,
                         left_max_offset_m):
    """Exact copy of _update_c2()'s ORIGINAL logic (pre-fix), for the
    regression test's "before" case. No absolute_axes awareness at all."""
    slave_disp = [slave_pos[i] - right_anchor[i] for i in range(3)]
    unscaled = [
        slave_disp[i] / axis_scale[i] if abs(axis_scale[i]) > 1e-9 else 0.0
        for i in range(3)
    ]
    master_disp = apply_axis_map_inverse(axis_map, unscaled)
    limit = float(left_max_offset_m)
    if limit > 0.0:
        for i in range(3):
            if master_disp[i] > limit:
                master_disp[i] = limit
            elif master_disp[i] < -limit:
                master_disp[i] = -limit
    return [left_anchor[i] + master_disp[i] for i in range(3)]


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def test_bug_reproduction_old_formula_produces_large_discontinuity():
    """Prove the OLD formula, evaluated once C3 has converged (slave_pos ~=
    master_pos, the absolute-mode steady state), produces a target far from
    the master's actual position -- reproducing the measured 91mm-scale
    discontinuity from real hardware."""
    # Once C3 (absolute mode) has converged, slave_pos ~= master's actual
    # reference pose (that's the whole point of C3 = target_slave = p_master).
    slave_pos_converged = list(MASTER_REFERENCE_POSE)
    old_target = old_buggy_c2_target(
        slave_pos_converged, RIGHT_ANCHOR, LEFT_ANCHOR, IDENTITY_AXIS_MAP, AXIS_SCALE,
        LEFT_MAX_OFFSET_M,
    )
    error = [old_target[i] - MASTER_REFERENCE_POSE[i] for i in range(3)]
    error_norm_mm = norm(error) * 1000
    print(f"[OLD/buggy] C2 target={old_target}  master_ref={MASTER_REFERENCE_POSE}")
    print(f"[OLD/buggy] target - master_ref = {error}, ||.||={error_norm_mm:.2f}mm")
    # Reproduces the measured ~91mm-scale discontinuity (bounded above by
    # left_max_offset_m=0.08m=80mm on any single axis after clamping, but
    # still large -- Y alone clamps to exactly 80mm here).
    assert error_norm_mm > 30.0, (
        f"expected the OLD formula to reproduce a large (>30mm) discontinuity, "
        f"got {error_norm_mm:.2f}mm -- bug reproduction failed"
    )
    print("PASS: old formula reproduces a large discontinuity, as measured on hardware.\n")


def test_fixed_c2_target_matches_master_reference_in_absolute_mode():
    """The FIX: with absolute_axes=[0,1,2], C2's target for those axes must
    equal the slave's raw position (mirroring C3 exactly) regardless of
    left_anchor/right_anchor -- eliminating the anchor-mismatch-dependent
    discontinuity entirely."""
    slave_pos_converged = list(MASTER_REFERENCE_POSE)
    new_target, travel_limit_flag = compute_c2_target(
        slave_pos_converged, RIGHT_ANCHOR, LEFT_ANCHOR, IDENTITY_AXIS_MAP, AXIS_SCALE,
        LEFT_MAX_OFFSET_M, absolute_axes=ABSOLUTE_AXES, absolute_trim=ABSOLUTE_TRIM,
    )
    error = [new_target[i] - MASTER_REFERENCE_POSE[i] for i in range(3)]
    error_norm_mm = norm(error) * 1000
    print(f"[FIXED] C2 target={new_target}  master_ref={MASTER_REFERENCE_POSE}")
    print(f"[FIXED] target - master_ref = {error}, ||.||={error_norm_mm:.6f}mm")
    assert error_norm_mm < 0.001, (
        f"expected the fixed formula to reproduce master's own position exactly "
        f"(it should be slave_pos*scale+trim = master_pos*1+0), got {error_norm_mm:.4f}mm residual"
    )
    assert not travel_limit_flag, "absolute axes must not be travel-limited by left_max_offset_m"
    print("PASS: fixed formula eliminates the discontinuity; C2 target == master's own "
          "position for absolute axes, exactly as C3 already guaranteed the reverse.\n")


def test_fixed_formula_unchanged_for_non_absolute_axes_no_regression():
    """With absolute_axes=[] (the pre-existing, non-absolute C2 behavior),
    the fixed function must produce EXACTLY the same result as the old
    formula -- confirms this fix does not change behavior for any
    configuration that was not using --absolute-axes."""
    slave_pos = [0.02, -0.15, -0.05]  # arbitrary displaced slave position
    old_target = old_buggy_c2_target(
        slave_pos, RIGHT_ANCHOR, LEFT_ANCHOR, IDENTITY_AXIS_MAP, AXIS_SCALE, LEFT_MAX_OFFSET_M,
    )
    new_target, _ = compute_c2_target(
        slave_pos, RIGHT_ANCHOR, LEFT_ANCHOR, IDENTITY_AXIS_MAP, AXIS_SCALE, LEFT_MAX_OFFSET_M,
        absolute_axes=[], absolute_trim=[0.0, 0.0, 0.0],
    )
    diff = norm([old_target[i] - new_target[i] for i in range(3)])
    print(f"[no-absolute-axes] old={old_target} new={new_target} diff={diff:.9f}")
    assert diff < 1e-12, "fix must be a no-op when absolute_axes is empty (no regression)"
    print("PASS: no behavior change for non-absolute-axes configurations.\n")


if __name__ == "__main__":
    test_bug_reproduction_old_formula_produces_large_discontinuity()
    test_fixed_c2_target_matches_master_reference_in_absolute_mode()
    test_fixed_formula_unchanged_for_non_absolute_axes_no_regression()
    print("ALL TESTS PASSED")
