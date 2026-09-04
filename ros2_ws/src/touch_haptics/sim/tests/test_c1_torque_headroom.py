"""C1 torque-budget headroom regression, sim-only.

Operationalizes a real-hardware finding from 2026-09-03's first live C1 test:
gravity feedforward alone can consume most or all of a joint's derived torque
ceiling (derivePerJointTorqueLimits(dob_torque_f_max_n)) even before any C1
signal is added, silently swallowing the relay. Shoulder was found pinned
exactly at its clamp ceiling at rest, live -- but shoulder was already
excluded from C1's relay (bilateral_4ch.py's --c1-disable-joints default) for
an unrelated reason (unreliable identification data), so nobody had checked
whether the two joints C1 actually relays (waist, elbow) have the same
problem.

This test drives touch_slave_sim through the 10 poses in
~/dob_logs/results/gravity_left_minimal.json's own per_pose_table -- the
exact poses the deployed gravity model was fit against, the most defensible
"representative sweep" available -- at the sim's real default
dob_torque_f_max_n (0.9N as of 2026-09-03, not the elevated 1.5N
test_joint_space_write_gated.py uses for an unrelated reason), reads back
joint_pd_gravity_ff_nm from DobDiagnostics (added 2026-09-03 for the C4
investigation, reused here), and asserts a minimum headroom fraction for
waist and elbow (shoulder is out of scope -- already excluded from the
relay).

Threshold (headroom_frac >= 0.30, i.e. gravity FF alone must stay under 70%
of the clamp) is an explicit judgment call, not derived from first
principles -- the real-hardware finding was binary (shoulder at ~100%), not
a calibrated margin. Loose enough not to be a hair-trigger, tight enough
that it originally caught what this test's own math predicted before it was
even run: at the OLD 0.6N default, elbow sat at 75-94% of ceiling (5.6-24.8%
headroom) at 8 of the 10 calibrated poses, verified directly against the
calibration file and then confirmed by actually running this test.
dob_torque_f_max_n was raised 0.6 -> 0.9N in response (touch_slave.cpp,
same date) -- at 0.9N elbow's worst-case headroom is 37.1%, clearing this
test's bar with real margin. See FOUR_CHANNEL_TODO.md's 2026-09-03 section
for the full decision record (raising the shared default was chosen over a
per-joint ceiling or excluding elbow from the relay too).
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from sim_common import SimNode, unique_ns  # noqa: E402

from touch_haptics.msg import DobDiagnostics  # noqa: E402
from geometry_msgs.msg import Vector3Stamped  # noqa: E402

import pytest  # noqa: E402

GRAVITY_JSON = os.path.expanduser("~/dob_logs/results/gravity_left_minimal.json")
# Sim's real default, touch_slave.cpp's dob_torque_f_max_n declare_parameter
# call. Raised 0.6 -> 0.9 N on 2026-09-03 specifically because this test
# (first run at 0.6N) found elbow's headroom as low as 5.6% -- see that
# file's comment at the same spot for the full computation. Kept as an
# explicit constant here, not inferred from the C++ default, since the
# whole point of this test is verifying the ACTUALLY DEPLOYED default has
# adequate headroom -- if the C++ default ever changes again without this
# constant being updated to match, this test should be reviewed, not
# silently pass against a stale assumption.
DOB_TORQUE_F_MAX_N = 0.9
L1_M = L2_M = 0.135  # touch_dob.hpp kL1Meters/kL2Meters
CLAMP_NM = {
    0: DOB_TORQUE_F_MAX_N * (L1_M + L2_M),  # waist
    1: DOB_TORQUE_F_MAX_N * L1_M,           # shoulder (out of scope here)
    2: DOB_TORQUE_F_MAX_N * L2_M,           # elbow
}
MIN_HEADROOM_FRAC = 0.30
SETTLE_S = 1.0

JOINT_NAMES = {0: "waist", 1: "shoulder", 2: "elbow"}


def _load_poses() -> list:
    if not os.path.exists(GRAVITY_JSON):
        pytest.skip(f"calibration file not found: {GRAVITY_JSON}")
    with open(GRAVITY_JSON) as f:
        data = json.load(f)
    return [tuple(p["q"]) for p in data["per_pose_table"]]


def test_c1_torque_headroom_waist_and_elbow():
    import rclpy
    from rclpy.node import Node as RclpyNode
    from rclpy.executors import SingleThreadedExecutor

    poses = _load_poses()

    ns = unique_ns("headroom")
    with SimNode(
        ns, device="Left",
        params={
            "joint_space_control": "true",
            "dob_torque_f_max_n": str(DOB_TORQUE_F_MAX_N),
        },
    ) as sim:
        if not rclpy.ok():
            rclpy.init()

        driver = RclpyNode(f"driver_{ns}")
        target_pub = driver.create_publisher(Vector3Stamped, sim.topic("joint_target"), 10)

        # Diagnostics on a separate node/executor, spun on a background
        # thread -- learned 2026-09-03 while diagnosing C4 that subscribing
        # dob_diagnostics (published near the full servo rate) on the same
        # actively-polled node as the publish loop measurably perturbs the
        # sim's own closed-loop timing.
        diag_node = RclpyNode(f"diag_{ns}")
        diag_log: list = []
        diag_node.create_subscription(
            DobDiagnostics, sim.topic("dob_diagnostics"), diag_log.append, 50)
        diag_executor = SingleThreadedExecutor()
        diag_executor.add_node(diag_node)
        diag_thread = threading.Thread(target=diag_executor.spin, daemon=True)
        diag_thread.start()

        results = {0: [], 2: []}  # joint index -> list of headroom_frac per pose
        try:
            for q in poses:
                diag_log.clear()
                deadline = time.time() + SETTLE_S
                while time.time() < deadline:
                    msg = Vector3Stamped()
                    msg.vector.x, msg.vector.y, msg.vector.z = q
                    target_pub.publish(msg)
                    rclpy.spin_once(driver, timeout_sec=0.02)

                active = [d for d in diag_log if d.joint_pd_active]
                assert active, (
                    f"no active joint_pd samples for pose q={q} -- "
                    f"joint_space_control branch never ran"
                )
                latest = active[-1]
                for i in (0, 2):
                    grav_ff = latest.joint_pd_gravity_ff_nm[i]
                    headroom = 1.0 - abs(grav_ff) / CLAMP_NM[i]
                    results[i].append((q, grav_ff, headroom))
        finally:
            diag_executor.shutdown()
            diag_thread.join(timeout=2.0)
            driver.destroy_node()
            diag_node.destroy_node()

    failures = []
    for i in (0, 2):
        for q, grav_ff, headroom in results[i]:
            if headroom < MIN_HEADROOM_FRAC:
                failures.append(
                    f"{JOINT_NAMES[i]}: pose q={tuple(round(x, 4) for x in q)} "
                    f"gravity_ff={grav_ff:.4f} N*m vs ceiling {CLAMP_NM[i]:.4f} "
                    f"N*m -> headroom={headroom:.1%} (need >= {MIN_HEADROOM_FRAC:.0%})"
                )

    assert not failures, (
        "C1-relayed joint(s) have insufficient torque headroom at "
        f"dob_torque_f_max_n={DOB_TORQUE_F_MAX_N}N -- gravity feedforward "
        "alone consumes too much of the clamp ceiling, silently swallowing "
        "any C1 signal added on top (the same failure mode found live on "
        "real hardware for shoulder, 2026-09-03). See this file's module "
        "docstring and FOUR_CHANNEL_TODO.md's 2026-09-03 section for the "
        "decision this surfaces:\n  " + "\n  ".join(failures)
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
