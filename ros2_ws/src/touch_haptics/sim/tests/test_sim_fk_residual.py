"""Regression pin: HD_CURRENT_TRANSFORM in hd_sim must equal
fkFromJacobianConvention(HD_CURRENT_JOINT_ANGLES) to machine precision.

This is the same invariant tools/hdapi_fk_residual.cpp checked when it was run
against hd_sim earlier in this project (RMS 0.000000 m over a 115 mm span).
Both quantities read from d->arm.qReported() on the SAME frame in
hd_sim_api.cpp (see arm_model.hpp:tipPositionM(), which builds the pose from
qReported(), and hdGetDoublev's HD_CURRENT_JOINT_ANGLES case, which reads the
same qReported()) -- so this is a floating-point-exact identity, not a
statistical check. A nonzero residual means someone changed one side of that
pairing (a different q, or a different FK formula) without the other, and
this test exists to catch exactly that regression cheaply, before it ever
reaches a hardware residual check that costs real Touch time.
"""
from __future__ import annotations

import math
import sys

import pytest
from geometry_msgs.msg import PoseStamped, Vector3Stamped

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from sim_common import Collector, SimNode, unique_ns  # noqa: E402

L1 = 0.135
L2 = 0.135


def fk(q):
    r = L1 * math.cos(q[1]) + L2 * math.sin(q[2])
    return (
        -math.sin(q[0]) * r,
        L1 * math.sin(q[1]) - L2 * math.cos(q[2]),
        math.cos(q[0]) * r,
    )


def test_fk_residual_near_zero():
    ns = unique_ns("fkres")
    with SimNode(ns, device="Left") as sim:
        pose_c = Collector(sim.topic("pose"), PoseStamped)
        q_c = Collector(sim.topic("joint_angles"), Vector3Stamped)
        try:
            pose_c.wait_for(20, timeout=10.0)
            q_c.wait_for(20, timeout=10.0)

            # Pair by header.stamp, not list index: /pose and /joint_angles are
            # two independently-queued subscriptions, so their arrival order is
            # not guaranteed to line up index-for-index even though both were
            # written from the SAME publishTick() snapshot and therefore share
            # the identical stamp (pose.header.stamp, reused verbatim for every
            # other publish_telemetry_ topic in that tick). The arm is free to
            # move under gravity/friction between ticks (nothing commands it in
            # this test), so an index-based off-by-one silently compares two
            # different poses and manufactures a residual that has nothing to
            # do with the FK identity this test exists to check.
            q_by_stamp = {
                (m.header.stamp.sec, m.header.stamp.nanosec): m for m in q_c.messages
            }
            residuals = []
            for pm in pose_c.messages:
                key = (pm.header.stamp.sec, pm.header.stamp.nanosec)
                qm = q_by_stamp.get(key)
                if qm is None:
                    continue
                q = (qm.vector.x, qm.vector.y, qm.vector.z)
                p = pm.pose.position
                fx, fy, fz = fk(q)
                residuals.append(math.sqrt(
                    (fx - p.x) ** 2 + (fy - p.y) ** 2 + (fz - p.z) ** 2))
            n = len(residuals)
            assert n > 0, "no paired pose/joint_angles samples (no matching stamps)"

            rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))
            assert rms < 1e-9, (
                f"FK residual RMS={rms:.3e} m over {n} samples -- hd_sim's "
                f"HD_CURRENT_TRANSFORM and HD_CURRENT_JOINT_ANGLES have drifted "
                f"apart. See {sim.log_text()[-500:]}"
            )
        finally:
            pose_c.close()
            q_c.close()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
