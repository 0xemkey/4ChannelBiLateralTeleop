"""Regression test for the 2026-09-04 applied_force telemetry bug.

BUG (now fixed, touch_slave.cpp ~line 1301): `state_.applied_force_n` used
to be assigned from `force_ros`, captured BEFORE the C1/C4-onto-Cartesian
relay block runs -- so the `applied_force` ROS topic (documented
project-wide as "the REAL rendered Cartesian force... the haptic force the
operator actually feels", e.g. log_phase2_scenario.py's own docstring)
silently omitted the relay's contribution whenever it was active, even
though the actual `HD_CURRENT_FORCE` device write (and the separate
`last_force` topic, which reads the write back) was correct.

THIS TEST WOULD FAIL AGAINST THE PRE-FIX CODE: with nothing else driving
the device (no target_pose, no force_cmd -- force_ros stays exactly
(0,0,0) throughout), a relayed joint torque comfortably above
kMinRelayTorqueNormNm should still produce a nonzero applied_force. Before
the fix, applied_force stayed at (0,0,0) regardless of the relay,
identically to the "relay inactive" case below -- i.e. the two cases were
indistinguishable from applied_force alone, which is exactly the defect.

Deliberately does NOT use bilateral_4ch.py or a wall model -- publishes
directly to joint_torque_ext_cmd, the same minimal-harness style
test_c1_cartesian_relay_deadband.py already uses, so this test isolates
the telemetry fix from the relay math (already covered by that other
test) and from bilateral_4ch.py's own weighting/deadband (covered
elsewhere).
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import pytest
from geometry_msgs.msg import WrenchStamped

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, SimNode, unique_ns  # noqa: E402

from touch_haptics.msg import ForceEstimate  # noqa: E402

BELOW_FLOOR_NM = 0.008  # kMinRelayTorqueNormNm is 0.02
ABOVE_FLOOR_NM = 0.12


def _publish_ext_torque(pub, tau) -> None:
    msg = ForceEstimate()
    msg.tau_ext_deadbanded_nm = list(tau)
    msg.valid = True
    pub.publish(msg)


def _drive_and_read(tau_axis0: float) -> tuple:
    """Returns (applied_force_norm, last_force_norm) after driving a
    constant relayed torque for long enough to settle past last_force's
    one-frame lag."""
    ns = unique_ns("appliedfix")
    with SimNode(
        ns, device="Left",
        params={"require_deadman": "false", "enabled_on_start": "true",
                "publish_telemetry": "true"},
    ) as sim:
        import rclpy
        if not rclpy.ok():
            rclpy.init()

        with LiveNode("applied_fix_check") as live:
            live.watch("applied_force", sim.topic("applied_force"), WrenchStamped,
                      extract=lambda m: (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z))
            live.watch("last_force", sim.topic("last_force"), WrenchStamped,
                      extract=lambda m: (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z))

            ext_pub = live._node.create_publisher(
                ForceEstimate, sim.topic("joint_torque_ext_cmd"), 10)

            tau = (tau_axis0, 0.0, 0.0)
            deadline = time.time() + 1.5
            while time.time() < deadline:
                _publish_ext_torque(ext_pub, tau)
                live.spin_once(timeout_sec=0.01)

            af = live.latest.get("applied_force")
            lf = live.latest.get("last_force")
            assert af is not None, "applied_force never arrived -- publish_telemetry not on?"
            assert lf is not None, "last_force never arrived -- publish_telemetry not on?"

            def _norm(v):
                return (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5

            return _norm(af), _norm(lf)


def test_applied_force_reflects_relay_when_active():
    """Relay ACTIVE (tau above the floor): applied_force must move. This is
    the case that silently read exactly 0.0 before the 2026-09-04 fix,
    identical to the relay-inactive case below -- the two being
    indistinguishable from applied_force alone was the whole bug."""
    af_norm, lf_norm = _drive_and_read(ABOVE_FLOOR_NM)
    assert af_norm > 0.02, (
        f"relayed tau={ABOVE_FLOOR_NM} N*m (above the relay floor) produced "
        f"applied_force norm={af_norm:.5f} N -- expected a real nonzero "
        f"reading; this is exactly the pre-fix bug (applied_force captured "
        f"before the relay's Cartesian addition) if it reads ~0.0"
    )
    # Cross-consistency: applied_force (same-frame) and last_force
    # (one-frame-lagged device readback) should now agree once settled,
    # since both derive from the same underlying force_device write --
    # confirms the fix didn't just make applied_force move, but made it
    # move to the CORRECT (matching) value.
    assert abs(af_norm - lf_norm) < 0.05 * max(af_norm, lf_norm, 1e-6), (
        f"applied_force ({af_norm:.5f} N) and last_force ({lf_norm:.5f} N) "
        f"disagree by more than 5% at steady state -- they should converge "
        f"to the same value once settled, since both reflect the same "
        f"force_device write"
    )


def test_applied_force_stays_quiet_when_relay_inactive():
    """Relay INACTIVE (tau below the floor): applied_force must stay near
    zero, matching pre-fix behavior -- proves the fix did not regress the
    already-correct no-relay case (test_c1_cartesian_relay_deadband.py's
    own floor, re-checked here from the applied_force side specifically)."""
    af_norm, lf_norm = _drive_and_read(BELOW_FLOOR_NM)
    assert af_norm < 0.01, (
        f"relayed tau={BELOW_FLOOR_NM} N*m (below the relay floor) still "
        f"produced applied_force norm={af_norm:.5f} N -- deadband leaking "
        f"into telemetry, or the fix broke the inactive case"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
