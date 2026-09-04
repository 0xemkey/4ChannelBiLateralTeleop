"""Proves the compile-time write gate from 0.5 BOTH ways, behaviourally.

touch_slave_sim is the only CMake target built with
TOUCH_DOB_ENABLE_TORQUE_WRITE=1 (see CMakeLists.txt, next to
add_touch_sim_node(touch_slave_sim ...)); touch_slave (hardware) never gets
that define, so DobConfig::writeAuthorized()'s only true-returning branch does
not exist in that binary. This test cannot link against touch_slave here (no
OpenHaptics hardware), so it proves the RUNTIME half of the argument instead:
with joint_space_control:=false (the default on every node, hardware or sim),
publishing a joint_target has no effect on the arm -- the Cartesian path stays
live and ignores it. With joint_space_control:=true on a
TOUCH_DOB_ENABLE_TORQUE_WRITE=1 build, the same joint_target measurably pulls
the arm toward it via the PD path. Both halves exercise the actual running
node, not a source-text grep.
"""
from __future__ import annotations

import math
import sys
import time

import pytest
from geometry_msgs.msg import Vector3Stamped

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from sim_common import Collector, SimNode, unique_ns  # noqa: E402


# rad, added to whatever q starts at. Deliberately SMALL: max_torque_nm
# ({0.162,0.081,0.081} N*m, sized for a 0.6 N contact-force envelope, not for
# repositioning) clamps prepareJointTorqueCommand()'s output, and on this
# arm's light inertia a large step saturates that clamp and rings rather than
# settling within a few seconds -- a real property of the shipped safety
# clamp, not a bug in the gate this test exists to check. A small step stays
# inside the clamp's near-linear range, matching how joint_target actually
# moves in real operation (continuous small increments from the peer arm's
# tracked position, never a multi-degree jump).
JOINT_TARGET_OFFSET = (0.05, -0.04, 0.03)
DRIVE_S = 2.5
HOLD_S = 1.5  # settle at a PD-held pose before stepping, see note below


def _drive_and_measure(joint_space_control: bool) -> tuple:
    """Returns (q_initial, q_final, q_target).

    q_initial is taken AFTER a hold phase (joint_target = wherever the arm
    happened to be), not at raw startup. Nothing controls the arm before
    joint_space_control_ arms, so its as-launched pose is wherever gravity and
    friction left it after an uncontrolled settle -- run to run that can land
    close to a joint limit, and a fixed offset from an unpredictable pose is
    an unreliable base for a convergence measurement. Holding first (a no-op
    when joint_space_control is false: the Cartesian path ignores the topic)
    gives both branches the same kind of starting point.
    """
    ns = unique_ns("gate")
    with SimNode(
        ns, device="Left",
        params={
            "joint_space_control": "true" if joint_space_control else "false",
            # 1.5 N (up from the 0.6 N default), matching the value already
            # used in this project's DOB tuning analysis. At 0.6 N, gravity
            # feedforward alone consumes 81-95% of shoulder/elbow's torque
            # budget (0.081 N*m each) at this test's starting pose, leaving
            # almost no headroom for position correction and saturating
            # immediately on any step -- confirmed by direct tau_cmd logging
            # (raw vs clamped), not assumed. That is a torque-budget
            # limitation, not a control-law bug; Kp/Kd tuning cannot fix it.
            # Test-only: dob_torque_f_max_n defaults to 0.6N (unchanged real
            # hardware behavior) everywhere this isn't explicitly overridden.
            "dob_torque_f_max_n": "1.5",
        },
    ) as sim:
        import rclpy
        from rclpy.node import Node as RclpyNode

        if not rclpy.ok():
            rclpy.init()
        driver = RclpyNode(f"driver_{ns}")
        target_pub = driver.create_publisher(Vector3Stamped, sim.topic("joint_target"), 10)

        q_c = Collector(sim.topic("joint_angles"), Vector3Stamped)

        def _publish_and_settle(target_xyz: tuple, duration_s: float) -> None:
            """Keep publishing target_xyz on target_pub for duration_s while
            also spinning q_c, instead of going silent on joint_target while
            waiting for q_c's buffer to catch up.

            Root-caused 2026-09-03 (C4 diagnostic investigation): the
            original code called q_c.spin_for(...) alone here, which spins
            ONLY q_c's node -- target_pub.publish() never fires during that
            window. Combined with the control law's stale_timeout_s_=0.05s
            default, that silence is long enough for joint_target_fresh to
            go false and the PD correction term to drop out entirely, with
            gravity feedforward alone (which does not track a moving target,
            only approximately cancels gravity at the CURRENT pose) left to
            govern the arm -- confirmed directly via touch_slave.cpp's
            joint_pd_error_term_nm reading exactly 0.0 for ~70-190
            consecutive servo ticks at exactly this call site, in every
            failing run inspected. The post-drive call (duration_s=0.2) is
            the worse of the two: it sits immediately before q_final is
            read, so a real chunk of "failure" was measuring uncorrected
            gravity-drift during this test's own bookkeeping pause, not the
            control law's actual tracking. bilateral_4ch.py (the real
            production path) has no equivalent gap -- it publishes
            joint_target unconditionally every tick -- so this was a test
            methodology bug, not a C4 control-law bug.
            """
            deadline = time.time() + duration_s
            while time.time() < deadline:
                msg = Vector3Stamped()
                msg.vector.x, msg.vector.y, msg.vector.z = target_xyz
                target_pub.publish(msg)
                rclpy.spin_once(driver, timeout_sec=0.01)
                rclpy.spin_once(q_c._node, timeout_sec=0.01)

        try:
            q_c.wait_for(5, timeout=5.0)
            q_guess = q_c.messages[-1]
            hold = (q_guess.vector.x, q_guess.vector.y, q_guess.vector.z)

            hold_deadline = time.time() + HOLD_S
            while time.time() < hold_deadline:
                msg = Vector3Stamped()
                msg.vector.x, msg.vector.y, msg.vector.z = hold
                target_pub.publish(msg)
                rclpy.spin_once(driver, timeout_sec=0.02)

            _publish_and_settle(hold, 0.1)
            q0 = q_c.messages[-1]
            q_initial = (q0.vector.x, q0.vector.y, q0.vector.z)
            q_target = tuple(q_initial[i] + JOINT_TARGET_OFFSET[i] for i in range(3))

            deadline = time.time() + DRIVE_S
            while time.time() < deadline:
                msg = Vector3Stamped()
                msg.vector.x, msg.vector.y, msg.vector.z = q_target
                target_pub.publish(msg)
                rclpy.spin_once(driver, timeout_sec=0.02)

            _publish_and_settle(q_target, 0.2)
            qf = q_c.messages[-1]
            q_final = (qf.vector.x, qf.vector.y, qf.vector.z)
        finally:
            q_c.close()
            driver.destroy_node()

    return q_initial, q_final, q_target


def _closed_fraction(q_initial, q_final, q_target) -> float:
    gap0 = math.sqrt(sum((q_target[i] - q_initial[i]) ** 2 for i in range(3)))
    gapf = math.sqrt(sum((q_target[i] - q_final[i]) ** 2 for i in range(3)))
    if gap0 < 1e-6:
        return 0.0
    return 1.0 - gapf / gap0


def test_write_gated_off_by_default_ignores_joint_target():
    q0, qf, qt = _drive_and_measure(joint_space_control=False)
    closed = _closed_fraction(q0, qf, qt)
    assert closed < 0.25, (
        f"joint_space_control=false but the arm closed {closed:.0%} of the gap "
        f"to joint_target -- the Cartesian path is no longer the live default. "
        f"q0={q0} qf={qf} qt={qt}"
    )


def test_write_armed_tracks_joint_target():
    q0, qf, qt = _drive_and_measure(joint_space_control=True)
    closed = _closed_fraction(q0, qf, qt)
    assert closed > 0.6, (
        f"joint_space_control=true but the arm only closed {closed:.0%} of the "
        f"gap to joint_target in {DRIVE_S}s -- either the PD path did not arm "
        f"(check configureBilateral()/writeAuthorized()) or gains are too weak. "
        f"q0={q0} qf={qf} qt={qt}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
