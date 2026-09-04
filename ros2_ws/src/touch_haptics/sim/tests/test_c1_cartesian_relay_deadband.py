"""C1/C4-onto-Cartesian relay deadband, sim-only.

Regression for the real-hardware finding from 2026-09-03: the first version
of this relay (touch_slave.cpp, the block right before the final
HD_CURRENT_FORCE write) applied bilateral_4ch.py's relayed joint torque to
the Cartesian force output UNCONDITIONALLY whenever fresh -- including
resting-state residual (never exactly zero, ~0.0003-0.05 N*m even with
nothing touching the peer arm). On real hardware this continuously nudged
the receiving node's true Cartesian position, which bridge_master_slave.py
(reading that live position to compute the peer's C2/C3 target) mistook for
deliberate motion, slowly dragging the peer arm in an unintended axis.
Fixed with kMinRelayTorqueNormNm (0.02 N*m): the raw joint torque must clear
a real, deliberate-contact-sized floor BEFORE any Cartesian conversion
happens, not just be clamped after.

Verification method: checks the actual C++ logic directly via the
TOUCH_DIAG_CARTESIAN_RELAY=1 stderr trace (force_device's value right at the
HD_CURRENT_FORCE write site), not the `last_force` ROS telemetry topic --
that topic was found, while building this test, to not reliably reflect a
Cartesian-only write in this exact scenario (a separate, pre-existing sim
readback quirk, not a defect in the relay logic itself, which the direct
force_device trace independently confirms is correct).
"""
from __future__ import annotations

import re
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import SimNode, unique_ns  # noqa: E402

from touch_haptics.msg import ForceEstimate  # noqa: E402

BELOW_FLOOR_NM = 0.008  # kMinRelayTorqueNormNm is 0.02; this norm is ~0.008
ABOVE_FLOOR_NM = 0.12   # comfortably above the floor


def _publish_ext_torque(pub, tau) -> None:
    msg = ForceEstimate()
    msg.tau_ext_deadbanded_nm = list(tau)
    msg.valid = True
    pub.publish(msg)


def _last_force_device_norm(log_text: str) -> float:
    lines = [l for l in log_text.splitlines() if "[diag_relay]" in l]
    assert lines, "no [diag_relay] trace lines found -- TOUCH_DIAG_CARTESIAN_RELAY not active?"
    m = re.search(
        r"force_device=\(([-\d.]+),([-\d.]+),([-\d.]+)\)", lines[-1])
    assert m, f"could not parse force_device from: {lines[-1]!r}"
    x, y, z = (float(g) for g in m.groups())
    return (x * x + y * y + z * z) ** 0.5


@pytest.mark.parametrize(
    "tau_axis0,expect_force", [
        (BELOW_FLOOR_NM, False),
        (ABOVE_FLOOR_NM, True),
    ],
)
def test_c1_cartesian_relay_respects_deadband(tau_axis0: float, expect_force: bool):
    import rclpy
    from rclpy.node import Node as RclpyNode

    ns = unique_ns("reldb")
    with SimNode(
        ns, device="Left",
        params={"require_deadman": "false", "enabled_on_start": "true"},
        sim_env={"TOUCH_DIAG_CARTESIAN_RELAY": "1"},
    ) as sim:
        if not rclpy.ok():
            rclpy.init()
        driver = RclpyNode(f"driver_{ns}")
        ext_pub = driver.create_publisher(
            ForceEstimate, sim.topic("joint_torque_ext_cmd"), 10)
        try:
            # tau on the waist axis only, matching C1's actual relay shape
            # (shoulder excluded upstream, but this node doesn't know or
            # care -- it just consumes whatever arrives on this topic).
            tau = (tau_axis0, 0.0, 0.0)
            deadline = time.time() + 1.0
            while time.time() < deadline:
                _publish_ext_torque(ext_pub, tau)
                rclpy.spin_once(driver, timeout_sec=0.02)
        finally:
            driver.destroy_node()

        norm = _last_force_device_norm(sim.log_text())

        if expect_force:
            assert norm > 0.02, (
                f"tau={tau} (above the 0.02 N*m floor) produced no "
                f"meaningful Cartesian force -- force_device norm={norm:.5f} N, "
                f"the relay may be over-suppressing"
            )
        else:
            assert norm < 0.01, (
                f"tau={tau} (below the 0.02 N*m floor) still produced a "
                f"Cartesian force -- force_device norm={norm:.5f} N, "
                f"the deadband regressed and resting noise is leaking "
                f"into the Cartesian output again"
            )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
