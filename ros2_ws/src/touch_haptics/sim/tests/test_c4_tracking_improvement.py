"""C4's specific claim (plan Sec 3.5): feed-forward of the master's estimated
hand torque should measurably tighten the slave's tracking of a MOVING master
target, compared to position coupling (C3) alone.

hd_sim's "hand" model applies a genuine impedance force to the LEFT device's
own physics (see environment.hpp) -- independent of anything this test
publishes -- so the master here is being driven by a real, if simulated,
disturbance, exactly the situation C4 exists for. Runs two full
bilateral_4ch.py sessions back to back (w4=0, then w4=0.7) against a fresh
pair of sim nodes each time, and compares RMS tip-position tracking error.

This is the most physics-dependent test in the suite -- allow a wide margin
(w4>0 need only be NOT WORSE within 10%, not dramatically better) to keep it
a useful regression check rather than a source of flaky CI failures.
"""
from __future__ import annotations

import math
import subprocess
import sys
import time
from pathlib import Path
from typing import List

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import Collector, SimNode, unique_ns  # noqa: E402

SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bilateral_4ch.py"
)

HAND_ENV = {
    "TOUCH_SIM_HAND": "stiff",
    "TOUCH_SIM_HAND_MOTION": "sine",
    "TOUCH_SIM_HAND_F0": "0.8",
    "TOUCH_SIM_HAND_AMP": "0.02",
}
RUN_S = 6.0
SETTLE_S = 1.5


def _run_session(w4: float) -> float:
    """Returns RMS |pose_left - pose_right| over the measurement window."""
    left_ns = unique_ns("c4l")
    right_ns = unique_ns("c4r")

    left = SimNode(left_ns, device="Left",
                   params={"joint_space_control": "true"}, sim_env=HAND_ENV)
    right = SimNode(right_ns, device="Default Device",
                    params={"joint_space_control": "true"})
    bridge_env = dict(**__import__("os").environ)
    bridge = subprocess.Popen(
        [sys.executable, str(SCRIPT),
         "--architecture", "custom", "--w1", "0", "--w2", "0", "--w3", "1",
         "--w4", str(w4),
         "--left-force-topic", f"/{left_ns}/force_ext_hat",
         "--right-force-topic", f"/{right_ns}/force_ext_hat",
         "--left-joint-angles-topic", f"/{left_ns}/joint_angles",
         "--right-joint-angles-topic", f"/{right_ns}/joint_angles",
         "--left-joint-target-topic", f"/{left_ns}/joint_target",
         "--right-joint-target-topic", f"/{right_ns}/joint_target",
         "--left-joint-torque-ext-topic", f"/{left_ns}/joint_torque_ext_cmd",
         "--right-joint-torque-ext-topic", f"/{right_ns}/joint_torque_ext_cmd",
         "--status-topic", f"/{unique_ns('status')}"],
        env=bridge_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        left.wait_started()
        right.wait_started()
        time.sleep(SETTLE_S)

        left_c = Collector(left.topic("pose"), PoseStamped)
        right_c = Collector(right.topic("pose"), PoseStamped)
        left_c.spin_for(RUN_S)
        right_c.spin_for(0.2)

        n = min(len(left_c.messages), len(right_c.messages))
        assert n > 20, f"too few pose samples: left={len(left_c.messages)} right={len(right_c.messages)}"
        errs = []
        for i in range(n):
            lp = left_c.messages[i].pose.position
            rp = right_c.messages[i].pose.position
            errs.append(math.sqrt((lp.x - rp.x) ** 2 + (lp.y - rp.y) ** 2 + (lp.z - rp.z) ** 2))
        left_c.close()
        right_c.close()
        return float(np.sqrt(np.mean(np.square(errs))))
    finally:
        bridge.send_signal(subprocess.signal.SIGINT)
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge.kill()
        left.stop()
        right.stop()


@pytest.mark.skipif(not SCRIPT.exists(), reason=f"bilateral_4ch.py not found at {SCRIPT}")
def test_c4_does_not_worsen_tracking():
    rms_w4_off = _run_session(w4=0.0)
    rms_w4_on = _run_session(w4=0.7)
    assert rms_w4_on <= rms_w4_off * 1.10, (
        f"w4=0.7 RMS tracking error ({rms_w4_on*1000:.3f} mm) is more than 10% "
        f"worse than w4=0 ({rms_w4_off*1000:.3f} mm) -- C4 should tighten "
        f"tracking, or at worst not degrade it"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
