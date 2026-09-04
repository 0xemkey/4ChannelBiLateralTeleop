"""Chirp stability sweep under 4ch_safe (plan Sec 4, Stage 2 preview).

Drives the master with hd_sim's chirp hand motion (0.2-20 Hz, deliberately
spanning the rig's documented ~4.4 Hz resonance) while running the full
4ch_safe preset, and checks that the C1 force-channel envelope (peak-to-peak
of the torque this test's own bilateral_4ch.py instance sent toward the
master) does NOT GROW between an early and a late window.

Per the plan: this records a margin, it does not gate on a specific number --
a growing envelope across the sweep is exactly the pumping failure mode this
project has hit before (0.007 N -> 0.65 N, 93x, on the Cartesian bridge). Not
growing is the pass criterion; the actual ratio is only logged, not asserted
to any tight bound, and is the number the deferred Stage 2 hardware chirp test
re-validates against real timing and jitter.
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import Collector, SimNode, unique_ns  # noqa: E402

from touch_haptics.msg import BilateralStatus

SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bilateral_4ch.py"
)

CHIRP_ENV = {
    "TOUCH_SIM_HAND": "stiff",
    "TOUCH_SIM_HAND_MOTION": "chirp",
    "TOUCH_SIM_HAND_F0": "0.2",
    "TOUCH_SIM_HAND_F1": "20.0",
    "TOUCH_SIM_HAND_DUR": "12.0",
    "TOUCH_SIM_HAND_AMP": "0.02",
}
RUN_S = 12.0
GROWTH_FACTOR_LIMIT = 3.0  # loose: catches real pumping (which was 93x), not tuning noise


@pytest.mark.skipif(not SCRIPT.exists(), reason=f"bilateral_4ch.py not found at {SCRIPT}")
def test_chirp_envelope_does_not_grow():
    left_ns = unique_ns("chirpl")
    right_ns = unique_ns("chirpr")
    status_topic = f"/{unique_ns('chirpstatus')}"

    left = SimNode(left_ns, device="Left",
                   params={"joint_space_control": "true"}, sim_env=CHIRP_ENV)
    right = SimNode(right_ns, device="Default Device",
                    params={"joint_space_control": "true"})
    bridge = subprocess.Popen(
        [sys.executable, str(SCRIPT), "--architecture", "4ch_safe",
         "--left-force-topic", f"/{left_ns}/force_ext_hat",
         "--right-force-topic", f"/{right_ns}/force_ext_hat",
         "--left-joint-angles-topic", f"/{left_ns}/joint_angles",
         "--right-joint-angles-topic", f"/{right_ns}/joint_angles",
         "--left-joint-target-topic", f"/{left_ns}/joint_target",
         "--right-joint-target-topic", f"/{right_ns}/joint_target",
         "--left-joint-torque-ext-topic", f"/{left_ns}/joint_torque_ext_cmd",
         "--right-joint-torque-ext-topic", f"/{right_ns}/joint_torque_ext_cmd",
         "--status-topic", status_topic],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        left.wait_started()
        right.wait_started()
        time.sleep(1.0)

        c = Collector(status_topic, BilateralStatus)
        try:
            c.spin_for(RUN_S)
            msgs = c.messages
        finally:
            c.close()

        assert len(msgs) > 40, f"too few BilateralStatus samples: {len(msgs)}"

        mags = np.array([
            np.linalg.norm(m.master_torque_cmd_nm) for m in msgs
        ])
        n = len(mags)
        early = mags[: n // 3]
        late = mags[-n // 3:]
        early_pp = float(early.max() - early.min()) if len(early) else 0.0
        late_pp = float(late.max() - late.min()) if len(late) else 0.0

        ratio = late_pp / early_pp if early_pp > 1e-9 else (
            float("inf") if late_pp > 1e-6 else 1.0)
        print(f"\n[chirp stability] early_pp={early_pp:.5f} N*m "
              f"late_pp={late_pp:.5f} N*m ratio={ratio:.2f} "
              f"(recorded margin, not a hard bound)")

        assert ratio <= GROWTH_FACTOR_LIMIT, (
            f"C1 envelope grew {ratio:.1f}x from early to late window "
            f"(early_pp={early_pp:.5f}, late_pp={late_pp:.5f} N*m) -- this is "
            f"the pumping signature documented at 93x on the Cartesian bridge"
        )
    finally:
        bridge.send_signal(subprocess.signal.SIGINT)
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge.kill()
        left.stop()
        right.stop()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
