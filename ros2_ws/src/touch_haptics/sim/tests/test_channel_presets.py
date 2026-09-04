"""Each architecture preset in bilateral_4ch.py must publish the documented
channel_weight[] (plan Sec 3.5's table) and a consistent channel_active[]/
channel_reason[].

Runs bilateral_4ch.py standalone, with NO upstream sim nodes -- this isolates
the preset/ramp/status-publishing logic from physics entirely, so it is fast
and cannot be broken by an unrelated sim regression. Because nothing publishes
force_ext_hat or joint_angles, every weight>0 channel is expected to read
STALE (never ZERO_WEIGHT, never OK) -- that split is itself the thing being
checked: ZERO_WEIGHT channels must stay ZERO_WEIGHT regardless of freshness,
and weighted channels must not silently read OK with no real input.
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import Collector, unique_ns  # noqa: E402

from touch_haptics.msg import BilateralStatus

SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bilateral_4ch.py"
)

# C1, C2, C3, C4 -- must match bilateral_4ch.py's PRESETS table exactly.
EXPECTED = {
    "unilateral": (0.0, 0.0, 1.0, 0.0),
    "pp": (0.0, 1.0, 1.0, 0.0),
    "pf": (1.0, 0.0, 1.0, 0.0),
    "4ch_safe": (0.5, 1.0, 1.0, 0.3),
    "4ch": (1.0, 1.0, 1.0, 1.0),
}

REASON_ZERO_WEIGHT = 1
REASON_STALE = 2


@pytest.mark.skipif(not SCRIPT.exists(), reason=f"bilateral_4ch.py not found at {SCRIPT}")
@pytest.mark.parametrize("architecture", sorted(EXPECTED))
def test_preset_weights_and_reasons(architecture: str):
    status_topic = f"/{unique_ns('status')}"
    proc = subprocess.Popen(
        [sys.executable, str(SCRIPT), "--architecture", architecture,
         "--status-topic", status_topic,
         "--ramp-time-s", "0.2",  # short, so the test does not need to wait long
         # No upstream topics given (defaults point at devices that do not
         # exist in this test) -- deliberate, see module docstring.
         ],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        time.sleep(1.0)  # let the process come up and the ramp settle
        c = Collector(status_topic, BilateralStatus)
        try:
            c.wait_for(3, timeout=10.0)
            msg = c.messages[-1]
        finally:
            c.close()

        expected = EXPECTED[architecture]
        got = tuple(round(w, 3) for w in msg.channel_weight)
        assert got == expected, (
            f"{architecture}: channel_weight={got}, expected {expected}"
        )

        for i in range(4):
            if expected[i] == 0.0:
                assert msg.channel_reason[i] == REASON_ZERO_WEIGHT, (
                    f"{architecture} C{i+1}: weight is 0 but reason="
                    f"{msg.channel_reason[i]}, expected ZERO_WEIGHT"
                )
                assert not msg.channel_active[i]
            else:
                assert msg.channel_reason[i] == REASON_STALE, (
                    f"{architecture} C{i+1}: weight={expected[i]} with no "
                    f"upstream publishers should read STALE, got "
                    f"{msg.channel_reason[i]}"
                )
                assert not msg.channel_active[i], (
                    f"{architecture} C{i+1}: reads ACTIVE with no real input "
                    f"-- a channel must never appear live on stale data"
                )
    finally:
        proc.send_signal(subprocess.signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
