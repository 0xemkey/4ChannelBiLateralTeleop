"""2026-09-10: dob_g_dis_scale, the preferred DOB bandwidth sweep mechanism.

Scales the shipped per-joint g_dis baseline {10,10,5} rad/s (touch_slave.cpp's
kGDis array) rather than collapsing it to one value the way the older
dob_g_dis_override_rad_s_ does -- so scale=1.5 gives {15,15,7.5}, preserving
the paper's per-joint ratio (waist=shoulder=2x elbow).

g_dis is not published on any topic (DobDiagnostics has no such field), so
these tests parse touch_slave_sim's own startup log line ("DOB bank
configured: ... g_dis(w/s/e)=X/X/X rad/s ..."), the same signal a human
operator would read.
"""
from __future__ import annotations

import re
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import SimNode  # noqa: E402

GDIS_RE = re.compile(r"g_dis\(w/s/e\)=([\d.]+)/([\d.]+)/([\d.]+) rad/s")


def _resolved_g_dis(params: dict, ns: str) -> tuple:
    node = SimNode(ns, device="Left", params=params)
    try:
        node.wait_started(timeout=10.0)
        deadline = time.time() + 3.0
        text = ""
        while time.time() < deadline:
            text = node.log_text()
            m = GDIS_RE.search(text)
            if m:
                return tuple(float(g) for g in m.groups())
            time.sleep(0.05)
        raise AssertionError(f"g_dis log line never appeared; log tail:\n{text[-2000:]}")
    finally:
        node.proc.terminate()
        try:
            node.proc.wait(timeout=5.0)
        except Exception:
            node.proc.kill()


@pytest.mark.parametrize("scale,expected", [
    ("1.0", (10.0, 10.0, 5.0)),
    ("1.5", (15.0, 15.0, 7.5)),
    ("2.0", (20.0, 20.0, 10.0)),
])
def test_scale_resolves_expected_g_dis(scale, expected):
    got = _resolved_g_dis({"dob_g_dis_scale": scale}, ns=f"dobscale_{scale.replace('.', '_')}")
    assert got == expected, f"scale={scale}: expected {expected}, got {got}"


def test_default_scale_matches_unmodified_baseline():
    """No dob_g_dis_scale param at all -- must be byte-identical to the
    pre-existing shipped behavior (default 1.0)."""
    got = _resolved_g_dis({}, ns="dobscale_default")
    assert got == (10.0, 10.0, 5.0)


def test_override_still_takes_precedence_over_scale():
    """Backward compatibility: dob_g_dis_override_rad_s_ must still work
    unchanged, and win over dob_g_dis_scale_ when both are set (the scale
    is the newer, preferred tool -- it does not silently supersede the
    override's existing behavior)."""
    got = _resolved_g_dis(
        {"dob_g_dis_scale": "2.0", "dob_g_dis_override_rad_s": "7.0"},
        ns="dobscale_override_precedence",
    )
    assert got == (7.0, 7.0, 7.0)


def test_override_alone_unchanged_from_before_scale_existed():
    got = _resolved_g_dis({"dob_g_dis_override_rad_s": "7.0"}, ns="dobscale_override_alone")
    assert got == (7.0, 7.0, 7.0)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
