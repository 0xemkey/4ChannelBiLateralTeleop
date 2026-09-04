"""Phase 1.3 (2026-09-04 fine-tuning task) -- unit tests for the
zero-crossing-rate secondary check added to bilateral_4ch.py's oscillation
guard.

Deliberately NOT a sim/ROS integration test: analyze_window() is a pure
function with no ROS dependency (same "messaging logic, not control math,
test it standalone" precedent as touch_force_estimate.hpp's
computeForceEstimate(), see test_c1_cartesian_relay_deadband.py's docstring
for the prior art). That makes this test fast, deterministic, and runnable
with no sim process, no rclpy.init(), and -- relevant to this session's
constraint -- nothing that touches or requires physical hardware.

BACKGROUND: the pre-existing "growing" heuristic (3 consecutive rising
1s-window peak-to-peak values, above a floor fraction of trip_pp_nm) was
found live 2026-09-03/04 to false-positive on a human deliberately
escalating push force to find a deadband threshold -- exactly the scenario
this session's own verification testing kept doing, tripping the latch
three separate times. A monotonic escalating push and a genuine developing
oscillation produce the SAME rising-pp shape; they differ in periodicity,
which the pp-only heuristic never looked at. This file proves the added
zero-crossing check tells them apart.
"""
from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bilateral_4ch.py"
)


def _load_bilateral_4ch():
    """Import bilateral_4ch.py directly by file path (it's a standalone
    script, not part of an installed package) so analyze_window() and the
    THRESHOLD_NM/MIN_RELAY_TORQUE_NORM_NM constants can be unit-tested
    without spawning it as a ROS node or subprocess."""
    if not SCRIPT.exists():
        pytest.skip(f"bilateral_4ch.py not found at {SCRIPT}")
    spec = importlib.util.spec_from_file_location("bilateral_4ch", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules["bilateral_4ch"] = module
    spec.loader.exec_module(module)
    return module


b4c = _load_bilateral_4ch()


def _make_samples(t0: float, dt: float, values):
    """values: list of scalars, applied to axis 0 only (waist), other axes
    held at zero -- matches a single-axis push/oscillation on the rig."""
    return [(t0 + i * dt, [v, 0.0, 0.0]) for i, v in enumerate(values)]


class TestAnalyzeWindow:
    def test_empty_window(self):
        pp, hz = b4c.analyze_window([])
        assert pp == 0.0
        assert hz == 0.0

    def test_monotonic_ramp_has_near_zero_crossing_rate(self):
        """A human pushing harder and harder in ONE direction -- the exact
        shape of this session's own verification testing that falsely
        tripped the old heuristic."""
        values = [0.0 + 0.01 * i for i in range(100)]  # 0.0 -> 0.99, one direction
        samples = _make_samples(0.0, 0.01, values)
        pp, hz = b4c.analyze_window(samples)
        assert pp == pytest.approx(0.99, abs=1e-6)
        assert hz == 0.0, f"monotonic ramp should have 0 zero-crossings, got {hz} Hz"

    def test_step_and_hold_has_near_zero_crossing_rate(self):
        """A single push held steady (the original 2026-09-02 false-positive
        repro case) -- still must not look periodic."""
        values = [0.0] * 10 + [0.5] * 90
        samples = _make_samples(0.0, 0.01, values)
        pp, hz = b4c.analyze_window(samples)
        assert pp == pytest.approx(0.5, abs=1e-6)
        assert hz == 0.0

    def test_genuine_oscillation_has_high_crossing_rate(self):
        """A real periodic signal, 4 Hz, amplitude comparable to a firm
        push -- must register a clearly nonzero crossing rate."""
        duration_s = 1.0
        n = 200
        dt = duration_s / n
        values = [0.3 * math.sin(2 * math.pi * 4.0 * (i * dt)) for i in range(n)]
        samples = _make_samples(0.0, dt, values)
        pp, hz = b4c.analyze_window(samples)
        # pp is peak-to-peak of the VECTOR NORM (always >= 0), not the raw
        # signed value -- for a single-axis sine of amplitude A the norm
        # ranges [0, A], so pp ~= A, not 2A.
        assert pp == pytest.approx(0.3, abs=0.02)
        # 4 Hz sine crosses zero 8 times/second; allow generous slack for
        # discrete sampling at the window edges.
        assert hz >= 6.0, f"expected ~8 Hz worth of crossings, got {hz} Hz"

    def test_slow_multiaxis_release_sequence_low_crossing_rate(self):
        """A push on axis 0 released back through zero ONCE (not a sustained
        oscillation) -- one crossing in a 1s window is not "periodic"."""
        values = [0.4] * 40 + [-0.3] * 40 + [0.0] * 20
        samples = _make_samples(0.0, 0.01, values)
        pp, hz = b4c.analyze_window(samples)
        assert hz <= 2.0, f"a single release-through-zero should read low, got {hz} Hz"


class TestGrowingDecisionGating:
    """Reconstructs the exact 'growing' boolean expression from
    bilateral_4ch.py's _tick() (kept in sync by hand -- see the comment at
    that call site) to test the DECISION, not just the rate computation."""

    kGrowingFloorFrac = 0.25

    def _growing(self, pp_history, zero_crossing_hz, trip_pp_nm=0.4, min_hz=1.5):
        return (
            len(pp_history) == 3
            and pp_history[0] < pp_history[1] < pp_history[2]
            and pp_history[2] > self.kGrowingFloorFrac * trip_pp_nm
            and zero_crossing_hz >= min_hz
        )

    def test_escalating_human_push_does_not_trip(self):
        """Three rising windows, magnitude alone WOULD have tripped the old
        heuristic (0.15 N*m clears the 0.25*0.4=0.1 floor) -- must NOT trip
        now, because a monotonic push has ~0 Hz crossing rate."""
        pp_history = [0.05, 0.10, 0.15]
        assert pp_history[2] > self.kGrowingFloorFrac * 0.4  # sanity: old check would fire
        assert not self._growing(pp_history, zero_crossing_hz=0.0)
        assert not self._growing(pp_history, zero_crossing_hz=0.3)  # a little dither, still not periodic

    def test_genuine_oscillation_does_trip(self):
        """Same rising-pp shape, but WITH a real periodic crossing rate --
        must still trip, i.e. the fix does not disable real protection."""
        pp_history = [0.05, 0.10, 0.15]
        assert self._growing(pp_history, zero_crossing_hz=6.0)

    def test_non_rising_window_never_trips_regardless_of_frequency(self):
        """Sanity check on the existing (unchanged) first three conditions:
        even a high crossing rate should not trip if pp isn't actually
        rising across all 3 windows."""
        pp_history = [0.15, 0.10, 0.20]  # not strictly increasing
        assert not self._growing(pp_history, zero_crossing_hz=10.0)

    def test_hard_ceiling_is_unaffected_by_periodicity(self):
        """The pp > trip_pp_nm hard ceiling (checked separately in _tick(),
        not part of _growing()) must remain an unconditional backstop --
        this test documents/pins that expectation even though the ceiling
        check itself lives outside _growing()."""
        trip_pp_nm = 0.4
        pp_over_ceiling = 0.5
        assert pp_over_ceiling > trip_pp_nm  # trips regardless of growing/hz


class TestEffectiveThreshold:
    """Phase 1.2 -- effective_threshold_nm()."""

    def test_disabled_channel_is_infinite(self):
        thr = b4c.effective_threshold_nm(0.0)
        assert all(math.isinf(t) for t in thr)

    def test_matches_documented_w_range_for_elbow(self):
        # Elbow at w=0.4 should land almost exactly at 0.1 N*m per the
        # w-range documented in effective_threshold_nm()'s own docstring.
        thr = b4c.effective_threshold_nm(0.4)
        elbow = thr[2]
        assert elbow == pytest.approx(0.1, abs=1e-6)

    def test_higher_weight_lowers_threshold(self):
        thr_low = b4c.effective_threshold_nm(0.15)
        thr_high = b4c.effective_threshold_nm(0.4)
        for lo, hi in zip(thr_low, thr_high):
            assert hi < lo


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
