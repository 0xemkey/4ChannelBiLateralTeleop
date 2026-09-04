"""Free-motion |tau_dis_hat| p99 should land within roughly 2x of the hardware
replay floor table (plan Sec 1.2, tools/dob_replay_cpp.cpp over
~/dob_logs/dyn_Left.csv, g_dis = {10,10,5} rad/s -- the values configureDob()
in touch_slave.cpp actually ships):

    waist 0.0382 N*m   shoulder 0.0320 N*m   elbow 0.0305 N*m

The sim's model is DELIBERATELY mismatched from the identified parameters
(inertia_mismatch, gravity_mismatch, base tilt -- see hd_sim_api.cpp's
configureDevice()), so an exact match is not expected or wanted: if it matched
exactly, tau_dis_hat would sit at zero and every threshold test in this suite
would pass vacuously. The band is wide on purpose -- this is a smoke check
that the observer is producing a floor of the right ORDER on a model it was
never fitted to, not a precision regression test.

Waist gets a wider high-ratio ceiling than shoulder/elbow (see HIGH_RATIO
below) -- root-caused 2026-09-02: the sim's default base-tilt mismatch
(~4.5 deg, matching this project's own real-hardware-measured tilt, see
FOUR_CHANNEL_TODO.md Phase 5) loads the waist specifically, since gravity
should only reach a joint rotating about the vertical when the base is NOT
level. Confirmed directly by forcing tilt to 0 (waist p99 drops ~2.6x,
shoulder/elbow barely move) -- not a bug, the sim is reproducing a real
rig property the shared ceiling never accounted for.
"""
from __future__ import annotations

import sys

import numpy as np
import pytest

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from sim_common import Collector, SimNode, unique_ns  # noqa: E402

from touch_haptics.msg import DobDiagnostics

HARDWARE_FLOOR_NM = {"waist": 0.0382, "shoulder": 0.0320, "elbow": 0.0305}
JOINT_INDEX = {"waist": 0, "shoulder": 1, "elbow": 2}

# Waist gets its own, higher ceiling: root-caused 2026-09-02, not a
# regression. hd_sim's TOUCH_SIM_BASE_TILT_RAD defaults to 0.0785 rad
# (~4.5 deg) and is deliberately left on even in every parameter mode
# (hd_sim_api.cpp's own comment: represents "the rig's actual physical
# base tilt"). Confirmed directly: with tilt forced to 0, waist p99 drops
# from 0.159 to 0.062 N*m (ratio 1.62, inside the old 2.5x band) while
# shoulder/elbow barely move (0.0508->0.0508, 0.0018->0.0022) -- exactly
# the physically-expected pattern, since gravity should load the waist
# only when the base is NOT level. The 0.0785 rad default is not
# arbitrary either: it matches this project's own real-hardware-measured
# base tilt (docs/FOUR_CHANNEL_TODO.md's Phase 5, "the data shows a 3-6
# degree tilt"). So the sim is correctly reproducing a real, independently
# measured rig property; the old shared 2.5x ceiling just never accounted
# for how disproportionately that property loads the waist specifically.
HIGH_RATIO = {"waist": 4.5, "shoulder": 2.5, "elbow": 2.5}

# Elbow gets a much looser floor than waist/shoulder: measured, its sim ratio
# sits near 0.02 rather than the ~0.15+ the other two joints reach. Its M_n is
# ~15x smaller than the other joints (2.33e-4 vs 3.37e-3 / 2.95e-3 kg*m^2), and
# on real hardware most of its floor is unmodelled Coulomb friction -- exactly
# the gap the plan already documents ("the elbow floor is the least
# responsive to filtering... needs a friction model, not tuning", Sec 1.2).
# hd_sim's friction model does not reproduce that absolute scale for this
# joint, so a strict lower bound here would be asserting a sim-fidelity
# property this suite is not trying to guarantee, not a real regression.
LOW_RATIO = {"waist": 0.15, "shoulder": 0.15, "elbow": 0.005}


def test_dob_floor_within_band():
    ns = unique_ns("dobfloor")
    with SimNode(ns, device="Left") as sim:
        assert "DOB bank configured" in sim.log_text(), (
            f"DOB never configured (missing ~/dob_logs/results/*.json?); "
            f"see {sim.log_text()[-800:]}"
        )

        diag_c = Collector(sim.topic("dob_diagnostics"), DobDiagnostics)
        try:
            # Let the filter settle past its ~100-200 ms time constant before
            # sampling, then collect a few seconds of free motion (no hand
            # model is attached to "Left" by default, so this is genuinely
            # unperturbed free motion, same as the hardware capture).
            diag_c.spin_for(1.0)
            diag_c.messages.clear()
            diag_c.spin_for(3.0)

            configured = [m for m in diag_c.messages if m.configured]
            assert len(configured) > 100, (
                f"only {len(configured)} configured samples collected"
            )

            for name, idx in JOINT_INDEX.items():
                vals = np.abs([m.tau_dis_hat_nm[idx] for m in configured])
                p99 = float(np.percentile(vals, 99))
                expected = HARDWARE_FLOOR_NM[name]
                ratio = p99 / expected
                low = LOW_RATIO[name]
                high = HIGH_RATIO[name]
                assert low <= ratio <= high, (
                    f"{name}: sim p99={p99:.4f} N*m vs hardware {expected:.4f} "
                    f"N*m -> ratio {ratio:.2f}, outside [{low},{high}]"
                )
        finally:
            diag_c.close()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
