#!/usr/bin/env python3
"""Y-axis anomaly investigation (per user instruction, 2026-09-09): is the
near-zero net Y displacement and 10.8-12x Y->X cross-coupling found in
Phase 5's Y-axis experiment a push-MAGNITUDE artifact (0.4N too small for
Y specifically) or a genuine dynamic/kinematic property of this arm at
its rest pose? Tests baseline_asymmetric ONLY (config comparison is not
the question here) at 3 push magnitudes: 0.4N (matches the original
Phase 5 Y run, included for direct comparison), 1.0N, 2.0N. N=5 each.

100% simulation, same infrastructure as phase5_position_subsystem.py
(touch_slave_sim/hd_sim + bridge_master_slave.py). No hardware.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from phase5_position_subsystem import one_run, CONFIGS  # noqa: E402

MAGNITUDES = [0.4, 1.0, 2.0]
N_REPEATS = 5
OUT_JSON = Path(__file__).parent / "phase5_y_magnitude_raw.json"


def main() -> None:
    import phase5_position_subsystem as mod
    results = []
    for mag in MAGNITUDES:
        mod.PUSH_MAG = mag  # override module-level constant used inside one_run
        for r in range(N_REPEATS):
            rec = one_run("baseline_asymmetric", CONFIGS["baseline_asymmetric"], r, "y")
            rec["push_mag"] = mag
            n = len(rec.get("series", {}).get("t", []))
            print(f"mag={mag:.1f}N run {r}: n_samples={n} {rec.get('error', '')}")
            results.append(rec)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f)
    print(f"\nwrote {OUT_JSON}")


if __name__ == "__main__":
    main()
