#!/usr/bin/env python3
"""Offline reconstruction and comparison of 5 candidate C1/C4 deadband
architectures from ONE captured raw-signal dataset per source node
(diag_deadband_capture.py's output) -- sim-only, no C++ changes, no
hardware-facing parameter changed. See that script's docstring for why
reconstructing offline from the RAW (`tau_ext_nm`) signal is both safe and
methodologically cleaner than launching a separate live run per variant.

VARIANTS
--------
Notation: x[j] = raw tau_ext_nm (N*m, this joint, this instant, from the
capture). w = relay weight (w1 or w4). Cf = force-channel scale (1.0,
matching bilateral_4ch.py's own default and the plan's power-conservation
requirement -- see its own --cf argparse help). mask[j] = 1 unless this
joint is excluded (C1's shoulder exclusion, or a C4 variant that adds one).
threshold1[j] = touch_slave.cpp's kThreshold ("2x measured p99 noise"),
same values bilateral_4ch.py's own THRESHOLD_NM already hand-copies.
floor2 = kMinRelayTorqueNormNm = 0.02 N*m.

(a) CURRENT (2-stage, as shipped):
    stage1[j]   = sign(x[j]) * max(0, |x[j]| - threshold1[j])   -- per-joint,
                  BEFORE weighting (computeForceEstimate, source node)
    weighted[j] = w * Cf * mask[j] * stage1[j]
    norm        = ||weighted||_2                                -- AFTER
                  weighting, combined across all 3 joints (touch_slave.cpp's
                  Cartesian relay, peer node)
    output      = weighted if norm >= floor2 else 0              -- HARD
                  gate, no subtraction: a genuine discontinuity of up to
                  floor2 at the boundary.

(b) CURRENT DEADBAND, INCREASED WEIGHT: identical formula to (a), swept
    over higher w. Not a different architecture -- included because the
    task asks whether raising w1/w4 (the option this investigation was
    told NOT to just apply) is actually competitive with a structural fix.

(c1) STAGE-1-ONLY (drop the norm gate entirely):
    output[j] = w * Cf * mask[j] * stage1[j]                     -- always
                passed through, no stage-2 gate at all. Relies entirely on
                each joint's own per-joint noise floor; a joint whose noise
                floor is mis-estimated has no second line of defense.

(c2) STAGE-2-ONLY (drop the per-joint pre-weight deadband, keep only the
    norm gate):
    weighted[j] = w * Cf * mask[j] * x[j]                        -- RAW
                  signal, no stage-1 subtraction at all
    norm        = ||weighted||_2
    output      = weighted if norm >= floor2 else 0
    Relies entirely on ONE combined floor for noise rejection across all
    joints at once -- a joint with an outsized individual noise floor is
    no longer specifically suppressed.

(d) SINGLE DEADBAND, APPLIED AFTER SUMMATION (a genuinely different,
    continuous-form single stage, calibrated on the COMBINED signal
    instead of stacking two separately-tuned ones):
    weighted[j] = w * Cf * mask[j] * x[j]                        -- RAW
    norm        = ||weighted||_2
    combined_floor = ||[w*Cf*threshold1[j]*mask[j] for j]||_2     -- the
                  weighted RSS of the per-joint noise floors, i.e. what
                  "the same total noise rejection as stage 1, but applied
                  once, on the combined vector" means dimensionally
    reduced_norm = max(0, norm - combined_floor)                  -- SHRINKING
                  form (continuous, like stage 1), not a hard gate
    output[j]   = weighted[j] * (reduced_norm / norm) if norm > 0 else 0

(e) CONTINUOUS/COMPENSATED (removes stage 2's hard discontinuity while
    keeping stage 1 exactly as-is, addressing the specific "up to floor2 N*m
    step into a human hand" defect JointDobParams' own gate_release_ratio
    comment already documents as a concern for a DIFFERENT gate in this
    codebase -- same reasoning applied here):
    stage1[j]   = sign(x[j]) * max(0, |x[j]| - threshold1[j])     -- UNCHANGED
    weighted[j] = w * Cf * mask[j] * stage1[j]
    norm        = ||weighted||_2
    scale       = norm^2 / (norm^2 + floor2^2)                    -- smooth
                  soft-knee, C-infinity, 0 at norm=0, -> 1 for norm >> floor2
    output[j]   = weighted[j] * scale

OSCILLATION-LATCH REPLAY: reuses bilateral_4ch.py's own `analyze_window()`
(the actual per-window pp/zero-crossing math, imported directly, not
reimplemented) inside a faithful reconstruction of `_tick()`'s window/
growing-decision scaffolding (constants and structure copied verbatim --
1.0s windows, 3-window history, kGrowingFloorFrac=0.25, the same
`min_oscillation_hz` gate), run offline against each variant's
reconstructed weighted[](t) stream. This is the SAME decision logic the
real node runs, replayed against reconstructed instead of live data.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Callable, Dict, List, Tuple

REPO_ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(REPO_ROOT / "projects" / "teleop_left_master_right_slave" / "scripts"))
from bilateral_4ch import analyze_window, THRESHOLD_NM, MIN_RELAY_TORQUE_NORM_NM  # noqa: E402
from plot_phase2_scenario import jacobian  # noqa: E402

CAPTURE_DIR = Path("/tmp/deadband_capture")
MAX_TORQUE_NM = {"waist": 0.16200, "shoulder": 0.08100, "elbow": 0.08100}
JOINTS = ("waist", "shoulder", "elbow")


def jacobian_transpose_times(q, f) -> List[float]:
    j = jacobian(q)  # row-major J, 9 elements
    return [
        j[0] * f[0] + j[3] * f[1] + j[6] * f[2],
        j[1] * f[0] + j[4] * f[1] + j[7] * f[2],
        j[2] * f[0] + j[5] * f[1] + j[8] * f[2],
    ]


def load_capture(wall_side: str, pattern: str) -> Dict:
    with open(CAPTURE_DIR / f"{wall_side}_{pattern}.json") as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Variant formulas: each takes (raw_x[3], weight, mask[3]) -> weighted_output[3]
# ---------------------------------------------------------------------------
def _stage1(x, mask):
    return [
        (math.copysign(max(0.0, abs(x[i]) - THRESHOLD_NM[i]), x[i])) * mask[i]
        for i in range(3)
    ]


def variant_a_current(x, w, mask, cf=1.0):
    s1 = _stage1(x, mask)
    weighted = [w * cf * v for v in s1]
    norm = math.sqrt(sum(v * v for v in weighted))
    return weighted if norm >= MIN_RELAY_TORQUE_NORM_NM else [0.0, 0.0, 0.0]


def variant_c1_stage1_only(x, w, mask, cf=1.0):
    s1 = _stage1(x, mask)
    return [w * cf * v for v in s1]


def variant_c2_stage2_only(x, w, mask, cf=1.0):
    weighted = [w * cf * mask[i] * x[i] for i in range(3)]
    norm = math.sqrt(sum(v * v for v in weighted))
    return weighted if norm >= MIN_RELAY_TORQUE_NORM_NM else [0.0, 0.0, 0.0]


def variant_d_single_after_sum(x, w, mask, cf=1.0):
    weighted = [w * cf * mask[i] * x[i] for i in range(3)]
    norm = math.sqrt(sum(v * v for v in weighted))
    combined_floor = math.sqrt(sum((w * cf * THRESHOLD_NM[i] * mask[i]) ** 2 for i in range(3)))
    if norm <= 1e-12:
        return [0.0, 0.0, 0.0]
    reduced = max(0.0, norm - combined_floor)
    scale = reduced / norm
    return [v * scale for v in weighted]


def variant_e_continuous(x, w, mask, cf=1.0):
    s1 = _stage1(x, mask)
    weighted = [w * cf * v for v in s1]
    norm2 = sum(v * v for v in weighted)
    scale = norm2 / (norm2 + MIN_RELAY_TORQUE_NORM_NM ** 2) if norm2 > 0 else 0.0
    return [v * scale for v in weighted]


VARIANTS: Dict[str, Callable] = {
    "a_current": variant_a_current,
    "c1_stage1_only": variant_c1_stage1_only,
    "c2_stage2_only": variant_c2_stage2_only,
    "d_after_summation": variant_d_single_after_sum,
    "e_continuous": variant_e_continuous,
}


# ---------------------------------------------------------------------------
# Oscillation-latch replay -- faithful reconstruction of bilateral_4ch.py's
# _tick() windowing/decision scaffolding around the real analyze_window().
# ---------------------------------------------------------------------------
def simulate_oscillation_trip(t_series: List[float], vec_series: List[List[float]],
                              trip_pp_nm: float = 0.4, min_oscillation_hz: float = 1.5,
                              growing_floor_frac: float = 0.25) -> Dict:
    window_samples: List[Tuple[float, List[float]]] = []
    window_start = t_series[0] if t_series else 0.0
    pp_history: List[float] = []
    tripped = False
    trip_events = []
    for t, vec in zip(t_series, vec_series):
        window_samples.append((t, vec))
        window_samples = [(tt, vv) for tt, vv in window_samples if t - tt <= 1.0]
        if t - window_start >= 1.0:
            pp, zc_hz = analyze_window(window_samples)
            pp_history.append(pp)
            if len(pp_history) > 3:
                pp_history.pop(0)
            window_start = t
            growing = (
                len(pp_history) == 3
                and pp_history[0] < pp_history[1] < pp_history[2]
                and pp_history[2] > growing_floor_frac * trip_pp_nm
                and zc_hz >= min_oscillation_hz
            )
            if (pp > trip_pp_nm or growing) and not tripped:
                tripped = True
                trip_events.append({"t": t, "pp": pp, "growing": growing, "zc_hz": zc_hz})
    return {"tripped": tripped, "trip_events": trip_events, "n_windows": len(pp_history)}


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def analyze_variant_run(capture: Dict, variant_fn: Callable, w: float, mask,
                        peer_q_estimate) -> Dict:
    t_series = [s["t"] for s in capture["samples"]]
    raw_series = [s["raw"] for s in capture["samples"]]
    out_series = [variant_fn(r, w, mask) for r in raw_series]
    out_norms = [math.sqrt(sum(v * v for v in o)) for o in out_series]
    raw_norms = [math.sqrt(sum((r[i] * mask[i]) ** 2 for i in range(3))) for r in raw_series]

    nonzero_out = [n for n in out_norms if n > 1e-9]
    peak_out = max(out_norms) if out_norms else 0.0
    min_active_out = min(nonzero_out) if nonzero_out else 0.0

    peak_raw = max(raw_norms) if raw_norms else 0.0
    transmission_ratio = (peak_out / peak_raw) if peak_raw > 1e-9 else 0.0

    # Small-signal sensitivity: d(output)/d(raw) estimated across consecutive
    # samples straddling the transition band (output goes from ~0 to >0).
    sens_samples = []
    for i in range(1, len(out_norms)):
        d_out = out_norms[i] - out_norms[i - 1]
        d_raw = raw_norms[i] - raw_norms[i - 1]
        if abs(d_raw) > 1e-6 and 0.0 < raw_norms[i] < 2 * THRESHOLD_NM[0] + 0.05:
            sens_samples.append(d_out / d_raw)
    small_signal_gain = (sum(sens_samples) / len(sens_samples)) if sens_samples else None

    # min force at the SOURCE (raw norm, masked) that produced ANY nonzero output
    min_raw_to_activate = None
    for r_norm, o_norm in zip(raw_norms, out_norms):
        if o_norm > 1e-9:
            if min_raw_to_activate is None or r_norm < min_raw_to_activate:
                min_raw_to_activate = r_norm

    # Peer-side torque: touch_slave.cpp's relay converts output[] (a joint
    # torque) to Cartesian force via cartesianForceFromJointTorque (solves
    # J_peer^T F = output for F, using the PEER's own Jacobian), adds it to
    # force_device, clamps by max_force_n_ (1.5N), then the device converts
    # that Cartesian force back to actual joint torque = J_peer^T * F_clamped.
    # AWAY FROM CLAMPING, this round trip is an exact identity regardless of
    # the peer's specific pose (J_peer^T * (J_peer^T)^-1 = I), verified
    # earlier this session via a 2000-pose round-trip test on the same
    # Jacobian math -- so output[] itself IS the peer's actual resulting
    # joint torque, not an approximation, PROVIDED the intermediate Cartesian
    # force never exceeds 1.5N. At this investigation's signal magnitudes
    # (well under 1N in nearly every case below) that holds; treat any case
    # printed near or above ~1N as an UPPER BOUND, not exact, since clamping
    # can only ever reduce it from here, never increase it.
    peak_tau_frac = {}
    rms_tau = {}
    for i, name in enumerate(JOINTS):
        vals = [abs(o[i]) for o in out_series]
        peak = max(vals) if vals else 0.0
        rms = math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else 0.0
        peak_tau_frac[name] = peak / MAX_TORQUE_NM[name]
        rms_tau[name] = rms

    trip = simulate_oscillation_trip(t_series, out_series)

    return {
        "variant": None,  # filled by caller
        "weight": w,
        "peak_output_norm_nm": peak_out,
        "min_active_output_norm_nm": min_active_out,
        "transmission_ratio_peak": transmission_ratio,
        "min_raw_source_to_activate_nm": min_raw_to_activate,
        "small_signal_gain_approx": small_signal_gain,
        "peak_torque_budget_frac": peak_tau_frac,
        "rms_torque_nm": rms_tau,
        "oscillation_tripped": trip["tripped"],
        "trip_events": trip["trip_events"],
    }


def main() -> None:
    c1_mask_shoulder_excl = [1.0, 0.0, 1.0]   # waist, shoulder(excluded), elbow
    c1_mask_full = [1.0, 1.0, 1.0]
    c4_mask_full = [1.0, 1.0, 1.0]
    c4_mask_noshoulder = [1.0, 0.0, 1.0]

    weights = [0.4, 0.5, 0.6, 0.7, 1.0]
    patterns = ["escalating_fast", "escalating_slow", "multiaxis_fast"]

    all_results = []
    for wall_side, mask_sets, label in (
        ("slave", {"c1_masked": c1_mask_shoulder_excl, "c1_unmasked": c1_mask_full}, "C1"),
        ("master", {"c4_unmasked": c4_mask_full, "c4_masked": c4_mask_noshoulder}, "C4"),
    ):
        for pattern in patterns:
            capture = load_capture(wall_side, pattern)
            for mask_name, mask in mask_sets.items():
                for variant_name, variant_fn in VARIANTS.items():
                    for w in weights:
                        m = analyze_variant_run(capture, variant_fn, w, mask, None)
                        m.update({
                            "channel": label, "wall_side": wall_side,
                            "pattern": pattern, "mask_name": mask_name,
                            "variant": variant_name,
                        })
                        all_results.append(m)

    out_path = Path("/tmp/deadband_variant_results.json")
    with open(out_path, "w") as f:
        json.dump(all_results, f, indent=2)
    print(f"wrote {len(all_results)} rows -> {out_path}")


if __name__ == "__main__":
    main()
