#!/usr/bin/env python3
"""Write a C1/C4 qualification artifact only from a valid recorded trial.

2026-09-10 hardening (full4ch pre-tuning closure, Task 2): the previous
version of this script gated ONLY on `analyze_manual_motion.py`'s
`validation.overall_result == "VALID"` plus channel-isolation fields.
That analyzer's `validate_trial()` never reads force evidence at all (see
`validation_provenance/CURRENT_FULL4CH_AUDIT.md` Part 6) -- a trial where
the relay applied exactly zero force, or applied it with the wrong sign,
would still have been written VALID as long as the SOURCE arm moved
enough and channel config matched. This was not exercised by the two
artifacts already on disk (both have real, non-trivial, correctly-signed
force evidence -- verified independently in the audit), but the gate
itself provided no structural guarantee against a future vacuous pass.

This version computes force evidence directly from the trial's raw CSVs
(`raw_bilateral.csv` for source disturbance / commanded relay torque,
the receiving arm's `raw_master.csv`/`raw_slave.csv` for relay_valid/
relay_fresh/relay_applied/relay_force_n/applied_force_n), independent of
whichever analyzer produced validation.json/metrics.json, and REQUIRES
all of the checks below before writing an artifact. Does NOT touch or
regenerate any existing artifact -- this is a gate for FUTURE runs only.

THRESHOLDS: where a value is not independently derived (documented
inline), it is marked PROVISIONAL and the reasoning for the chosen number
is given -- per this task's explicit instruction not to manufacture false
precision. Provisional thresholds were chosen to comfortably admit the
two known-real historical qualification trials while still rejecting a
near-zero/absent-force or wrong-sign trial.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path
from typing import Optional

SCHEMA_VERSION = "2.0.0"
QUALIFICATION_TYPE = "C1_C4_FORCE_RELAY"

JOINTS = ("waist", "shoulder", "elbow")

# Reused, not invented: touch_slave.cpp's own kMinRelayTorqueNormNm --
# the exact floor the REAL relay-apply gate uses in production to decide
# a signal is a deliberate disturbance rather than resting-state DOB
# noise (documented there: resting residual tops out ~0.03-0.05 N*m,
# real contact reaches 0.03-0.4+ N*m; 0.02 sits below the real-contact
# range). Reusing it here means a qualification trial is held to the
# SAME floor the production relay itself already enforces at runtime.
MIN_SOURCE_DISTURBANCE_NM = 0.02
MIN_COMMANDED_RELAY_TORQUE_NM = 0.02

# PROVISIONAL: no independently derived force-telemetry noise floor
# exists for this project (see audit Part 12/6 -- not previously
# measured). Chosen to sit an order of magnitude above plausible
# quantization/publish-rate noise on a Cartesian-converted N*m->N value,
# while remaining well below both known-real trials' evidence (C1 0.74N
# peak relay force, C4 0.23N peak). Revisit with a measured quiet-state
# noise floor (same method as dob_g_dis / gravity-comp floors elsewhere
# in this project) before trusting this near a real boundary case.
MIN_RECEIVING_RELAY_FORCE_N = 0.01

# relay_valid/relay_fresh must hold for essentially the whole trial --
# these are the SAME booleans the production stale-timeout safety gate
# uses; a qualification trial where the relay was frequently stale/invalid
# is not evidence the relay path works, it is evidence it mostly didn't
# run. 0.99 (not 1.0) tolerates a single dropped sample at ~200Hz without
# manufacturing false precision about jitter.
MIN_RELAY_VALID_DUTY = 0.99
MIN_RELAY_FRESH_DUTY = 0.99

# Must be nonzero: an artifact from a trial where the relay force NEVER
# cleared the production deadband (touch_slave.cpp's own 0.02 N*m floor)
# would mean the receiving arm never actually felt anything.
MIN_RELAY_APPLIED_DUTY = 0.0  # strict > this, i.e. duty must be > 0

# PROVISIONAL: no bootstrap/null-distribution derivation of a
# statistically-significant correlation threshold exists for this
# project yet. Chosen to sit below both known-real trials' weakest
# evidence (C4's 0.737 source->relay-force correlation, the weaker of
# the two channels) while rejecting near-zero/random correlation.
# Revisit with a proper null-distribution derivation before trusting
# this near 0.5-0.6.
MIN_ABS_SIGN_CORRELATION = 0.4

# Expected sign of the source-disturbance -> commanded-relay-torque
# correlation, per joint, matching the documented formulas in
# bilateral_4ch.py / validation_provenance/ARCHITECTURE_FREEZE_4CH.md:
#   C1: tau_c1 = -w1 * cf * tau_e_hat * mask  -> NEGATIVE correlation expected
#   C4: tau_c4 = +w4 * cf * tau_h_hat          -> POSITIVE correlation expected
EXPECTED_SIGN = {"c1": -1, "c4": +1}


def _read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open() as f:
        return list(csv.DictReader(f))


def _f(row: dict, key: str) -> Optional[float]:
    v = row.get(key)
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def _b(row: dict, key: str) -> Optional[bool]:
    v = row.get(key)
    if v is None or v == "":
        return None
    return v.strip().lower() in ("true", "1")


def _pearson(xs: list[float], ys: list[float]) -> Optional[float]:
    if len(xs) < 3 or len(xs) != len(ys):
        return None
    if statistics.pstdev(xs) < 1e-12 or statistics.pstdev(ys) < 1e-12:
        return None  # constant signal -- correlation undefined, not zero
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    dx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    dy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if dx < 1e-12 or dy < 1e-12:
        return None
    return num / (dx * dy)


def compute_force_evidence(trial_dir: Path, channel: str, disabled_joints: set[str]) -> dict:
    """Independent of analyze_manual_motion.py's own metrics -- reads the
    raw CSVs directly. `disabled_joints` are excluded from the
    per-joint sign-correlation requirement (a masked joint has no relay
    contribution by construction and cannot be expected to show one)."""
    bilateral_rows = _read_csv(trial_dir / "raw_bilateral.csv")
    receiving_csv = "raw_master.csv" if channel == "c1" else "raw_slave.csv"
    receiving_rows = _read_csv(trial_dir / receiving_csv)

    source_key = "tau_e_hat_nm" if channel == "c1" else "tau_h_hat_nm"
    command_key = "master_torque_cmd_nm" if channel == "c1" else "slave_torque_cmd_nm"

    evidence: dict = {"channel": channel, "problems": []}

    if not bilateral_rows:
        evidence["problems"].append("raw_bilateral.csv missing or empty -- no force-relay evidence possible")
        return evidence
    if not receiving_rows:
        evidence["problems"].append(f"{receiving_csv} missing or empty -- no receiving-side evidence possible")
        return evidence

    per_joint = {}
    for j in JOINTS:
        src = [_f(r, f"{source_key}_{j}") for r in bilateral_rows]
        cmd = [_f(r, f"{command_key}_{j}") for r in bilateral_rows]
        src = [v for v in src if v is not None]
        cmd = [v for v in cmd if v is not None]
        peak_src = max((abs(v) for v in src), default=0.0)
        peak_cmd = max((abs(v) for v in cmd), default=0.0)
        pair_src = [_f(r, f"{source_key}_{j}") for r in bilateral_rows]
        pair_cmd = [_f(r, f"{command_key}_{j}") for r in bilateral_rows]
        paired = [(s, c) for s, c in zip(pair_src, pair_cmd) if s is not None and c is not None]
        corr = _pearson([s for s, _ in paired], [c for _, c in paired]) if paired else None
        per_joint[j] = {"peak_source_disturbance_nm": peak_src, "peak_commanded_relay_torque_nm": peak_cmd,
                        "source_to_command_correlation": corr, "masked": j in disabled_joints}
    evidence["per_joint"] = per_joint

    relay_force_norms = []
    relay_valid_flags = []
    relay_fresh_flags = []
    relay_applied_flags = []
    applied_force_norms = []
    for r in receiving_rows:
        fx, fy, fz = _f(r, "relay_force_n_x"), _f(r, "relay_force_n_y"), _f(r, "relay_force_n_z")
        if fx is not None and fy is not None and fz is not None:
            relay_force_norms.append(math.sqrt(fx * fx + fy * fy + fz * fz))
        rv, rf, ra = _b(r, "relay_valid"), _b(r, "relay_fresh"), _b(r, "relay_applied")
        if rv is not None:
            relay_valid_flags.append(rv)
        if rf is not None:
            relay_fresh_flags.append(rf)
        if ra is not None:
            relay_applied_flags.append(ra)
        ax, ay, az = _f(r, "applied_force_n_x"), _f(r, "applied_force_n_y"), _f(r, "applied_force_n_z")
        if ax is not None and ay is not None and az is not None:
            applied_force_norms.append(math.sqrt(ax * ax + ay * ay + az * az))

    evidence["peak_receiving_relay_force_n"] = max(relay_force_norms, default=0.0)
    evidence["peak_receiving_applied_force_n"] = max(applied_force_norms, default=0.0)
    evidence["relay_valid_duty"] = (sum(relay_valid_flags) / len(relay_valid_flags)
                                    if relay_valid_flags else None)
    evidence["relay_fresh_duty"] = (sum(relay_fresh_flags) / len(relay_fresh_flags)
                                    if relay_fresh_flags else None)
    evidence["relay_applied_duty"] = (sum(relay_applied_flags) / len(relay_applied_flags)
                                      if relay_applied_flags else None)

    required_joints = [j for j in JOINTS if j not in disabled_joints]
    expected_sign = EXPECTED_SIGN[channel]

    max_src = max((per_joint[j]["peak_source_disturbance_nm"] for j in required_joints), default=0.0)
    if max_src < MIN_SOURCE_DISTURBANCE_NM:
        evidence["problems"].append(
            f"peak source disturbance {max_src:.4f} N*m (best required joint) below "
            f"MIN_SOURCE_DISTURBANCE_NM={MIN_SOURCE_DISTURBANCE_NM} -- insufficient excitation to "
            f"attribute any force behavior to")

    max_cmd = max((per_joint[j]["peak_commanded_relay_torque_nm"] for j in required_joints), default=0.0)
    if max_cmd < MIN_COMMANDED_RELAY_TORQUE_NM:
        evidence["problems"].append(
            f"peak commanded relay torque {max_cmd:.4f} N*m below "
            f"MIN_COMMANDED_RELAY_TORQUE_NM={MIN_COMMANDED_RELAY_TORQUE_NM} -- relay never actually "
            f"commanded a deliberate signal")

    if evidence["peak_receiving_relay_force_n"] < MIN_RECEIVING_RELAY_FORCE_N:
        evidence["problems"].append(
            f"peak receiving relay_force_n norm {evidence['peak_receiving_relay_force_n']:.4f} N below "
            f"MIN_RECEIVING_RELAY_FORCE_N={MIN_RECEIVING_RELAY_FORCE_N} (provisional) -- receiving arm "
            f"never measurably felt the relay")

    if evidence["relay_valid_duty"] is None or evidence["relay_valid_duty"] < MIN_RELAY_VALID_DUTY:
        evidence["problems"].append(
            f"relay_valid_duty={evidence['relay_valid_duty']} below {MIN_RELAY_VALID_DUTY}")
    if evidence["relay_fresh_duty"] is None or evidence["relay_fresh_duty"] < MIN_RELAY_FRESH_DUTY:
        evidence["problems"].append(
            f"relay_fresh_duty={evidence['relay_fresh_duty']} below {MIN_RELAY_FRESH_DUTY}")
    if evidence["relay_applied_duty"] is None or evidence["relay_applied_duty"] <= MIN_RELAY_APPLIED_DUTY:
        evidence["problems"].append(
            f"relay_applied_duty={evidence['relay_applied_duty']} not > {MIN_RELAY_APPLIED_DUTY} -- relay "
            f"force never cleared the production deadband during this trial")

    for j in required_joints:
        corr = per_joint[j]["source_to_command_correlation"]
        if corr is None:
            evidence["problems"].append(f"{j}: source_to_command_correlation UNDEFINED "
                                        f"(constant or insufficient signal) -- cannot verify sign")
            continue
        signed_ok = (corr <= -MIN_ABS_SIGN_CORRELATION) if expected_sign < 0 else (corr >= MIN_ABS_SIGN_CORRELATION)
        per_joint[j]["sign_check_passed"] = signed_ok
        if not signed_ok:
            evidence["problems"].append(
                f"{j}: source_to_command_correlation={corr:.4f} does not meet expected "
                f"{'<=-' if expected_sign < 0 else '>=+'}{MIN_ABS_SIGN_CORRELATION} threshold "
                f"(channel={channel})")

    evidence["required_joints"] = required_joints
    evidence["passed"] = not evidence["problems"]
    return evidence


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial-dir", type=Path, required=True)
    parser.add_argument("--channel", choices=["c1", "c4"], required=True)
    parser.add_argument("--session", required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--disabled-joints", default="",
                        help="comma-separated joints excluded from this channel's relay "
                             "(e.g. c1-disable-joints value actually used for this trial's launch)")
    args = parser.parse_args()

    manifest = json.loads((args.trial_dir / "manifest.json").read_text())
    validation = json.loads((args.trial_dir / "validation.json").read_text())
    metrics = json.loads((args.trial_dir / "metrics.json").read_text())

    if validation.get("overall_result") != "VALID":
        raise SystemExit(f"refusing artifact: validation={validation.get('overall_result')}")
    expected = manifest.get("expected_channel_configuration", {})
    if expected.get(args.channel) is not True:
        raise SystemExit(f"refusing artifact: expected {args.channel}=true not present in manifest")
    isolation = validation.get("channel_isolation", {})
    if not isolation.get("live_channel_active_available"):
        raise SystemExit("refusing artifact: no live bilateral_status channel-active evidence")
    if isolation.get("missing_expected_activation"):
        raise SystemExit(f"refusing artifact: {isolation['missing_expected_activation']}")

    disabled = {j.strip() for j in args.disabled_joints.split(",") if j.strip()}
    force_evidence = compute_force_evidence(args.trial_dir, args.channel, disabled)
    if not force_evidence["passed"]:
        raise SystemExit(
            "refusing artifact: force-relay evidence insufficient:\n  - "
            + "\n  - ".join(force_evidence["problems"])
        )

    args.out_dir.mkdir(parents=True, exist_ok=True)
    artifact = {
        "schema_version": SCHEMA_VERSION,
        "qualification_type": QUALIFICATION_TYPE,
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "channel": args.channel.upper(),
        "session": args.session,
        "source_session": args.session,
        "trial_dir": str(args.trial_dir),
        "git_head": manifest.get("git_head"),
        "git_dirty": manifest.get("git_dirty"),
        "git_branch": manifest.get("git_branch"),
        "config_fingerprint": manifest.get("config_fingerprint"),
        "analyzer_version": "analyze_manual_motion.py (validation/isolation) + "
                            "write_qualification_artifact.py:force_evidence v2 (force gate)",
        "expected_channel_configuration": expected,
        "manifest_expected_channel_configuration": expected,
        "validation": validation,
        "force_evidence": force_evidence,
        "metrics_summary": {
            "analysis_direction": metrics.get("analysis_direction"),
            "source_role": metrics.get("source_role"),
            "follower_role": metrics.get("follower_role"),
            "source_peak_y_m": metrics.get("source_peak_y_m"),
            "follower_peak_y_m": metrics.get("follower_peak_y_m"),
            "positive_motion_gain": metrics.get("positive_motion_gain"),
            "force_dob": metrics.get("force_dob"),
        },
        "verdict": "QUALIFIED",
    }
    out = args.out_dir / f"{args.channel.upper()}_QUALIFIED.json"
    out.write_text(json.dumps(artifact, indent=2))
    print(f"[qualification] wrote {out}")
    print(f"[qualification] force evidence: {json.dumps(force_evidence, indent=2)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
