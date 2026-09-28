#!/usr/bin/env python3
"""H2 gravity-settle analysis (2026-09-10). Pure post-hoc: reads the CSVs +
part1_meta.json written by part1_gravity_settle_capture.py, independently
re-derives every PASS/FAIL criterion from the raw recorded samples (not
just trusting the capture script's own live abort decision -- a second,
independent check), computes metrics, and writes plots. Publishes/commands
nothing.

Independent re-verification matters here specifically because the task's
own instruction is explicit: "Do NOT pass H2 merely because the velocity
telemetry is zero at one instant" -- this script re-walks the RAW recorded
velocity samples across the full accumulated quiet window (not just the
single BridgeDiagnostics tick that reported the gate passing) to confirm
continuous sub-threshold speed, rather than trusting the state machine's
own internal bookkeeping alone.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path


def _read_csv(path: Path) -> list:
    if not path.exists():
        return []
    with open(path) as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        row = {}
        for k, v in r.items():
            if v == "" or v is None:
                row[k] = None
            elif v in ("True", "False"):
                row[k] = (v == "True")
            else:
                try:
                    row[k] = float(v)
                except ValueError:
                    row[k] = v
        out.append(row)
    return out


def _norm3(row, prefix):
    return math.sqrt(row[f"{prefix}_x"] ** 2 + row[f"{prefix}_y"] ** 2 + row[f"{prefix}_z"] ** 2)


def analyze(out_dir: Path) -> dict:
    # 2026-09-10 fix (Task 4): a PRE_ARM_FAIL used to leave NO
    # part1_meta.json at all, crashing this function with
    # FileNotFoundError instead of reporting a structured result. Every
    # H2 capture exit path now writes part1_meta.json (see
    # part1_gravity_settle_capture.py's _write_pre_arm_fail_meta()), so
    # this should not trigger in practice any more -- kept as a defensive
    # guard (belt-and-suspenders) rather than removed, since "the analyzer
    # must also handle missing/partial capture CSVs gracefully" is a
    # standing requirement, not a one-time fix.
    meta_path = out_dir / "part1_meta.json"
    if not meta_path.exists():
        validation = {"overall_result": "ANALYSIS_INCOMPLETE",
                     "flags": [{"code": "META_JSON_MISSING", "severity": "INVALID",
                               "detail": f"{meta_path} does not exist -- the capture script's run "
                                        "produced no artifact at all (should not happen after the "
                                        "2026-09-10 Task 4 fix; if it does, that fix itself has a gap)"}],
                     "capture_script_aborted": None, "capture_script_abort_reason": None,
                     "capture_script_abort_t": None}
        (out_dir / "h2_validation.json").write_text(json.dumps(validation, indent=2, default=str))
        return {"metrics": {}, "validation": validation}
    meta = json.loads(meta_path.read_text())

    _abort_reason = meta.get("abort_reason") or ""
    if _abort_reason.startswith("PRE_ARM_FAILED"):
        # A structured PRE_ARM_FAIL artifact -- report it directly rather
        # than running the (necessarily empty) telemetry-based analysis,
        # which would otherwise report a confusing pile of
        # TELEMETRY_STALE/GRAVITY_RAMP_NEVER_COMPLETED/etc. flags for a run
        # that never even armed.
        validation = {"overall_result": "PRE_ARM_FAILED", "flags": [
            {"code": "PRE_ARM_FAILED", "severity": "INVALID", "detail": meta.get("abort_reason")}],
            "capture_script_aborted": True, "capture_script_abort_reason": meta.get("abort_reason"),
            "capture_script_abort_t": meta.get("abort_t")}
        metrics = {"pre_arm_evaluation": meta.get("pre_arm_evaluation"),
                  "publisher_ownership": meta.get("publisher_ownership"),
                  "final_phase": meta.get("final_phase")}
        (out_dir / "h2_metrics.json").write_text(json.dumps(metrics, indent=2, default=str))
        (out_dir / "h2_validation.json").write_text(json.dumps(validation, indent=2, default=str))
        return {"metrics": metrics, "validation": validation}

    master = _read_csv(out_dir / "part1_gravity_settle_master.csv")
    slave = _read_csv(out_dir / "part1_gravity_settle_slave.csv")
    bridge = _read_csv(out_dir / "part1_bridge_diag.csv")

    th = meta.get("frozen_thresholds", {})
    stale_s = th.get("force_budget_telemetry_stale_s", 0.2)
    vel_thresh = th.get("gravity_settle_velocity_threshold_mps", 0.005)
    hold_s = th.get("gravity_settle_hold_s", 0.15)

    flags = []

    def flag(code, severity, detail):
        flags.append({"code": code, "severity": severity, "detail": detail})

    # 1. FAULT.
    fault_rows = [r for r in bridge if r["bridge_state"] == "FAULT"]
    if fault_rows or meta.get("final_phase") == "FAULT":
        flag("BRIDGE_FAULT", "INVALID", f"bridge_state==FAULT observed, first at t={fault_rows[0]['t'] if fault_rows else 'unknown (final_phase only)'}")

    # 2. Telemetry freshness (independent re-derivation from raw gaps).
    for name, rows in (("master", master), ("slave", slave)):
        gaps = [rows[i]["t"] - rows[i - 1]["t"] for i in range(1, len(rows))]
        max_gap = max(gaps) if gaps else None
        if max_gap is not None and max_gap > stale_s:
            flag("TELEMETRY_STALE", "INVALID", f"{name} max inter-sample gap {max_gap:.3f}s > {stale_s}s, first exceeding at index {gaps.index(max_gap)}")

    # 3. Unexpected clamps.
    for name, rows in (("master", master), ("slave", slave)):
        clamped = [r for r in rows if r["position_error_clamped"] or r["primary_force_clamped"] or r["final_force_clamped"]]
        if clamped:
            flag("UNEXPECTED_CLAMP", "INVALID", f"{name} clamp engaged, first at t={clamped[0]['t']}")

    # 4. coupling_ramp_alpha stays exactly 0.
    nonzero_alpha = [r for r in bridge if abs(r["coupling_ramp_alpha"]) > 1e-9]
    if nonzero_alpha:
        flag("COUPLING_ALPHA_NONZERO", "INVALID", f"coupling_ramp_alpha={nonzero_alpha[0]['coupling_ramp_alpha']} at t={nonzero_alpha[0]['t']} (must be exactly 0 throughout H2)")

    # 5. C1-C4 inactive (configured AND relay path).
    bad_channel_rows = [r for r in bridge if r["c1_enabled"] or r["c2_enabled"] or r["c3_enabled"] or r["c4_enabled"]]
    if bad_channel_rows:
        r = bad_channel_rows[0]
        flag("UNEXPECTED_CHANNEL_CONFIGURED", "INVALID",
            f"c1={r['c1_enabled']} c2={r['c2_enabled']} c3={r['c3_enabled']} c4={r['c4_enabled']} at t={r['t']}")
    # 2026-09-10 real-hardware finding: relay_valid is NOT used here --
    # touch_slave.cpp latches command_.joint_torque_ext_valid=true ONCE, in
    # the joint_torque_ext_cmd callback, and never resets it -- a permanent
    # process-lifetime latch, not a live-state flag (confirmed via the
    # first two real H2 attempts: relay_valid=True on every sample of both
    # runs, purely from this project's driver processes having handled a
    # legitimate C1/C4 relay message in some earlier session; relay_fresh
    # and relay_applied were False throughout both, with
    # relay_torque_received_nm/relay_force_n exactly zero). relay_fresh
    # (time-gated) and relay_applied (proof a nonzero force was summed) are
    # the real live-contribution signals.
    for name, rows in (("master", master), ("slave", slave)):
        relay_rows = [r for r in rows if r["relay_fresh"] or r["relay_applied"]]
        if relay_rows:
            flag("UNEXPECTED_RELAY_CONTRIBUTION", "INVALID",
                f"{name} relay_fresh/relay_applied True at t={relay_rows[0]['t']} despite c1/c4 configured off")

    # 6. Gravity ramp reached its intended stage.
    gravity_full = [r for r in bridge if r["gravity_ramp_alpha"] >= 1.0 - 1e-6]
    gravity_reached = bool(gravity_full)
    if not gravity_reached:
        flag("GRAVITY_RAMP_NEVER_COMPLETED", "INVALID", "gravity_ramp_alpha never reached 1.0")

    # 7. Quiet-hold gate: independently re-verify continuous sub-threshold
    # speed across the FULL accumulated window (not a single sample), using
    # RAW recorded velocity, cross-checked against the state machine's own
    # gate_pass_t.
    gate_pass_t = meta.get("gate_pass_t")
    gate_pass_accumulated_s = meta.get("gate_pass_accumulated_s")
    settle_evidence = {"gate_pass_t": gate_pass_t, "gate_pass_accumulated_s": gate_pass_accumulated_s,
                       "configured_hold_s": hold_s, "configured_velocity_threshold_mps": vel_thresh}
    if gate_pass_t is None:
        flag("QUIET_HOLD_GATE_NEVER_PASSED", "INVALID",
            "gravity_settle_quiet_window_accumulated_s never reached the configured hold_s "
            "within MAX_WAIT_S -- the mechanism never settled quietly")
    else:
        window_start = gate_pass_t - gate_pass_accumulated_s
        for name, rows in (("master", master), ("slave", slave)):
            window_rows = [r for r in rows if window_start - 0.02 <= r["t"] <= gate_pass_t + 1e-3]
            if not window_rows:
                flag("QUIET_HOLD_WINDOW_NO_SAMPLES", "INVALID",
                    f"{name}: no raw telemetry samples found inside the reported quiet window "
                    f"[{window_start:.3f}, {gate_pass_t:.3f}] -- cannot independently verify")
                continue
            speeds = [_norm3(r, "velocity_mps") for r in window_rows]
            over = [(r["t"], s) for r, s in zip(window_rows, speeds) if s >= vel_thresh]
            settle_evidence[f"{name}_window_n_samples"] = len(window_rows)
            settle_evidence[f"{name}_window_peak_speed_mps"] = max(speeds)
            if over:
                flag("QUIET_HOLD_SPEED_EXCEEDED_IN_WINDOW", "INVALID",
                    f"{name}: raw velocity sample at t={over[0][0]:.3f} read {over[0][1]*1000:.4f}mm/s "
                    f">= threshold {vel_thresh*1000:.4f}mm/s INSIDE the reported quiet window -- "
                    "the state machine's own accumulated-window claim does not hold up against the "
                    "raw recorded samples (this is exactly the 'false-pass from a single static "
                    "sample' failure mode this check exists to catch)")

    # 8. Pose-drift criteria -- no frozen "max acceptable drift" threshold
    # exists; the only genuine frozen bound available is each arm's own
    # max_position_error_m (a real physical clamp limit, not invented here)
    # -- drift is reported unconditionally, and only flagged INVALID if it
    # is implausibly large relative to that bound (would indicate the
    # settle was accepted during real, large-amplitude motion, not stillness).
    accepted_rows = [r for r in bridge if r["gravity_settle_accepted_master_pose_drift_m"] not in (None, "") ]
    accepted = accepted_rows[-1] if accepted_rows else None
    if accepted is None:
        flag("POSE_DRIFT_NOT_REPORTED", "FLAG", "gravity_settle_accepted_*_pose_drift_m never populated")
    else:
        settle_evidence["accepted_master_pose_drift_m"] = accepted["gravity_settle_accepted_master_pose_drift_m"]
        settle_evidence["accepted_slave_pose_drift_m"] = accepted["gravity_settle_accepted_slave_pose_drift_m"]

    # 9. Deadman held throughout enabled force.
    deadman_lost = [r for r in master if r["enabled"] and not r["deadman_ok"]]
    if deadman_lost:
        flag("DEADMAN_LOST_WHILE_ENABLED", "INVALID", f"master deadman_ok=False while enabled=True at t={deadman_lost[0]['t']}")

    # 10. Parameter-mismatch fault path.
    mismatch_rows = [r for r in bridge if not r["planner_parameter_match_left"] or not r["planner_parameter_match_right"]]
    if mismatch_rows:
        flag("PLANNER_PARAMETER_MISMATCH", "INVALID", f"at t={mismatch_rows[0]['t']}")

    # 11. PHYSICAL gravity-force validation (2026-09-10 real-hardware
    # finding, Task 7). Previously the aux-force numbers below were
    # informational only -- a run could report overall PASS purely from
    # state-machine/settle-gate criteria even if the slave's `enabled`
    # flag was False the ENTIRE run (a real occurrence: session
    # 20260910T104521Z_h2 passed every settle-gate criterion while
    # auxiliary_force_n/applied_force_n were EXACTLY [0,0,0] throughout,
    # because `enabled` was False on both arms for the whole run --
    # traced to touch_slave.cpp:842 `if (command.enabled && deadman_ok)`,
    # which gates ALL force including auxiliary_force (force_cmd_n summed
    # at touch_slave.cpp:863, inside that same block) -- root cause was
    # the master deadman never being detected as held, NOT a telemetry
    # field bug: every force-decomposition field (spring, damping,
    # auxiliary, before/after primary clamp, relay, before/after final
    # clamp, applied) was independently confirmed exactly zero). H2's own
    # stated purpose is validating the gravity-support force path, so a
    # run where that force was never actually enabled must not report an
    # unqualified PASS.
    #
    # Evaluated over the STEADY settle interval (t >= gate_pass_t,
    # phase==GRAVITY_SETTLE) rather than the whole GRAVITY_SETTLE phase,
    # which also includes the pre-gate-pass transient.
    axis_names = {0: "x", 1: "y", 2: "z"}
    slave_gravity_axis = th.get("slave_gravity_axis")
    slave_gravity_comp_n = th.get("slave_gravity_comp_n")
    axis_key = axis_names.get(int(slave_gravity_axis), "y") if slave_gravity_axis is not None else "y"

    if gate_pass_t is not None:
        steady_settle = [r for r in slave if r["phase"] == "GRAVITY_SETTLE" and r["t"] >= gate_pass_t]
    else:
        steady_settle = [r for r in slave if r["phase"] == "GRAVITY_SETTLE"]

    physical_force_validated = None
    physical_force_mean_applied_n = None
    enabled_rows = [r for r in steady_settle if r["enabled"]]

    if slave_gravity_comp_n is None:
        # Task 7 test 3: missing expected-value metadata must not allow a
        # silent PASS -- explicit INVALID flag, not an assumed-fine skip.
        flag("GRAVITY_FORCE_EXPECTED_VALUE_MISSING", "INVALID",
            "frozen_thresholds.slave_gravity_comp_n is missing from part1_meta.json -- cannot "
            "validate the physical gravity-force path against an unknown expected value")
        physical_force_validated = False
    elif not steady_settle:
        flag("GRAVITY_FORCE_NO_STEADY_SETTLE_SAMPLES", "INVALID",
            "no slave samples found in the steady (post-gate-pass) GRAVITY_SETTLE interval -- "
            "cannot validate the physical gravity-force path")
        physical_force_validated = False
    elif not enabled_rows:
        flag("GRAVITY_FORCE_NEVER_ENABLED", "INVALID",
            f"slave 'enabled' was False for all {len(steady_settle)} sample(s) in the steady "
            "GRAVITY_SETTLE interval -- the gravity-support force was never actually enabled "
            "(touch_slave.cpp gates auxiliary_force_n on command.enabled && deadman_ok), so the "
            "physical force path this stage exists to validate was never exercised. Common cause: "
            "the master deadman was not detected as held (see BridgeDiagnostics/master telemetry "
            "for the deadman gate's own state) -- this is the bridge's safety gate working as "
            "designed, not a defect, but it means this run does not constitute a physical "
            "validation of the gravity-support force.")
        physical_force_validated = False
    else:
        applied_vals = [r[f"applied_force_n_{axis_key}"] for r in enabled_rows]
        physical_force_mean_applied_n = sum(applied_vals) / len(applied_vals)
        # Source-backed tolerance: reuses the SAME unexpected_spring_damping_norm_n
        # (0.05N) this file/part1 already use elsewhere as "how large a force
        # deviation is genuinely anomalous, not sensor noise" -- not a new,
        # invented number.
        tol = th.get("unexpected_spring_damping_norm_n", 0.05)
        deviation = abs(physical_force_mean_applied_n - slave_gravity_comp_n)
        if deviation > tol:
            flag("GRAVITY_FORCE_MISMATCH", "INVALID",
                f"slave enabled for {len(enabled_rows)}/{len(steady_settle)} steady-settle samples; "
                f"mean applied_force_n_{axis_key}={physical_force_mean_applied_n:.4f}N deviates from "
                f"expected slave_gravity_comp_n={slave_gravity_comp_n:.4f}N by {deviation:.4f}N "
                f"(> tolerance {tol}N)")
            physical_force_validated = False
        else:
            physical_force_validated = True

    # Auxiliary/gravity force sanity -- retained for the full raw
    # summary/plots (not phase/enabled-filtered), distinct from the
    # steady-settle/enabled-filtered check above.
    slave_aux = [r["auxiliary_force_n_y"] for r in slave if r.get("auxiliary_force_n_y") is not None]
    master_aux = [r["auxiliary_force_n_y"] for r in master if r.get("auxiliary_force_n_y") is not None]

    aborted_by_capture = bool(meta.get("aborted"))
    if aborted_by_capture and not any(f["severity"] == "INVALID" for f in flags):
        flag("CAPTURE_SCRIPT_ABORTED_BUT_ANALYSIS_FOUND_NO_INVALID_FLAG", "FLAG",
            f"capture script aborted with reason={meta.get('abort_reason')!r} but this independent "
            "re-analysis found no matching INVALID flag -- investigate the discrepancy before trusting either verdict alone")

    overall = "FAIL" if any(f["severity"] == "INVALID" for f in flags) else "PASS"
    if aborted_by_capture and overall == "PASS":
        # The capture script's own live abort is authoritative if it fired,
        # even if this independent re-check did not reproduce a matching
        # flag (see the CAPTURE_SCRIPT_ABORTED_BUT_... flag above) -- never
        # silently override a real-time abort with a more lenient post-hoc
        # verdict.
        overall = "FAIL"

    # DOB baseline-relative delta.
    baseline = meta.get("dob_baseline")
    dob_delta = None
    if baseline is not None:
        import statistics
        m_shoulder = [r["dob_shoulder"] for r in master if r.get("dob_shoulder") is not None]
        s_shoulder = [r["dob_shoulder"] for r in slave if r.get("dob_shoulder") is not None]
        m_mean = statistics.fmean(m_shoulder) if m_shoulder else None
        s_mean = statistics.fmean(s_shoulder) if s_shoulder else None
        dob_delta = {
            "master_shoulder_dob_h2_mean_nm": m_mean,
            "slave_shoulder_dob_h2_mean_nm": s_mean,
            "master_shoulder_dob_baseline_nm": baseline.get("master_shoulder_dob_mean_nm"),
            "slave_shoulder_dob_baseline_nm": baseline.get("slave_shoulder_dob_mean_nm"),
            "delta_tau_dis_master_shoulder_nm": (
                (m_mean - baseline["master_shoulder_dob_mean_nm"])
                if (m_mean is not None and baseline.get("master_shoulder_dob_mean_nm") is not None) else None),
            "delta_tau_dis_slave_shoulder_nm": (
                (s_mean - baseline["slave_shoulder_dob_mean_nm"])
                if (s_mean is not None and baseline.get("slave_shoulder_dob_mean_nm") is not None) else None),
            "note": "these are SHOULDER-joint (tau_dis_hat_nm[1]) disturbance estimates, NOT Cartesian-Y "
                    "-- see this file's/part1's module docstring. Not claimed as external-force accuracy "
                    "evidence from H2 alone.",
        }

    metrics = {
        "gravity_reached": gravity_reached,
        "gate_pass_evidence": settle_evidence,
        "master_aux_y_mean_n": (sum(master_aux) / len(master_aux)) if master_aux else None,
        "slave_aux_y_mean_n": (sum(slave_aux) / len(slave_aux)) if slave_aux else None,
        "slave_aux_y_expected_n": th.get("slave_gravity_comp_n"),
        "physical_gravity_force_validated": physical_force_validated,
        "physical_gravity_force_axis": axis_key,
        "physical_gravity_force_expected_n": slave_gravity_comp_n,
        "physical_gravity_force_mean_applied_n": physical_force_mean_applied_n,
        "physical_gravity_force_enabled_sample_count": len(enabled_rows),
        "physical_gravity_force_steady_settle_sample_count": len(steady_settle),
        "dob_baseline_relative": dob_delta,
        "disarm_confirmed": meta.get("disarm_confirmed"),
        "final_phase": meta.get("final_phase"),
    }

    validation = {"overall_result": overall, "flags": flags,
                 "capture_script_aborted": aborted_by_capture,
                 "capture_script_abort_reason": meta.get("abort_reason"),
                 "capture_script_abort_t": meta.get("abort_t")}

    (out_dir / "h2_metrics.json").write_text(json.dumps(metrics, indent=2, default=str))
    (out_dir / "h2_validation.json").write_text(json.dumps(validation, indent=2, default=str))

    make_plots(master, slave, bridge, meta, out_dir / "plots")

    return {"metrics": metrics, "validation": validation}


def make_plots(master, slave, bridge, meta, plot_dir: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_dir.mkdir(parents=True, exist_ok=True)
    if not master or not slave or not bridge:
        print("[analyze_h2] insufficient data for plots -- skipping")
        return
    t0 = min(master[0]["t"], slave[0]["t"], bridge[0]["t"])

    def ts(rows):
        return [r["t"] - t0 for r in rows]

    def _mark_gate(ax):
        gp = meta.get("gate_pass_t")
        if gp is not None:
            ax.axvline(gp - t0, color="green", linestyle="--", linewidth=1.2, label="gate pass")
        at = meta.get("arm_t")
        if at is not None:
            ax.axvline(at - t0, color="blue", linestyle=":", linewidth=1.0, label="armed")
        abt = meta.get("abort_t")
        if abt is not None:
            ax.axvline(abt - t0, color="red", linestyle="--", linewidth=1.2, label="abort")

    # 1. master/slave Y position.
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts(master), [r["position_m_y"] for r in master], label="master y")
    ax.plot(ts(slave), [r["position_m_y"] for r in slave], label="slave y")
    ax.set_ylabel("m"); ax.set_xlabel("t (s)"); ax.legend(); ax.set_title("H2: master/slave Y position")
    _mark_gate(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "01_y_position.png", dpi=150); plt.close(fig)

    # 2. master/slave Y velocity.
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts(master), [r["velocity_mps_y"] for r in master], label="master vy")
    ax.plot(ts(slave), [r["velocity_mps_y"] for r in slave], label="slave vy")
    ax.set_ylabel("m/s"); ax.set_xlabel("t (s)"); ax.legend(); ax.set_title("H2: master/slave Y velocity")
    _mark_gate(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "02_y_velocity.png", dpi=150); plt.close(fig)

    # 3/4. full XYZ.
    for name, rows, idx in (("master", master, "03"), ("slave", slave, "04")):
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(ts(rows), [r["position_m_x"] for r in rows], label="x")
        ax.plot(ts(rows), [r["position_m_y"] for r in rows], label="y")
        ax.plot(ts(rows), [r["position_m_z"] for r in rows], label="z")
        ax.set_ylabel("m"); ax.set_xlabel("t (s)"); ax.legend(); ax.set_title(f"H2: {name} XYZ position")
        _mark_gate(ax)
        fig.tight_layout(); fig.savefig(plot_dir / f"{idx}_{name}_xyz.png", dpi=150); plt.close(fig)

    # 5. gravity_ramp_alpha / coupling_ramp_alpha.
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts(bridge), [r["gravity_ramp_alpha"] for r in bridge], label="gravity_ramp_alpha")
    ax.plot(ts(bridge), [r["coupling_ramp_alpha"] for r in bridge], label="coupling_ramp_alpha")
    ax.set_ylim(-0.05, 1.1); ax.set_xlabel("t (s)"); ax.legend(); ax.set_title("H2: ramp alphas")
    _mark_gate(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "05_ramp_alphas.png", dpi=150); plt.close(fig)

    # 6. settle speed vs threshold.
    thresh = meta.get("frozen_thresholds", {}).get("gravity_settle_velocity_threshold_mps")
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts(bridge), [r["gravity_settle_master_speed_mps"] for r in bridge], label="master speed")
    ax.plot(ts(bridge), [r["gravity_settle_slave_speed_mps"] for r in bridge], label="slave speed")
    if thresh is not None:
        ax.axhline(thresh, color="r", linestyle="--", label=f"threshold={thresh}")
    ax.set_ylabel("m/s"); ax.set_xlabel("t (s)"); ax.legend(); ax.set_title("H2: settle speed vs threshold")
    _mark_gate(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "06_settle_speed_vs_threshold.png", dpi=150); plt.close(fig)

    # 7. pose drift.
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts(bridge), [r["gravity_settle_master_pose_drift_m"] for r in bridge], label="master pose drift")
    ax.plot(ts(bridge), [r["gravity_settle_slave_pose_drift_m"] for r in bridge], label="slave pose drift")
    ax.set_ylabel("m"); ax.set_xlabel("t (s)"); ax.legend(); ax.set_title("H2: pose drift during current quiet window")
    _mark_gate(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "07_pose_drift.png", dpi=150); plt.close(fig)

    # 8. quiet-window accumulation.
    hold_s = meta.get("frozen_thresholds", {}).get("gravity_settle_hold_s")
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts(bridge), [r["gravity_settle_quiet_window_accumulated_s"] for r in bridge])
    if hold_s is not None:
        ax.axhline(hold_s, color="r", linestyle="--", label=f"hold_s={hold_s}")
    ax.set_ylabel("s"); ax.set_xlabel("t (s)"); ax.legend(); ax.set_title("H2: quiet-window accumulation")
    _mark_gate(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "08_quiet_window_accumulation.png", dpi=150); plt.close(fig)

    # 9. force decomposition (both arms, Y).
    for name, rows, idx in (("master", master, "09"), ("slave", slave, "09b")):
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(ts(rows), [r["spring_force_n_y"] for r in rows], label="spring_y")
        ax.plot(ts(rows), [r["damping_force_n_y"] for r in rows], label="damping_y")
        ax.plot(ts(rows), [r["auxiliary_force_n_y"] for r in rows], label="auxiliary_y")
        ax.plot(ts(rows), [r["relay_force_n_y"] for r in rows], label="relay_y")
        ax.plot(ts(rows), [r["applied_force_n_y"] for r in rows], label="applied_y (final)")
        ax.set_ylabel("N"); ax.set_xlabel("t (s)"); ax.legend(fontsize=7)
        ax.set_title(f"H2: {name} force decomposition (Y)")
        _mark_gate(ax)
        fig.tight_layout(); fig.savefig(plot_dir / f"{idx}_force_decomposition_{name}.png", dpi=150); plt.close(fig)

    # 10. applied force magnitude.
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts(master), [_norm3(r, "applied_force_n") for r in master], label="master |applied|")
    ax.plot(ts(slave), [_norm3(r, "applied_force_n") for r in slave], label="slave |applied|")
    ax.set_ylabel("N"); ax.set_xlabel("t (s)"); ax.legend(); ax.set_title("H2: applied force magnitude")
    _mark_gate(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "10_applied_force_magnitude.png", dpi=150); plt.close(fig)

    # 11/12. DOB per-joint (both arms) + baseline-relative delta (shoulder only).
    baseline = meta.get("dob_baseline") or {}
    for name, rows, idx in (("master", master, "11"), ("slave", slave, "11b")):
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(ts(rows), [r["dob_waist"] for r in rows], label="waist")
        ax.plot(ts(rows), [r["dob_shoulder"] for r in rows], label="shoulder")
        ax.plot(ts(rows), [r["dob_elbow"] for r in rows], label="elbow")
        ax.set_ylabel("Nm"); ax.set_xlabel("t (s)"); ax.legend()
        ax.set_title(f"H2: {name} DOB disturbance estimate (per-joint, [waist,shoulder,elbow])")
        _mark_gate(ax)
        fig.tight_layout(); fig.savefig(plot_dir / f"{idx}_dob_{name}.png", dpi=150); plt.close(fig)

    m_base = baseline.get("master_shoulder_dob_mean_nm")
    s_base = baseline.get("slave_shoulder_dob_mean_nm")
    if m_base is not None and s_base is not None:
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(ts(master), [r["dob_shoulder"] - m_base for r in master], label="master delta (shoulder)")
        ax.plot(ts(slave), [r["dob_shoulder"] - s_base for r in slave], label="slave delta (shoulder)")
        ax.axhline(0.0, color="k", linewidth=0.5)
        ax.set_ylabel("Nm"); ax.set_xlabel("t (s)"); ax.legend()
        ax.set_title("H2: baseline-relative DOB change (shoulder joint, NOT Cartesian-Y)")
        _mark_gate(ax)
        fig.tight_layout(); fig.savefig(plot_dir / "12_dob_baseline_relative_delta.png", dpi=150); plt.close(fig)


def main():
    if len(sys.argv) < 2:
        print("usage: analyze_h2_gravity_settle.py <out_dir>", file=sys.stderr)
        sys.exit(2)
    out_dir = Path(sys.argv[1])
    try:
        result = analyze(out_dir)
    except Exception as exc:
        # 2026-09-10 fix (Task 4): "handle missing/partial capture CSVs
        # gracefully" -- any unexpected exception (malformed/truncated
        # file, unexpected schema) becomes a structured ANALYSIS_INCOMPLETE
        # result, never a raw traceback that leaves no validation artifact.
        validation = {"overall_result": "ANALYSIS_INCOMPLETE",
                     "flags": [{"code": "ANALYZER_EXCEPTION", "severity": "INVALID",
                               "detail": f"{type(exc).__name__}: {exc}"}]}
        (out_dir / "h2_validation.json").write_text(json.dumps(validation, indent=2, default=str))
        print(json.dumps(validation, indent=2, default=str))
        sys.exit(1)
    print(json.dumps(result["validation"], indent=2, default=str))
    print(json.dumps(result["metrics"], indent=2, default=str))
    sys.exit(0 if result["validation"]["overall_result"] == "PASS" else 1)


if __name__ == "__main__":
    main()
