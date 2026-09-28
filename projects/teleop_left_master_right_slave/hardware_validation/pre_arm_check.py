"""H2 PRE-ARM verification (2026-09-10 safety-hardening change).

Structurally separate from verify_disarmed.py's POST-disarm check, per the
explicit distinction this task requires:

  PRE-ARM  (this module)  -- proves any stale enabled=True latch (found by
            a read-only live check before this task) was actually CLEARED
            by the bridge's own unconditional enable=False publish, and
            that every other pre-arm invariant holds, BEFORE arm_teleop is
            ever sent. Does not rely on the internal device state being
            trustworthy on its own -- it is independently re-verified from
            live samples collected AFTER the bridge started publishing.
  POST-DISARM (verify_disarmed.py) -- proves the system returned to a safe
            state AFTER a run, from a fresh subscription, not the last
            recorded CSV row.

Two functions:
  - check_enable_publisher_ownership(node, expected_node_name) -- rclpy
    graph introspection; needs a live Node, not importable/testable
    without rclpy.
  - evaluate_pre_arm_samples(...) -- pure, takes already-collected sample
    rows (same schema part1_gravity_settle_capture.py's GravitySettleCapture
    already accumulates), fully unit-testable without ROS.
"""
from __future__ import annotations

import math
import time

VALID_PRE_ARM_BRIDGE_STATES = {
    "WAIT_FOR_POSES", "HOLD_STILL", "CAPTURE_ANCHORS", "VALIDATE_ALIGNMENT", "READY",
}

EXPECTED_BRIDGE_NODE_NAME = "touch_master_slave_bridge"  # bridge_master_slave.py's own
                                                          # super().__init__(...) argument --
                                                          # see preflight.py's 2026-09-10 bug-fix
                                                          # note for why this must NOT be the
                                                          # differently-ordered "bridge_master_slave"


def _norm3(row: dict, prefix: str) -> float:
    return math.sqrt(row[f"{prefix}_x"] ** 2 + row[f"{prefix}_y"] ** 2 + row[f"{prefix}_z"] ** 2)


ENABLE_AND_DIAG_TOPICS = ("/touch_left/enable", "/touch_right/enable", "/bridge_diagnostics")


def check_enable_publisher_ownership(node, expected_node_name: str = EXPECTED_BRIDGE_NODE_NAME) -> dict:
    """Confirms EXACTLY ONE publisher ENDPOINT on each enable topic (and on
    /bridge_diagnostics), and that it is the H2 bridge.

    2026-09-10 real-hardware finding, bug fix: the previous version
    collapsed publisher identity to `{i.node_name for i in infos}` -- a
    SET of node-name strings. Two DISTINCT publisher endpoints (e.g. two
    separate `touch_master_slave_bridge` process instances, one from a
    still-shutting-down previous H2 run racing a newly-launched one) that
    happen to share the same node name collapse into ONE set entry, so
    `len(pubs) == 1` was true even with two live publishers -- this check
    could not detect the exact double-bridge scenario it exists to catch.
    Fixed to check `len(infos)` (actual endpoint count, from
    TopicEndpointInfo, one entry per DISTINCT DDS publisher handle) and to
    record each endpoint's `endpoint_gid` so two same-named duplicates are
    visibly distinguishable in the pre-arm artifact, not merged away."""
    result = {"expected_node_name": expected_node_name}
    ok = True
    for topic in ENABLE_AND_DIAG_TOPICS:
        try:
            infos = node.get_publishers_info_by_topic(topic)
        except Exception as exc:
            result[topic] = {"error": str(exc)}
            ok = False
            continue
        endpoints = [{"node_name": i.node_name, "node_namespace": i.node_namespace,
                     "endpoint_gid": list(i.endpoint_gid)} for i in infos]
        result[topic] = endpoints
        if len(endpoints) != 1:
            ok = False
        elif endpoints[0]["node_name"] != expected_node_name:
            ok = False
    result["ok"] = ok
    return result


def check_single_bridge_node_instance(node, expected_node_name: str = EXPECTED_BRIDGE_NODE_NAME) -> dict:
    """Hard invariant: exactly ONE live node instance named
    `expected_node_name` in the ROS graph -- distinct from the publisher-
    endpoint check above (a node could theoretically exist without
    currently publishing anything, though the bridge always does once
    started). ROS 2 does not enforce unique node names at the DDS layer,
    so two independently-launched processes using the same name are a
    real, not merely hypothetical, failure mode -- confirmed as the
    suspected mechanism behind a real-hardware READY<->GRAVITY_RAMP_DOWN
    oscillation observed 2026-09-10 (a previous H2 run's bridge process
    still shutting down while a new one, launched only ~3s later, was
    already starting up -- see run_h2.sh's Task 3 single-instance lock,
    which is the actual fix; this check is the pre-arm-time verification
    of its precondition)."""
    names_and_ns = node.get_node_names_and_namespaces()
    matches = [(n, ns) for n, ns in names_and_ns if n == expected_node_name]
    return {"expected_node_name": expected_node_name, "matches": matches,
           "count": len(matches), "ok": len(matches) == 1}


DEFAULT_EXPECTED_CHANNELS = {"c1": False, "c2": False, "c3": False, "c4": False}


def evaluate_pre_arm_samples(rows_left: list, rows_right: list, rows_bridge: list,
                             force_tolerance_n: float, telemetry_stale_s: float,
                             min_samples: int = 5, window_start_t: float = None,
                             decision_time_t: float = None, expected_channels: dict = None,
                             require_enabled_false: bool = True) -> dict:
    """Pure, no ROS -- independently re-derives every pre-arm criterion
    from already-collected sample rows. `rows_left`/`rows_right` use the
    same schema as GravitySettleCapture.rows["left"/"right"]
    (part1_gravity_settle_capture.py); `rows_bridge` matches
    GravitySettleCapture.rows["bridge"].

    2026-09-10 real-hardware finding, bug fix: this function used to
    evaluate whatever rows it was handed with no notion of a bounded,
    recent window -- if the caller (part1_gravity_settle_capture.py)
    accumulated rows from node-creation time onward (including, e.g., the
    bridge-startup wait, BEFORE the bridge even reported ready), an old
    startup-phase gap would poison the decision even though telemetry was
    genuinely fresh and continuous during the actual pre-arm window. The
    real safety requirement is "prove BOTH streams are currently fresh and
    have remained continuously fresh throughout a bounded RECENT
    observation window" -- not "fail forever if any historical gap ever
    occurred since the node was created."

    This function does not itself decide what counts as "the window" --
    the caller is responsible for only handing it rows collected inside
    the intended window (see part1_gravity_settle_capture.py's explicit
    `node.rows[...].clear()` immediately after the bridge reports ready,
    before starting a fresh PRE_ARM_WINDOW_S collection). `window_start_t`
    is an ADDITIONAL, independent proof check: if given, any row with
    `t < window_start_t` is treated as evidence the caller's window reset
    did not actually happen (defense in depth, not the primary mechanism).

    `decision_time_t` (defaults to `time.time()` if omitted) is used for
    the LAST-SAMPLE-AGE check -- a genuinely distinct quantity from
    max-inter-sample-gap: a window can have uniformly small inter-sample
    gaps throughout yet still have gone stale if nothing arrived between
    the window's last sample and the moment the decision is made.

    `expected_channels` (2026-09-10, production-arming reuse): a dict
    {"c1": bool, "c2": bool, "c3": bool, "c4": bool} describing what the
    CONFIGURED channel state should be before arming -- defaults to all-
    False, H2's own requirement (gravity-only, no bilateral action of any
    kind). Production C3-only teleoperation intentionally arms with
    c3=True, so this must be a real parameter, not a hardcoded "all must
    be False" -- the ORIGINAL version of this check hardcoded exactly
    that assumption, which would have wrongly rejected the (correct)
    production pre-arm state where c3_enabled=True is the whole point.

    `require_enabled_false` (2026-09-10, production-arming reuse,
    confirmed via live-sim testing): whether `enabled=True` on either arm
    BEFORE arming is itself evidence of a problem. This is only true when
    a deadman gate exists (H2's require_master_deadman=True): with no
    deadman (production's normal configuration, matching
    deadman_gate_state()'s `if not require: return True, ...`),
    bridge_master_slave.py publishes `enable=True` UNCONDITIONALLY from
    the moment it starts, regardless of arm state -- confirmed on the
    REAL production run this task investigates (`/touch_right/enable ==
    true` while `arm_teleop==false` and `bridge_state==READY`) and
    reproduced live in sim. Treating that as a "stale latch" would make
    production pre-arm verification impossible to ever pass. Defaults to
    True (H2's existing, unchanged behavior); production callers must
    pass False explicitly."""
    if decision_time_t is None:
        decision_time_t = time.time()
    if expected_channels is None:
        expected_channels = DEFAULT_EXPECTED_CHANNELS
    reasons = []
    result = {"min_samples_required": min_samples, "force_tolerance_n": force_tolerance_n,
             "telemetry_stale_s": telemetry_stale_s, "window_start_t": window_start_t,
             "decision_time_t": decision_time_t,
             "pre_arm_window_duration_s": (decision_time_t - window_start_t) if window_start_t is not None else None}

    for side, rows in (("left", rows_left), ("right", rows_right)):
        n = len(rows)
        result[f"{side}_n_samples"] = n
        if window_start_t is not None:
            stale_predating = [r for r in rows if r["t"] < window_start_t]
            if stale_predating:
                reasons.append(f"{side}: {len(stale_predating)} sample(s) predate the pre-arm window start "
                               f"t={window_start_t:.3f} -- the window was not properly reset before evaluation")
        if n < min_samples:
            reasons.append(f"{side}: only {n} sample(s) collected (need >= {min_samples})")
            result[f"{side}_max_gap_s"] = None
            result[f"{side}_final_sample_age_s"] = None
            continue
        for r in rows:
            if require_enabled_false and r["enabled"]:
                reasons.append(f"{side}: enabled=True at t={r['t']:.3f} "
                               "(stale latch not cleared, or a real unexpected publish)")
            fn = _norm3(r, "applied_force_n")
            if fn >= force_tolerance_n:
                reasons.append(f"{side}: applied_force_norm={fn:.4f}N >= tolerance "
                               f"{force_tolerance_n}N at t={r['t']:.3f}")
            if r["position_error_clamped"] or r["primary_force_clamped"] or r["final_force_clamped"]:
                reasons.append(f"{side}: clamp flag set at t={r['t']:.3f}")
        gaps = [rows[i]["t"] - rows[i - 1]["t"] for i in range(1, len(rows))]
        max_gap = max(gaps) if gaps else None
        result[f"{side}_max_gap_s"] = max_gap
        if max_gap is not None and max_gap > telemetry_stale_s:
            reasons.append(f"{side}: telemetry gap {max_gap:.3f}s > {telemetry_stale_s}s -- not fresh "
                           "(inside the evaluated window)")
        final_age = decision_time_t - rows[-1]["t"]
        result[f"{side}_final_sample_age_s"] = final_age
        if final_age > telemetry_stale_s:
            reasons.append(f"{side}: final sample age {final_age:.3f}s > {telemetry_stale_s}s at decision time "
                           "-- not fresh RIGHT NOW, independent of historical inter-sample gaps")

    n_bridge = len(rows_bridge)
    result["bridge_n_samples"] = n_bridge
    result["bridge_max_gap_s"] = None
    result["bridge_final_sample_age_s"] = None
    if window_start_t is not None:
        stale_predating = [r for r in rows_bridge if r["t"] < window_start_t]
        if stale_predating:
            reasons.append(f"bridge: {len(stale_predating)} sample(s) predate the pre-arm window start "
                           f"t={window_start_t:.3f} -- the window was not properly reset before evaluation")
    if n_bridge < min_samples:
        reasons.append(f"bridge: only {n_bridge} diagnostics sample(s) collected (need >= {min_samples})")
    else:
        for r in rows_bridge:
            if r["bridge_state"] == "FAULT":
                reasons.append(f"bridge FAULT observed at t={r['t']:.3f}")
            elif r["bridge_state"] not in VALID_PRE_ARM_BRIDGE_STATES:
                reasons.append(f"bridge_state={r['bridge_state']!r} is not a valid pre-arm state "
                               f"at t={r['t']:.3f} (expected one of {sorted(VALID_PRE_ARM_BRIDGE_STATES)})")
            if abs(r["coupling_ramp_alpha"]) > 1e-9:
                reasons.append(f"coupling_ramp_alpha={r['coupling_ramp_alpha']} != 0 at t={r['t']:.3f}")
            actual_channels = {"c1": bool(r["c1_enabled"]), "c2": bool(r["c2_enabled"]),
                              "c3": bool(r["c3_enabled"]), "c4": bool(r["c4_enabled"])}
            if actual_channels != expected_channels:
                reasons.append(f"a channel is configured differently than expected before arm: "
                               f"actual={actual_channels} expected={expected_channels} at t={r['t']:.3f}")
        bgaps = [rows_bridge[i]["t"] - rows_bridge[i - 1]["t"] for i in range(1, n_bridge)]
        bmax_gap = max(bgaps) if bgaps else None
        result["bridge_max_gap_s"] = bmax_gap
        if bmax_gap is not None and bmax_gap > telemetry_stale_s:
            reasons.append(f"bridge: diagnostics gap {bmax_gap:.3f}s > {telemetry_stale_s}s -- not fresh "
                           "(inside the evaluated window)")
        bfinal_age = decision_time_t - rows_bridge[-1]["t"]
        result["bridge_final_sample_age_s"] = bfinal_age
        if bfinal_age > telemetry_stale_s:
            reasons.append(f"bridge: final sample age {bfinal_age:.3f}s > {telemetry_stale_s}s at decision time")

    result["reasons"] = reasons
    result["status"] = "PRE_ARM_FAIL" if reasons else "PRE_ARM_PASS"
    return result
