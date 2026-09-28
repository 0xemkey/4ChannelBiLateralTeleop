# H2 orchestration investigation (2026-09-10)

Three real-hardware H2 attempts across this investigation
(`20260910T100244Z_h2`, `20260910T100247Z_h2`, `20260910T102922Z_h2`)
exposed THREE independent H2 TOOLING bugs (not controller bugs, not real
hardware faults). All are root-caused with direct measurement/evidence
below, fixed, and covered by new regression tests. No controller gains,
gravity compensation, settle thresholds, DOB parameters, or channel logic
were changed at any point in this investigation.

## Anomaly 1: "right telemetry stale (0.838s)" immediately after arming

**Root cause: capture-tool callback starvation, not real telemetry loss.**

`part1_gravity_settle_capture.py` sent the arm request via
`subprocess.run(["ros2", "param", "set", "/touch_master_slave_bridge",
"arm_teleop", "true"])` — a blocking call that spawns a brand-new `ros2`
CLI process (its own rclpy init, DDS discovery, service call, process
exit). While that subprocess ran, the CALLING process's own
`rclpy.spin_once()` was never invoked, so every subscription's `self.latest[...]`
timestamp froze for the full duration. The very next `check_abort()` call
then measured the frozen timestamp's age as "stale."

**Measured directly** (sim harness, isolated domain, real bridge process):
`subprocess.run(["ros2","param","set",...])` took **1.3262s** wall time;
during that window, the gap since the last left/right/diag callback was
1.3263s/1.3294s/1.3272s (i.e. exactly the blocking duration). The very
next `spin_once()` after the subprocess returned received a fresh message
with 17µs age — proving the sim device's publisher never stopped
publishing; only the capture process's own draining stopped.

**Fix:** `GravitySettleCapture.set_arm_teleop()` now uses
`rcl_interfaces/srv/SetParameters` as an **async** service client on the
node itself, spinning that same node while awaiting the future — every
other subscription keeps being serviced throughout. Measured after the
fix: the same call took **0.0008s**, with a max observed callback gap of
9.5ms during the wait (normal control-loop cadence, not starvation). Also
added a bounded **post-arm freshness gate**: before declaring the armed
monitoring phase established, the script now requires fresh left/right/
diagnostics samples (not assumed).

**The `0.2s` telemetry-stale threshold was not touched.** Genuine
telemetry loss (SIGSTOP'd sim publisher, real OS process pause) still
correctly aborts — proven by a dedicated regression test.

## Anomaly 2: READY <-> GRAVITY_RAMP_DOWN oscillation before PRE_ARM

**Root cause: a publisher-ownership/node-identity check bug that could not
detect two same-named bridge instances, combined with no protection
against back-to-back H2 launches.**

The two real H2 runs were started only ~3 seconds apart. `run_h2.sh` had
no single-instance lock, and `part1_gravity_settle_capture.py` had no
check for a still-alive previous bridge process before launching its own.
If a previous run's `touch_master_slave_bridge` process was still mid-
shutdown when a new one started, BOTH would be alive and publishing
`/bridge_diagnostics` simultaneously, interleaving each other's states
(the old one's shutdown sequence and the new one's startup sequence) --
exactly the observed READY<->GRAVITY_RAMP_DOWN alternation.

Independently, `pre_arm_check.check_enable_publisher_ownership()` had a
real bug that would have masked this even if caught in time:

```python
pubs = sorted({i.node_name for i in infos})   # WRONG -- a SET of names
```

Two distinct publisher endpoints (two separate process instances) that
happen to share a node name collapse into ONE set entry, so
`len(pubs) == 1` could read True with two live publishers. Confirmed via
a dedicated live-sim test: two independent rclpy nodes both named
`touch_master_slave_bridge`, each publishing to the same enable topic,
produce 2 distinct `TopicEndpointInfo` entries (2 distinct
`endpoint_gid`s) that the old check would have silently merged.

**Fixes:**
1. `check_enable_publisher_ownership()` now checks actual endpoint COUNT
   (`len(infos)`), not unique node-name count, and records each endpoint's
   `endpoint_gid`. Extended to also check `/bridge_diagnostics`.
2. New `check_single_bridge_node_instance()`: hard invariant, exactly one
   live node named `touch_master_slave_bridge` in the graph.
3. `part1_gravity_settle_capture.py` now runs this check **before
   launching its own bridge subprocess at all** — if any instance already
   exists, it refuses to launch (`PRE_LAUNCH_FAILED`, no bridge ever
   spawned this run).
4. `run_h2.sh` now holds a `flock`-based single-instance lock for its
   entire run — a second concurrent invocation fails immediately
   (`FAIL CLOSED`) rather than interleaving with an in-progress one.
5. Teardown (`_verified_teardown()`) now polls the ROS graph until the
   bridge node AND its publisher endpoints on all three watched topics
   have actually disappeared (not just that the OS process exited — DDS
   discovery has its own latency) before this script returns control,
   within a bounded 8s timeout; if it doesn't clear in time, reports
   `SAFETY_INCOMPLETE` rather than silently claiming clean.

Master/slave DRIVER processes (the persistent, long-running real-hardware
nodes) are never touched by any of this — only H2's own
`touch_master_slave_bridge` process is subject to the lock/check/verify.

## Separately found and fixed: `relay_valid` false-abort

(Root-caused earlier in this session, included here for completeness of
the H2 orchestration picture.) Both real H2 arm attempts immediately
aborted with `"relay_valid=True relay_applied=False"`. `touch_slave.cpp`
latches `command_.joint_torque_ext_valid = true` **once**, in the
`joint_torque_ext_cmd` callback, and never resets it — a permanent,
process-lifetime latch (confirmed by direct source read:
`touch_slave.cpp:750`), not a live-state flag. Fixed by using
`relay_fresh`/`relay_applied` (the genuinely time-gated/proof-of-summing
fields) instead of `relay_valid` in both the capture script's abort check
and the analyzer's post-hoc validation.

## Anomaly 3: PRE_ARM_FAIL on ~0.96s left/right telemetry gaps
(`20260910T102922Z_h2`)

**Root cause: a NEW regression introduced by fixing Anomaly 2 (moving
`node` creation before the bridge subprocess launch), combined with
`evaluate_pre_arm_samples()` having no notion of a bounded recent window.**

The bridge-ready wait loop —

```python
while time.time() < deadline:
    line = bridge_proc.stdout.readline()   # BLOCKING, text mode
    ...
```

— had zero `rclpy.spin_once()` calls. Before Anomaly 2's fix, `node`
didn't exist yet at this point in the script, so this was harmless. After
Anomaly 2's fix moved node creation earlier (so the pre-launch single-
instance check could run before the bridge is spawned), this SAME node's
live telemetry subscriptions were now active during this same blocking
wait — starving them for the entire bridge-startup duration.

**Measured directly** (sim harness, isolated domain): bridge startup took
**1.566s** in one run; during that exact window, master telemetry showed
one single frozen gap of **1.566s** (last sample at t-launch=-0.004s, next
at t-launch=+1.566s) — the entire wait, one gap, matching the real
hardware's 0.96s finding in kind and magnitude (real bridge startup is
evidently a bit faster on the actual rig, ~0.96-0.99s vs sim's slower
~1.5s, but the mechanism is identical).

Independently, `evaluate_pre_arm_samples()` was being handed
`node.rows["left"/"right"/"bridge"]` — lists that accumulate from node-
creation time onward with no reset — so even a harmless historical gap
(from this bug, or in principle from anything else that happens before the
intended pre-arm window) would poison a decision that should only be about
"is telemetry fresh right now, in a bounded recent window."

**Two fixes, both required:**

1. **Non-blocking bridge-ready wait.** `bridge_proc.stdout` is now read in
   non-blocking raw-byte mode (`os.set_blocking(fd, False)`), manually
   buffered and split on newlines, with `rclpy.spin_once(node, timeout_sec=0.02)`
   interleaved every iteration — telemetry keeps draining throughout the
   wait, the same pattern as the async arm-parameter fix (Anomaly 1).
2. **Explicit pre-arm window reset.** Immediately after the bridge reports
   ready, `node.rows["left"/"right"/"bridge"].clear()` and
   `pre_arm_window_start_t = time.time()` are recorded; the existing
   dedicated collection loop then fills a provably fresh window from that
   point. `evaluate_pre_arm_samples()` gained two new parameters:
   `window_start_t` (defense-in-depth — flags any row that somehow
   predates the reset) and `decision_time_t` (a genuinely distinct
   **last-sample-age** check — catches "everything was fresh throughout
   the window, but nothing has arrived since the window ended," which a
   max-inter-sample-gap check alone cannot).

**Both new fields are persisted** into `part1_pre_arm_result.json`
(`pre_arm_window_duration_s`, `left/right/bridge_final_sample_age_s`)
automatically, since they're part of `evaluate_pre_arm_samples()`'s own
return dict.

**End-to-end confirmation** (real `part1_gravity_settle_capture.py`
entrypoint, sim devices on the real topic names, isolated domain): after
the fix, `PRE_ARM_PASS` with `window_duration_s=0.198s`,
`left_final_age_s=0.003s`, `right_final_age_s=0.001s` — armed, reached
`GATE PASSED` at `quiet_window_accumulated_s=0.150s`, clean teardown
(`status=CLEAN`, 0 remaining graph endpoints). The 0.2s stale threshold
was never changed; no grace period was added; a genuine >0.2s in-window
gap or a stale last-sample at decision time both still correctly fail
(regression-tested).

## Task 4: PRE_ARM_FAIL artifact handling

The second real run correctly failed pre-arm but left no `part1_meta.json`
at all, crashing `analyze_h2_gravity_settle.py` with `FileNotFoundError`.
Fixed: every exit path (pre-launch refusal, pre-arm fail, arm-request
failure, post-arm-freshness failure, normal completion, or any unhandled
exception) now writes a complete `part1_meta.json` via
`_write_pre_arm_fail_meta()`/the normal completion path, and
`analyze_h2_gravity_settle.py` recognizes a `PRE_ARM_FAILED`/
`PRE_LAUNCH_FAILED` artifact and reports it directly rather than running
telemetry analysis on empty data; a top-level `try/except` in `main()`
converts any other unexpected exception into a structured
`ANALYSIS_INCOMPLETE` result instead of a raw traceback.

## Test results

- New regression tests: 4 pure (`test_part1_relay_abort_check.py`, prior
  session) + 4 pure (`test_part1_pre_arm_fail_artifact.py`, Task 4) + 5
  live-sim (`test_h2_arm_orchestration_live.py`, Tasks 1-3/5).
- Full pure/unit regression: **106 passed, 0 failed** (was 98 before this
  investigation).
- Full live-sim regression (18 files): see the final report for the exact
  count from this investigation's run.
- Manual end-to-end smoke test (sim, no deadman for speed): armed → 
  `GRAVITY_RAMP_UP` → `GRAVITY_SETTLE` → `GATE PASSED` (quiet_window_
  accumulated_s=0.150s, exactly at `hold_s`) → disarmed, with zero false
  aborts from either fixed mechanism — confirms the fixes work together
  on the actual happy path, not just in isolation.
