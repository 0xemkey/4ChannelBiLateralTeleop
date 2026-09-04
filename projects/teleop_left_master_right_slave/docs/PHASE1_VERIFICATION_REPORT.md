# Phase 1 verification report — bug fixes prerequisite

Date: 2026-09-04. Written under an explicit session constraint: no code was
run against physical hardware (the operator was not at the rig). Everything
below was verified via static/sim/unit-test means only, clearly marked
where real-hardware verification is still the open step.

Companion reading: `SYSTEM_ARCHITECTURE.pdf` (mechanism reference),
`FOUR_CHANNEL_TODO.md` (full chronological log).

---

## 1.1 Silent channel disablement — **PASS**

**Change:** `bilateral_4ch.py`'s `__init__` now logs, per channel, an
explicit `WARN` if its target weight is `0.0` (e.g. `"C4 (master force ->
slave): DISABLED (weight=0.0)"`) or an `INFO` with the live weight
otherwise — on top of the pre-existing single-line weight-list log, which
required the reader to already know the channel order to interpret.

**Evidence:** code reviewed directly; `python3 -m py_compile` clean;
exercised indirectly by `test_channel_presets.py` (5/5 presets pass,
confirms the node starts and logs without error at each weight
combination, including presets with `w1=w4=0`).

**Acceptance criterion met:** at startup, which channels are active is
now printed explicitly and individually, not left to be inferred from a
raw weight tuple.

---

## 1.2 Deadband stacking consolidation — **PASS**

**Change:** added `effective_threshold_nm(weight)` to `bilateral_4ch.py`,
computing `THRESHOLD_NM[j] + MIN_RELAY_TORQUE_NORM_NM / weight` per joint
(`+inf` at `weight<=0`). Published live every tick to two new
`BilateralStatus.msg` fields, `effective_threshold_c1_nm` and
`effective_threshold_c4_nm`, computed at the current *ramped* weight (not
the target), so it reflects what's actually required right now during a
ramp.

**W-value range documented** (solving `effective_threshold <= 0.1` N·m,
this session's working "real touch" reference size):

| joint | required weight |
|---|---|
| waist | ≥ 0.845 |
| shoulder | ≥ 0.556 (moot — masked out of C1 by default) |
| elbow | ≥ 0.400 |

Elbow is the binding joint for C1 (shoulder masked, waist stricter than
elbow) — this matches 2026-09-03's live finding exactly: at `w1=0.4`,
elbow relayed a real push while waist needed noticeably harder pushing to
clear its own stricter floor.

**Evidence:** `test_oscillation_guard.py::TestEffectiveThreshold` (3/3
pass) checks the `w=0` → `+inf` case, the documented elbow-at-`w=0.4`
value lands at exactly `0.1` N·m, and monotonicity (higher weight → lower
threshold). `.msg` change built cleanly (`colcon build --packages-select
touch_haptics`, no errors).

**Known limitation, documented not silently accepted:** `THRESHOLD_NM`
and `MIN_RELAY_TORQUE_NORM_NM` in `bilateral_4ch.py` are hand-kept copies
of `touch_slave.cpp`'s `kThreshold`/`kMinRelayTorqueNormNm` — that value
lives in C++ and is not currently exposed over a ROS topic or parameter.
If the C++ values are ever changed, this Python copy must be updated by
hand or the published effective threshold will silently report a stale
number. Flagged in both files' comments; not fixed in this pass (would
require exposing threshold_nm from `DobConfig` over ROS, out of Phase 1's
scope as specified).

---

## 1.3 Oscillation-abort latch false-positive fix — **PASS**

**Change:** added `analyze_window()`, a pure function (no ROS dependency,
same "messaging logic, test it standalone" precedent as
`touch_force_estimate.hpp`) computing both peak-to-peak norm AND a
per-axis zero-crossing rate (Hz) over each 1s window. The existing
"growing" early-warning condition now additionally requires
`zero_crossing_hz >= min_oscillation_hz` (new `--min-oscillation-hz`
CLI arg, default 1.5 Hz). The hard `pp > trip_pp_nm` ceiling is
**unaffected** — it remains an unconditional backstop regardless of
periodicity, deliberately: a large-enough push should still cut
regardless of whether it happens to be periodic.

**Verified separately against both required scenarios**
(`test_oscillation_guard.py`, 12/12 pass, plus 4 pre-existing
`test_c1_relay_live.py` integration tests unaffected — 33/33 total sim
suite pass, 1 pre-existing unrelated skip):

| scenario | zero-crossing rate | old behavior | new behavior |
|---|---|---|---|
| Monotonic escalating push (this session's own repeated false-trip pattern) | ~0 Hz | **tripped** (false positive) | does not trip |
| Single push-and-hold (2026-09-02's original false-positive repro) | 0 Hz | correctly did not trip | still does not trip |
| Genuine 4 Hz oscillation, same rising-pp shape | ~8 Hz | tripped | **still trips** (protection preserved) |
| Non-rising windows, high frequency | any | did not trip | still does not trip (sanity check) |

**Acceptance criterion met:** neither scenario is misclassified — direct,
deterministic proof via unit test, not a single anecdotal run.

**Open item, not a blocker:** the pre-existing sim integration tests
(`test_c1_no_false_trip_on_gentle_ramp`,
`test_c1_oscillation_trip_repeated_push_baseline`) still pass, but neither
was written to specifically exercise a *periodic* signal through the real
subprocess path — the periodicity discrimination itself is proven at the
unit level only. A sim-level integration test injecting a genuine
sinusoidal `force_ext_hat` through a live `bilateral_4ch.py` subprocess
would close that gap; not built this pass since the unit-level proof
already satisfies the stated acceptance criterion.

---

## 1.4 Reproduction/resolution of the 2.79 N·m anomaly — **PASS (real hardware, 2026-09-04)**

**Real-hardware repro run completed 2026-09-04**, using the script below,
on the real rig: 25/25 restart cycles, **0 anomalies**, every single peak
C4 reading exactly `0.0` N·m (fully at rest, as expected with nothing
touching either arm), all 25 master-node restarts came up cleanly (no
`node_up: false` entries), and the `AT CONFIGURE` trace confirmed
`lpf_`/`u_prev_`/`tau_dis_hat` all read exactly `0.000000` for every joint
on every single restart. Full data:
`~/dob_logs/restart_repro_20260904_163423/summary.json` +
per-run `master_stderr.log`/`c4_capture.txt`.

**Conclusion, meeting the task's own no-vague-language requirement:** the
restart-transient-alone hypothesis is **ruled out** — a plain master
restart with nothing touching either arm does not reproduce the anomaly,
across 25 independent trials. Filter-state leakage was already ruled out
by code guarantee + sim; this real-hardware run confirms that guarantee
holds in practice, every time, not just in principle. **What remains
undetermined** (and cannot be resolved without the original incident's
exact push timing, which was never recorded): whether the anomaly
requires a push occurring simultaneously with or immediately after a
restart to manifest. That scenario was not tested here — it needs a
human-in-the-loop variant of this script (push cued partway through each
settle window), not built this pass, and is now the only remaining
open thread on this item. Given 25/25 clean resting-state trials, the
practical risk this poses to normal operation (restart, then wait a
moment before resuming) is assessed as low, but not formally zero.

<details>
<summary>Original code-review/sim-only findings (superseded above by the real-hardware run, kept for the record)</summary>

**DOCUMENTED, NOT REPRODUCIBLE THIS SESSION (real hardware required)** — this was the status before hardware became available.

**What was ruled out by code review + sim, this session:**
- **Filter-state leakage across restarts.** `JointDob::reset()` (called
  unconditionally at the end of every `configure()`) provably zeroes
  `lpf_` and `u_prev_`. Added public accessors (`lpfState()`,
  `uPrevState()`) and an env-gated trace
  (`TOUCH_DIAG_DOB_RESTART_TRANSIENT=1`) that prints both immediately
  after every `configure()` call. **Confirmed empirically in sim**: both
  read exactly `0.000000` for all three joints at configure time (see
  smoke-test output below) — this candidate is ruled out, not merely
  assumed correct from reading the header.
- **A DOB-math-intrinsic startup spike.** The same trace, extended to the
  first 10 servo frames after configure, shows `tau_dis_hat` staying
  small (≤0.01 N·m) and `q_rad`/`qdot` behaving smoothly through sim's own
  clean startup — the DOB code itself does not manufacture a large
  spurious estimate from a clean restart with zero real disturbance.

**What was NOT ruled out, and cannot be from this session:**
- A **real-hardware-specific** transient — most plausibly
  `HD_CURRENT_JOINT_ANGLES` returning a stale/garbage first reading before
  OpenHaptics' internal buffer has refreshed after a device re-init, which
  would alias directly into `tau_dis_hat` via the `g·M_n·q̇` term on the
  first primed frame. Sim cannot exercise this because `hd_sim_api.cpp`'s
  simulated device does not model that failure mode at all — a clean sim
  result here is expected either way and does not confirm or rule out the
  real-hardware hypothesis.
- Whether the original incident's push happened before, during, or after
  the restart (not recorded at the time).

**Instrumentation added** (`touch_dob.hpp`: `lpfState()`/`uPrevState()`
accessors; `touch_slave.cpp`: `TOUCH_DIAG_DOB_RESTART_TRANSIENT`-gated
trace at configure-time and for the first 10 frames after) and **smoke-tested
in sim** — compiles cleanly, runs without crashing, produces the expected
trace format:

```
[diag_dob_restart] joint=0 AT CONFIGURE lpf=0.000000 u_prev=0.000000 tau_dis_hat=0.000000
[diag_dob_restart] joint=1 AT CONFIGURE lpf=0.000000 u_prev=0.000000 tau_dis_hat=0.000000
[diag_dob_restart] joint=2 AT CONFIGURE lpf=0.000000 u_prev=0.000000 tau_dis_hat=0.000000
[diag_dob_restart] frame=1 q_rad=(0.00000,0.45000,0.90000) qdot=(0.00000,0.00000,0.00000) ...
...
[diag_dob_restart] frame=10 q_rad=(...) qdot=(...) tau_dis_hat=(0.00002,-0.00529,0.00088) ...
```

**Repro script written** (at the time, not yet executed — real hardware
was unavailable this session): `projects/teleop_left_master_right_slave/scripts/repro_dob_restart_anomaly.py`.
Restarts only the master node 20–30 times (default 25) with nothing
touching either arm, capturing C4's relay output and the restart trace
each cycle, producing a `summary.json`/`summary.txt` with either a
reproduction count (cross-referenced against the trace at the moment of
each anomaly) or an explicit "not reproducible, conditions X and Y ruled
out" statement.

</details>

**Update, 2026-09-04, same day, real hardware became available:** the
script above was run for real — see the result at the top of this section.
25/25 clean, restart-alone hypothesis ruled out. This closes the
sim/code-review gap the box above describes.

---

## 1.5 Shoulder identification / base-tilt — **CARRIED FORWARD, per the task's own instruction**

Checked `right-arm-identification-status` memory + `FOUR_CHANNEL_TODO.md`:
Phase 5 (base leveling) has **not** happened. Per the task's own branching
instruction ("if not complete, carry this item into Phase 2 with a
'shoulder still masked' note; it is not a blocker"), no action taken here
— shoulder stays masked via `bilateral_4ch.py`'s existing
`--c1-disable-joints=shoulder` default, unchanged.

---

## Regression check

Full `sim/tests/` suite, run twice (once mid-Phase-1, once after all
changes including the 1.4 C++ instrumentation): **33 passed, 1 skipped
(pre-existing, unrelated — missing tool binary), 0 failed**, both runs
identical.

## Files touched this phase

- `ros2_ws/src/touch_haptics/msg/BilateralStatus.msg` — 3 new fields (1.2, 1.3 diagnostics)
- `projects/teleop_left_master_right_slave/scripts/bilateral_4ch.py` — 1.1, 1.2, 1.3
- `ros2_ws/src/touch_haptics/include/touch_haptics/touch_dob.hpp` — 1.4 accessors
- `ros2_ws/src/touch_haptics/src/touch_slave.cpp` — 1.4 diagnostic trace
- `ros2_ws/src/touch_haptics/sim/tests/test_oscillation_guard.py` — new, 1.2/1.3 unit tests
- `projects/teleop_left_master_right_slave/scripts/repro_dob_restart_anomaly.py` — new, 1.4 repro tool, later run for real (see below)
- `projects/teleop_left_master_right_slave/scripts/log_phase2_scenario.py` — new, Phase 2 synchronized logger (built ahead of the repro run finishing; sim-smoke-tested, not yet real-hardware-tested)
- `projects/teleop_left_master_right_slave/scripts/plot_phase2_scenario.py` — new, Phase 2 plotting (sim-smoke-tested)

## Phase 1 — fully closed, 2026-09-04

All five items resolved: 1.1–1.3 sim/code-verified same day; **1.4
confirmed on real hardware the same day** (25/25 clean restart cycles,
restart-alone hypothesis ruled out — see updated section above); 1.5
correctly carried forward per the task's own branching rule (not a
blocker). Phase 2 is clear to begin.
