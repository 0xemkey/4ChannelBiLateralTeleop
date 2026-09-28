# Y-axis position diagnosis: gravity-comp A/B test

Date: 2026-09-08
Session: `y_axis_diag_20260908` (`~/HIL-RL/experiment_runs/teleop_setup/y_axis_diag_20260908`)
Config: C3=ON, C1=C2=C4=OFF (production `03_start_position_bridge.sh` topology), startup alignment active, all normal watchdogs/clamps active.

## Question

Is the bridge's constant Cartesian gravity-compensation term (`SLAVE_GRAVITY_COMP_N=0.55`, `SLAVE_GRAVITY_AXIS=1`, i.e. Y) the primary cause of the observed Y-axis position bias?

## Y mapping — confirmed correct (code trace, no hardware needed)

Full trace of `touch_slave.cpp` + `bridge_master_slave.py`: Y = index 1 consistently in the device driver, both ROS pose topics, and the bridge's `axis_map`. All `invert_*`/`swap_xy` params are `false` in every production launch script. 0-indexed throughout, no sign/scale/swap bug found. This was not re-tested on hardware in this session since it was already conclusively settled by direct code reading.

## Runs

| | Gravity ON (baseline) | Gravity OFF (diagnostic) |
|---|---|---|
| Bridge launch | `03_start_position_bridge.sh --session y_axis_diag_20260908 --arm-position` (session-tracked) | `run_bridge.sh` invoked directly with `SLAVE_GRAVITY_COMP_N=0`, same `CALIBRATION_FILE` path, all other args identical (not session-pid-tracked, per the runbook's "short diagnostic" design) |
| `SLAVE_GRAVITY_COMP_N` | 0.55 N | 0.0 N |
| `SLAVE_GRAVITY_AXIS` | 1 (Y) | 1 (Y) |
| Capture file | `~/dob_logs/y_axis_diag/gravity_on/single_axis_slow_y_rep3_20260908_145248.csv` | `~/dob_logs/y_axis_diag/gravity_off/single_axis_slow_y_rep1_20260908_145552.csv` |
| Duration | 35.0 s, 7000 samples | 34.9 s, 6997 samples |
| Motion | Continuous slow up/down (Y) hand motion, operator-driven | Same style, same operator |

**Note on motion quality:** two earlier attempts were discarded before these: one had zero motion (operator/capture timing didn't sync — no live cue was possible across a text-turn interface), one had heavy, unintended X/Z coupling. The runs used here have well-isolated X (RMS(X)/RMS(Y) ≈ 0.13 both runs) but non-trivial Z coupling (RMS(Z)/RMS(Y) ≈ 0.55 ON, 0.66 OFF) and larger amplitude than the originally-planned ±10mm (actual Y std ≈ 43–49mm). Stationary-baseline/travel-limit/saturation checks below confirm the data is still usable for this specific gravity question; it is **not** the small-amplitude controller-tuning dataset called for in a later stage.

Anchors were freshly captured at each bridge (re)start (normal `startup_alignment.py` behavior), so the two runs do not share the exact same anchor point — a secondary, expected source of run-to-run difference, distinct from the gravity question.

## Metrics

| Metric | Gravity ON | Gravity OFF |
|---|---:|---:|
| RMSE | 17.6 mm | 28.9 mm |
| MAE | 14.2 mm | 24.1 mm |
| **Mean signed error** | **+8.3 mm** | **+23.9 mm** |
| Peak abs error | 56.9 mm | 53.9 mm |
| p95 abs error | 30.6 mm | 44.6 mm |
| Estimated lag | 85 ms | 90 ms |
| Cross-axis RMS(X)/RMS(Y) | 0.129 | 0.128 |
| Cross-axis RMS(Z)/RMS(Y) | 0.549 | 0.658 |
| Travel-limit % | 0.0% | 0.0% |
| Force-saturation % (logged flag) | 0.0% | 0.0% |
| % time \|error\| > 40mm clamp | 2.2% | 16.1% |
| **First 1.5s: slave_y std / range** | **1.5mm / 3.2mm** | **0.000mm / 0.000mm** |

Full metrics: `docs/y_axis_diagnosis_plots/metrics.json`.

## Plots (`docs/y_axis_diagnosis_plots/`)

- `y_target_vs_actual_on_vs_off.png` — Y target vs actual, both runs
- `y_position_master_vs_slave.png` — master vs slave Y, both runs
- `y_tracking_error_on_vs_off.png` — `c3_position_error_y` overlay
- `y_controller_decomposition.png` — spring/damping/gravity/total force, both runs
- `y_stationary_drift_comparison.png` — first-1.5s zoom (drift/stiction check)
- `x_z_cross_axis_comparison.png` — X/Z motion during the Y test
- `y_limit_saturation_flags.png` — travel-limit/force-saturation flags over time

**Controller decomposition caveat:** `spring_y`/`damping_y`/`gravity_y` are *reconstructed* from logged position/error data and the known fixed params (stiffness 35.0 N/m, damping 0.20 Ns/m, gravity comp 0.55/0.0 N) — `log_phase2_scenario.py` does not currently subscribe to `CartesianControllerTelemetry`, which would carry these directly (a known gap, already flagged in `OPERATOR_PROCEDURE.md`'s missing-telemetry list). The reconstructed total-applied-force line was cross-checked against the actually-logged `slave_applied_force_y` telemetry in the same plot — the two match almost exactly in both runs, which validates the reconstruction.

## Findings

1. **Gravity OFF makes Y tracking substantially and directionally worse, not better.** Mean signed error nearly triples (+8.3mm → +23.9mm), RMSE increases 65% (17.6mm → 28.9mm), and time spent against the 40mm error clamp goes from 2.2% to 16.1%.
2. **In the OFF run, the slave was completely unable to move in Y for the first 1.5 seconds** after arming (position std = 0.000mm, range = 0.000mm) while the commanded target was already changing — a clean, unambiguous stiction/can't-overcome-gravity failure, not a subtle bias.
3. **The target-vs-actual plot shows a clear directional asymmetry in the OFF run**: the slave tracks the downward (negative-Y, gravity-assisted) excursions reasonably well but consistently fails to reach the upward (positive-Y, against-gravity) excursions — e.g. target swings to +55mm while actual barely exceeds +10mm. The ON run does not show this asymmetry to nearly the same degree.
4. The reconstructed decomposition shows the OFF run's spring force pinned at the 1.5N primary clamp for long, wide plateaus during upward strokes — the controller is trying and failing to lift against unassisted gravity, not producing a small steady bias.
5. No travel-limit or force-saturation *flag* was set in either run (0.0%/0.0%), so the degradation in OFF is not an artifact of hitting the workspace/travel clamp — it is the Cartesian force clamp (1.5N) being reached routinely while still short of target, i.e. a genuine force-authority shortfall against gravity.

## Root-cause classification

**The constant +0.55N Y gravity compensation is NOT the primary cause of the Y-axis bias — it is currently mitigating a larger underlying gravity-loading problem, and removing it makes tracking meaningfully worse.**

The residual bias that remains *with* gravity comp ON (+8.3mm mean, 17.6mm RMSE) is still real and non-trivial, and is consistent with the compensation's magnitude being **insufficient** rather than wrong in sign or concept — matching the 2026-08-26 sweep noted in `run_bridge.sh`'s own comments, which found 0.55N improved Y reach ratio from 0.69 to 0.85, not to 1.0. This is the "retuned magnitude" scenario the task's own interpretation guidance anticipated, not a case for removing compensation.

**The ~13mm static Y anchor gap remains suspicious and unaddressed by this test.** Both runs used freshly-captured anchors (normal startup behavior), so this A/B test does not isolate the anchor-gap contribution from the gravity-loading contribution. It should be treated as a separate, still-open secondary item — this diagnosis neither confirms nor rules it out as a contributor to the ON-condition's residual +8.3mm bias.

## Recommended next action

1. **Do not remove or zero the gravity compensation.** The data argues for increasing/retuning its magnitude or making it configuration/pose-dependent, not eliminating it.
2. Before any controller-gain tuning (stiffness/damping), get a cleaner small-amplitude (±5/10/20mm), well axis-isolated Y-only dataset — this session's runs, while sufficient to answer the gravity question, are too large-amplitude and Z-coupled for the tuning-stage metrics.
3. Separately investigate the ~13mm anchor gap's contribution to the residual ON-condition bias (this was flagged, not tested, here).
4. Only after 2–3 above should Y-axis controller tuning (step 7 of the original task) begin.

## Session state after this report

Hardware was left in the following state pending your decision: master/slave nodes running (session `y_axis_diag_20260908`), gravity-OFF bridge instance from Stage 1B still active/armed unless stopped. Recommend stopping force output (`99_stop_setup.sh --session y_axis_diag_20260908`, or just re-killing the untracked gravity-off bridge process) before leaving the rig unattended.
