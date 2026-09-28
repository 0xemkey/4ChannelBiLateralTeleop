# Right/slave gravity identification — final result (Parts A & B)

Date: 2026-09-08
Sessions: `pilot_20260908`, `pilot_rerun_20260908`, `full_id_20260908`
(combined: `~/dob_logs/full_id_20260908/combined_all_poses.csv`, 46 rows,
17 unique poses)

**Bottom line: the deterministic gravity model does NOT pass its own
validation gate. Recommendation: keep the empirical 0.70N Y baseline
unchanged, do not integrate a new model, proceed to system-level
4-channel validation (Parts E-J) with the existing baseline.**

## Part A — full pose campaign

17 poses collected (up from the pilot's 7), using the live-pose-matching
tool validated earlier the same day. Coverage: waist -45° to +45°,
shoulder -7° to 65°, elbow 15° to 102° — spans low/mid/high on all three
joints, including the previously-underrepresented low-elbow and
high-shoulder+high-elbow regions, plus 2 deliberately repeated poses
(3↔15 high region, 6↔16 low region) for a repeatability check.

**Pose-matching quality: excellent.** 16 of 17 poses met the ≤0.05 rad
tolerance (most well under 0.03 — median pose_delta ≈0.009 rad). Only
pose 1 (carried over from the original pilot, not redone) remains at
0.071 rad and was excluded from fitting. One hold auto-rejected by the
tool's own bias-cap gate (pose 17, first attempt) and correctly excluded,
retried successfully. Zero bias-cap contamination in the accepted data.

Overall sanity: mean `|tau_c|/|tau_g|` ratio = 0.71 (gravity dominates
friction on average, a further improvement over the 4-pose rerun's 0.82),
only 3 of 17 poses flagged for the c/g review check, all at small absolute
magnitude.

## Part B — model fit and validation

Fit `tau_g_hat(q) = c0·cos(q2) + c1·sin(q2) + c2·cos(q3) + c3·sin(q3)` per
joint (same basis as the existing `fit_gravity.py`, robust `soft_l1` least
squares), 12 training poses / 4 held-out poses (spread across the
shoulder-angle range, not a random split, to test both interpolation and
edge behavior). Design condition number: 13.4 for all three joints — well
below the 1000 warning threshold, so this is **not** the collinearity/
overfitting failure mode seen in the old 6→14-pose attempts.

**Held-out validation (N·m):**

| joint | RMSE | MAE | mean signed | p95 | max |
|---|---:|---:|---:|---:|---:|
| waist | 0.01505 | 0.01315 | +0.00495 | 0.02142 | 0.02230 |
| shoulder | 0.00414 | 0.00276 | -0.00247 | 0.00706 | 0.00785 |
| elbow | 0.00375 | 0.00265 | -0.00265 | 0.00632 | 0.00711 |

**Waist RMSE (0.015 N·m) is comparable to the typical signal magnitude
across the whole dataset (roughly -0.02 to +0.016 N·m)** — the model has
essentially no useful predictive power on waist. Three of the four
held-out poses show wrong-sign or wrong-magnitude predictions (pose 2:
predicted +0.0029, actual -0.0194; pose 4: predicted near-zero, actual
-0.0108; pose 11: predicted near-zero, actual +0.0163). Shoulder and elbow
fare somewhat better (RMSE ~0.004 N·m) but are still a large fraction of
their own signal range.

**Repeated-pose consistency check (the deliberate design intent of poses
3/15 and 6/16) — this is the most important result:**

| pair | q_mid (shoulder, elbow) | tau_g pose A | tau_g pose B | |diff| |
|---|---|---|---|---:|
| 3 vs 15 | (44.3°/101.7°) vs (45.1°/99.0°) — **within 3°** | [-0.0045, -0.0037, -0.0044] | [+0.0022, +0.0056, +0.0070] | 0.0162 |
| 6 vs 16 | (-2.4°/44.8°) vs (0.4°/45.9°) — **within 3°** | [+0.0121, -0.0039, -0.0058] | [+0.0002, -0.0016, -0.0019] | 0.0127 |

**Verified the q-values are nearly identical between each repeated pair
(within 3° on every joint) — the discrepancy is not explained by residual
pose mismatch.** This is genuine measurement-to-measurement variability in
the tau_g estimate, at a magnitude (0.013-0.016 N·m) comparable to the
entire useful signal range of the dataset.

## Interpretation

The gravity torques on this device, at least at the waist, are simply
**very small** (single-digit to low-double-digit mN·m) — comparable in
magnitude to whatever hold-to-hold variability exists in the
bias-holding-force measurement method itself (backlash, stiction, grip
micro-position, stylus contact variation). This is not a new problem:
the *original* 6-pose model's own metadata already flagged
`waist_gravity_max_abs_nm ≈ 0.013` and a poor waist R² (0.396) — this
campaign, with far better pose-matching and much wider coverage, confirms
that low waist SNR is a property of the measurement, not an artifact of
poor pose-matching (which was this task's leading hypothesis before this
campaign, and which the January... — 2026-09-08 pilot rerun did resolve
for the *friction* sign confusion, but does not resolve for gravity
repeatability, a separate issue).

## Acceptance gate result

Per Part B's explicit criteria:
- ❌ held-out performance is NOT reasonable (waist RMSE ≈ signal range)
- ❌ repeated measurements do NOT agree (diff ≈ signal range, on both
  repeated pairs)
- ✅ no design-collinearity/edge-explosion (cond=13.4, held-out residuals
  bounded, not diverging)
- ⚠️ direction-independence: cannot be cleanly separated from the
  repeatability problem above — the friction-sign audit already
  established that direction *specifically* is well-behaved (Part 4 of
  the prior corrective-rerun work); this is a different, gravity-specific
  noise issue

**2 of 4 hard criteria fail. The model does not pass.**

## Recommendation

**Do not integrate this gravity model.** Per the task's own framing (Part
C is conditioned on Part B validation passing) and the standing
instruction not to turn this into an open-ended subproject:

1. **Keep `SLAVE_GRAVITY_COMP_N=0.70` (the empirical constant Y baseline)
   unchanged.** It remains what it has always been —
   `EMPIRICAL_CONSTANT_Y_BASELINE`, not identified gravity — but it is
   still the best-evidenced, validated option for production use (see
   `Y_AXIS_GRAVITY_SWEEP.md`).
2. **Do not attempt Part C (controller integration) or Part D (A/B
   sanity) with this model.** There is nothing here that would pass a
   fair comparison against the existing baseline.
3. **Proceed directly to Part E-J** (four-channel system validation)
   using the existing, unchanged empirical baseline — this is consistent
   with "leave final parameter fine-tuning for the next stage" and avoids
   further open-ended gravity-model iteration.
4. **If gravity identification is revisited later**, the next productive
   step is almost certainly *more repeats per pose* (5-10x, to average
   out the observed noise) rather than more poses or a different basis —
   the design condition number and pose-matching are no longer the
   limiting factors; measurement noise at these small torque magnitudes
   is.

## Status

**REIDENTIFICATION_REQUIRED** in the sense that the model itself needs
more repeats to be usable, but **NOT** because of rig frame, pose-drift,
or basis-collinearity (all previously-suspected causes are now ruled out
or resolved). The blocking factor is now specifically measurement
repeatability at small torque magnitudes — a different, narrower problem
than any previously investigated.

Proceeding to `FOUR_CHANNEL_SYSTEM_VALIDATION.md` with the existing
0.70N baseline unchanged.
