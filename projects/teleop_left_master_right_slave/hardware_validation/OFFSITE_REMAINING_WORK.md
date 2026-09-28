# Off-site remaining work (2026-09-10)

Everything in this list is work that CANNOT be completed without either
physical hardware access (a person beside the robots) or a user decision.
Nothing below was left undone because of time — each item has a concrete
reason it stops here.

## Requires physical hardware access

1. **H2–H9 hardware execution** (`ONSITE_RUNBOOK.md`) — every stage is
   code-ready or "prepared but hardware-unvalidated"; none has touched
   real hardware under this session's changes. This is the large majority
   of remaining work and is fundamentally not off-site-completable.
2. **H3/H4 real-hardware manual-motion recorder** — designed
   (`ONSITE_RUNBOOK.md` H3 section) but writing it further without a
   person to test the deadman/hold-while-moving ergonomics against risks
   building the wrong tool. Implementing the script itself IS off-site
   work and remains open — see the "still buildable off-site" section
   below; the ergonomics QUESTION is what's physical-access-only.
3. **C1's corrected sign validation on real hardware** (H7) — simulated
   only (`test_c1_sign_vs_ground_truth.py`); the sign could theoretically
   still be wrong in a way the sim's contact model doesn't capture (e.g.
   a hardware-specific joint-to-Cartesian Jacobian sign flip not present
   in the simulated kinematics).
4. **C4 re-validation since 2026-09-03** — same category as (3), lower
   risk (sign was not flagged as wrong) but untested against this
   session's deadman/enable-consolidation changes.
5. **`gravity_settle_velocity_threshold_mps` (0.005 m/s)** — SIM-derived
   only; real Touch device velocity-signal noise floor is unknown and may
   require a different threshold to avoid false-quiet or never-settling
   behavior.
6. **All ten DOB/Kd/gravity TODO items** (item 13 of the task list:
   Q-filter cutoff, injected-disturbance validation, noise/bias/deadband
   characterization, joint-to-Cartesian interpretation confirmation, C4
   DOB use, Kd sweep, 6-8x overdamped observation, gravity-FF ON/OFF
   isolation) — each requires either a real disturbance/contact or a real
   noise sample from the physical sensors; none are simulatable with the
   fidelity this decision needs. NOT reconciled this pass beyond the two
   already-settled items (`left_gravity_comp_n=0.0`,
   `slave_gravity_comp_n=0.55` — both in `frozen_config.py`'s docstring).

## Requires a user decision

7. **H3's deadman ergonomics** — the same operator cannot simultaneously
   hold a deadman button AND move the master by hand with normal grip.
   Options: (a) a hands-free deadman (foot switch, or a second person),
   (b) relax the deadman requirement specifically for C3-only stages
   where the master's own controller force is zero (— this would be a
   real, considered exception, not an oversight, since C3's force lands
   on the SLAVE, not the master, so "master deadman gates master force"
   doesn't obviously apply the same way as it does to gravity/C2/C1).
   This is a genuine policy choice, not a technical one; not resolved
   here, flagged explicitly in `ONSITE_RUNBOOK.md`'s H3 section instead
   of silently picking (a) or (b).
8. **Whether to widen the C1 shoulder mask** — currently masked out under
   `SAFETY_ONLY`/`LEGACY_COMMISSIONING_PRESET` classification. Whether to
   attempt unmasking it is a risk decision, not something to change
   unilaterally.
9. **`--w1`/`--w4` starting weights for H7/H8** — `ONSITE_RUNBOOK.md`
   recommends a conservative 0.3 for a first attempt rather than the
   `4ch_safe` (0.5) or `4ch` (1.0) presets; final weight choice for a
   real first attempt is the operator's call at the time, informed by
   what they feel, not something to pre-commit to in a document.

## Still buildable off-site (tracked, not yet done — no hardware/user
## blocker, just not reached this pass)

10. **DONE (2026-09-10, H3/H4 recorder tooling task):** H3/H4
    real-hardware recorder SCRIPT (distinct from the ergonomics question
    in (7) above) — `hardware_validation/record_manual_motion.py` +
    `analyze_manual_motion.py`, simulation-validated end-to-end. The
    deadman-ergonomics answer from (7) is exposed as
    `--operator-mode {single_operator,assisted_deadman}`, metadata only —
    (7) itself remains open, this item is no longer blocked on it.
11. `hardware_validation/tests/` consolidation — currently intentionally
    left colocated with what they test (see `hardware_validation/README.md`)
    to avoid breaking relative imports mid-task; a later pass could move
    them without functional risk.
12. Update `FOUR_CHANNEL_TODO.md` with this session's findings (command-
    authority fix, channel gating, deadman integration) — not done this
    pass; `CHANNEL_READINESS_MATRIX.md` and this file are the current
    source of truth instead.
