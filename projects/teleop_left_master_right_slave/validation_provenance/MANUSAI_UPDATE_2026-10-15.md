# manusAI update — 2026-10-15

## Branch

`manusAI-update-2026-10-15`

This update is intentionally **not merged into `main`**.

## What changed

### 1. Added a current readiness source of truth

Added:

```text
validation_provenance/CURRENT_READINESS.md
```

It separates current implementation facts from historical hardware evidence and
records the required H0–H9 validation order. It explicitly documents:

- C1's current negative/opposing relay sign
- C2/C3 position ownership by `bridge_master_slave.py`
- C4's positive relay sign
- supervised `/enable` command authority
- the default C1 shoulder mask
- direct joint-torque writing remaining disabled
- the fact that DOB force is a model-based estimate, not a sensor measurement

### 2. Corrected stale operator documentation

Updated the root and project READMEs so they no longer present an older
2026-09-03/2026-09-07 status as the current qualification state. They now link
to `CURRENT_READINESS.md` and distinguish historical trials from current
checkout evidence.

Also corrected the build-option description: `TOUCH_HAPTICS_ENABLE_REAL_TORQUE_WRITE`
is a separate experimental direct joint-torque path. The supervised C1/C4
relay uses joint relay messages converted to Cartesian force and writes through
`HD_CURRENT_FORCE`; it does not require the compile-time joint-torque gate.

### 3. Added Git provenance to hardware preflight

`experiment_launch/00_preflight.sh` now records:

```text
GIT_COMMIT
GIT_BRANCH
GIT_WORKTREE_CLEAN
```

in the session output and `manifest.env`. This prevents hardware captures from
being detached from the exact source version and makes dirty-tree testing
visible instead of implicit.

## What was deliberately not changed

- No controller gains were changed.
- No C1 shoulder unmasking was performed.
- No direct `HD_CURRENT_JOINT_TORQUE` write was enabled.
- No hardware qualification was claimed or fabricated.
- `main` was not merged or modified remotely.

## Validation performed

Passed:

- `git diff --check`
- `bash -n` for the modified preflight and launch scripts

Not run in this sandbox:

- ROS/OpenHaptics hardware build
- physical H0–H9 procedure
- pytest suites, because `pytest` is not installed in the current sandbox

The pushed branch therefore contains documentation/provenance changes only;
real-hardware qualification must still be performed on the physical setup.
