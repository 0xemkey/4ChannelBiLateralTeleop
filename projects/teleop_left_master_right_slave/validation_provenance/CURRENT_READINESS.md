# Current Four-Channel Readiness

**Update:** `manusAI update` — 2026-10-15

This file is the current checkout's operator-facing readiness snapshot. Older
reports remain valuable historical evidence, but they may describe an earlier
commit, an earlier sign convention, or an earlier publisher arrangement.
Always record the Git commit and worktree state from `experiment_launch/00_preflight.sh`
with every hardware run.

## Current architecture

| Channel | Meaning | Current implementation | Hardware status |
|---|---|---|---|
| C1 | Slave DOB force estimate → master | `bilateral_4ch.py` applies `-w1 * cf`, then `touch_slave.cpp` converts joint relay torque to Cartesian force and adds it to the master `HD_CURRENT_FORCE` path | **Requires fresh H7 validation with the corrected sign** |
| C2 | Slave position → master | `bridge_master_slave.py` target pose and master Cartesian spring/damper path | Previously smoke-tested; rerun under the current checkout before qualification |
| C3 | Master position → slave | `bridge_master_slave.py` anchor-relative target and slave Cartesian spring/damper path | Previously smoke-tested; rerun under the current checkout before qualification |
| C4 | Master DOB force estimate → slave | `bilateral_4ch.py` applies `+w4 * cf`, then the slave Cartesian relay path | **Requires fresh H8 validation under the current checkout** |

## Command authority

In the supervised architecture, `bridge_master_slave.py` is the sole `/enable`
publisher. `bilateral_4ch.py` defaults to `--publish-enable=false` and must
not be started with `--publish-enable` when the bridge is running. The latter
flag is reserved for a standalone legacy run with no bridge.

The force relay therefore cannot independently override bridge arm, deadman,
startup-alignment, or FAULT state. The force-relay launcher records
`FORCE_RELAY_PUBLISH_ENABLE=false` in the session manifest.

## Safety defaults

- Direct `HD_CURRENT_JOINT_TORQUE` writing remains compile-time gated off for
  real hardware (`TOUCH_HAPTICS_ENABLE_REAL_TORQUE_WRITE=OFF`).
- C1 shoulder relay remains masked by default because its identified noise and
  friction floor has not been accepted for commissioning.
- C1 and C4 weights must start conservatively for first hardware validation.
- Relay input must be fresh and valid; stale input produces zero relay force.
- The receiving controller applies the final Cartesian force clamp after the
  relay contribution.
- An oscillation trip is latched and requires explicit manual re-arm.

## Required validation order

Do not jump directly to full four-channel operation:

1. H0 preflight and topology inspection
2. H1 disabled/free-space baseline
3. H2 gravity-only and settle check
4. H3/H4 C3 position tests
5. H5 C2-only test
6. H6 C2+C3 test
7. H7 C1-only test, including real-hardware sign validation
8. H8 C4-only test
9. H9 full four-channel test

A channel is not considered current-checkout qualified merely because an older
report says it was hardware-tested.

## Evidence required for H7–H9

Each run must preserve:

- Git commit, branch, and worktree state from `manifest.env`
- launch arguments and channel weights
- left/right device identity
- DOB and gravity/inertia file paths
- source force estimate and deadbanded estimate
- relay torque, relay Cartesian force, pre-clamp force, and applied force
- freshness, enable, deadman, singularity, clamp, tank, and trip flags
- raw capture plus analyzer output

Minimum checks:

```text
C1: tau_C1 / tau_ext_slave ≈ -w1*cf for samples above deadband
C4: tau_C4 / tau_ext_master ≈ +w4*cf for samples above deadband
stale relay input => relay force = 0
final applied force norm <= configured maximum
C1 shoulder torque = 0 while the default mask is active
```

## Explicit non-claims

This repository still does not contain a physical contact-force sensor. DOB
outputs are model-based residual estimates and can include gravity, friction,
inertia, frame, and velocity errors. The current software can be qualified
for stable, repeatable, correctly signed relay behavior within a tested range;
it cannot by itself certify that the estimate equals true contact force.
