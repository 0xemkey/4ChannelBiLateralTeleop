# Architecture Decision Record — final command authority (2026-09-10)

## Context

Two independent processes can run against the same two `touch_slave`
nodes: `bridge_master_slave.py` (C2/C3, Cartesian position, gravity comp,
this project's hardened arm/deadman/FAULT state machine) and
`bilateral_4ch.py` (C1/C4, DOB-derived force relay). Audited every
publisher in both files, `touch_slave.cpp`, and every launcher
(`CHANNEL_READINESS_MATRIX.md`'s Publisher Ownership section, 2026-09-10).

**Finding: only ONE command surface actually conflicted** — `/touch_left/enable`
and `/touch_right/enable`. `bilateral_4ch.py`'s own position-blend output
(its "C2/C3") is inert on real hardware (`joint_space_control:=false`).
Cartesian force composition (C2/C3 spring + C1/C4 relay) already happens
correctly, summed, INSIDE `touch_slave.cpp` itself
(`cartesianForceFromJointTorque()` adds directly onto `force_device`,
before the final `clampNorm` — confirmed by reading `touch_slave.cpp:1296`
and the surrounding block) — there is no force-composition bug. The
enable conflict existed because of a real, previously-necessary 2026-09-07
fix: without SOME process publishing the master's `/enable`, C1 silently
did nothing (the bridge only published it when `--enable-c2`/
`--reflect-enable` was also on). `bilateral_4ch.py`'s own independent
publisher fixed that precondition — at the cost of creating exactly the
last-writer-wins hazard this ADR resolves.

## Options considered

**A. Bridge as single final command authority.** The bridge becomes the
sole, unconditional publisher of `enable` for both arms; `bilateral_4ch.py`
becomes purely a force-relay CALCULATOR (still owns
`joint_torque_ext_cmd`, which nothing else publishes), never independently
gating enable when co-launched.

**B. Separate channel workers feeding a single command compositor.** A new,
third process reads C1/C2/C3/C4 contributions from topics and publishes
the single final command. More flexible, but adds a new process, a new
latency hop, and a new thing to keep alive/monitor — for a problem that
(per the finding above) is narrowly scoped to ONE flag, not the whole
force path.

**C. Controller-side composition through distinct component topics.**
Push composition into `touch_slave.cpp` itself, subscribing to named
component topics (spring/gravity/C1/C4) and summing there. This is
actually **already how force composition works** (the DOB relay path), so
this option is really "extend what already exists" — but doing the SAME
for `enable` would mean adding new C++ logic (an AND/OR over per-source
enable flags) to a safety-critical real-time loop, for a decision that a
Python-side supervisor already makes correctly.

## Decision: A — bridge as single final command authority for `enable`

Chosen because:
- **Deterministic safety**: the bridge already owns the ONLY validated
  arm/deadman/FAULT/GRAVITY_SETTLE state machine (73+11 tests this
  session). Extending its authority over `enable` costs nothing new;
  building a second authority (B) or new controller logic (C) would.
- **Update rate/jitter**: the bridge already ticks at 100Hz with the
  deadman check inline — no new process, no new hop.
- **Freshness isolation**: already solved (`_master_deadman_held()`,
  per-topic staleness) — reusing it is free.
- **Testability**: the bridge already has the heaviest test investment
  in this codebase; C1/C4-relevant `enable` behavior now falls under the
  SAME test suite (`test_master_deadman_gate.py`/`_live.py`) instead of
  needing a parallel one for a second authority.
- **Telemetry decomposition**: unaffected either way — decomposition
  (Section 3 of this task) lives in `CartesianControllerTelemetry`,
  already emitted by `touch_slave.cpp` regardless of which Python process
  gates `enable`.
- **Future performance**: B's extra hop would add latency to the
  force-critical C1/C4 path for no safety benefit, since (per the
  finding above) FORCE composition was never actually broken.

## Consequence (implemented 2026-09-10)

- `bridge_master_slave.py`: `left_enable_pub` (master `/enable`) is now
  created and published **unconditionally**, once per tick, with the same
  deadman/FAULT-gated value as the slave's. The previous THREE potential
  publisher paths onto that topic within this single file
  (`left_enable_pub` C2-gated, `master_enable_pub` reflect-gated, both
  redundant with each other) are consolidated into one.
- `bilateral_4ch.py`: `--publish-enable` (new, default **False**) gates
  its own enable-publishing code, which now only runs for a genuinely
  standalone launch with no bridge present. In the normal, supervised
  configuration (bridge + `bilateral_4ch.py` co-launched), it publishes
  NOTHING to either `enable` topic — the bridge's own unconditional
  publish already closes the 2026-09-07 precondition gap, for C1/C4
  exactly as it did for C2/reflect.
- `preflight_ownership_check.py`'s `KNOWN_UNSAFE_COMBINATIONS` check keeps
  detecting the old (now non-default) configuration as a hard failure, as
  defense in depth against a future regression or a manually-added
  `--publish-enable` flag.

## Invariant achieved

- Exactly one final `enable` publisher per device (the bridge,
  unconditional, deadman/FAULT-gated) — **verified** by
  `preflight_ownership_check.py` against source and (separately) the
  no-bridge-running baseline; **not yet verified with the bridge actually
  running**, since that would require launching it (deferred to the
  first supervised on-site session, per H2's own preflight step).
- Exactly one final target publisher per device: already true
  (`bridge_master_slave.py` owns both; `bilateral_4ch.py`'s own
  `joint_target` output is a SEPARATE topic the controller ignores in
  Cartesian mode — not a competing publisher on the SAME topic).
- Exactly one final Cartesian-force/torque publisher per device: already
  true structurally — `force_cmd` (gravity comp, bridge-owned) and
  `joint_torque_ext_cmd` (C1/C4 relay, `bilateral_4ch.py`-owned) are
  DIFFERENT topics, summed together INSIDE `touch_slave.cpp`, not
  competing on one topic.
- All components explicitly summed before final clamping: already true
  for force (see Context); now also true for `enable` in the sense that
  there is exactly one decision, not two independently-arrived-at ones.
- No last-writer-wins command behavior: the `enable` conflict (the only
  one that existed) is closed by construction (single publisher), not by
  hoping the last write happens to be the right one.
