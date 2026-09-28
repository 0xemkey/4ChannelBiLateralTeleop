"""Canonical frozen parameter table -- THE single authoritative source for
this project's hardware-validation parameters (2026-09-10, item 7:
"complete canonical configuration adoption" -- this file, not a helper
alongside other copies, is what launchers/stage runners/recorder metadata/
safety checks/tests must all import from).

MOVED 2026-09-10 from hardware_experiments/2026-09-09_.../frozen_config.py
to this directory (hardware_validation/) as the new canonical home -- the
old path now re-exports from here for backward compatibility with
already-written tests, rather than duplicating the table.

SOURCE OF TRUTH: this project's own checked-in, actually-reproducible
script defaults (run_bridge.sh, run_master_left.sh, run_slave_right.sh),
NOT narrative mentions in docs of a one-off value tried live in a specific
diagnostic session. Two real discrepancies were found and are documented
below rather than silently resolved:

1. LEFT_MAX_OFFSET_M: run_bridge.sh's own persisted default is 0.08.
   FOUR_CHANNEL_TODO.md narrates "raised to 0.15m" during a 2026-09-03 C2
   diagnostic session, but that value was NEVER written back into
   run_bridge.sh's default -- it only appears as an explicit override in
   03b_start_bridge_c2_diagnostic.sh (which actually uses 0.05, a THIRD,
   even smaller value, for that specific diagnostic). Three different
   numbers exist across this project's own history for the same parameter.
   CANONICAL HERE: 0.08 (the actually-persisted, reproducible default).
   0.15 and 0.05 are NOT used unless explicitly overridden with a stated
   reason.

2. LEFT_GRAVITY_COMP_N: run_bridge.sh's own comment is explicit --
   "Off by default (0.0) -- unlike SLAVE_GRAVITY_COMP_N above, this has
   NOT been through a controlled sweep yet, so there is no validated value
   to default to." The earlier version of this task's PREP_REPORT.md
   incorrectly called 0.45 (from 03b_start_bridge_c2_diagnostic.sh, a
   "working value found live" per FOUR_CHANNEL_TODO.md) a "validated"
   number -- it is not; it never went through the sweep SLAVE_GRAVITY_
   COMP_N=0.55 did. CANONICAL HERE: 0.0 (matches the actually-persisted,
   swept-and-justified default). Using 0.0 also means Part 1's "master
   gravity residual" measurement observes the master's TRUE uncompensated
   sag, which is arguably the more scientifically useful baseline anyway.

Every other value below matches run_bridge.sh's own persisted defaults
exactly (SLAVE_GRAVITY_COMP_N=0.55 IS a controlled-sweep-validated value,
documented in run_bridge.sh's own comment with the actual reach-ratio
numbers from that sweep).
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from typing import Optional


@dataclass(frozen=True)
class FrozenConfig:
    # Master (left) controller.
    master_stiffness_n_per_m: float = 15.0
    master_damping_n_s_per_m: float = 0.10
    master_max_force_n: float = 2.0
    master_max_position_error_m: float = 0.03

    # Slave (right) controller.
    slave_stiffness_n_per_m: float = 35.0
    slave_damping_n_s_per_m: float = 0.20
    slave_max_force_n: float = 1.5
    slave_max_position_error_m: float = 0.04

    # Gravity compensation.
    slave_gravity_comp_n: float = 0.55   # controlled-sweep-validated (run_bridge.sh)
    slave_gravity_axis: int = 1
    left_gravity_comp_n: float = 0.0     # NOT swept -- see module docstring item 2
    left_gravity_axis: int = 1

    # Workspace / travel limits.
    axis_map: str = "x,y,z"
    axis_scale: str = "1.0,1.0,1.0"
    max_offset_xyz: str = "0.12,0.19,0.15"   # slave (C3), run_bridge.sh
    max_offset_mode: str = "axis"
    left_max_offset_m: float = 0.08          # master (C2) -- see module docstring item 1
    max_speed_mps: float = 0.5
    deadband_m: float = 0.0002

    # Force-budget planner (bilateral coupling, Parts 4+ only).
    force_budget_reserve_n: float = 0.10
    force_budget_telemetry_stale_s: float = 0.2
    force_budget_stall_timeout_s: float = 3.0

    # Startup alignment / gravity settle.
    alignment_ramp_s: float = 1.0
    gravity_settle_velocity_threshold_mps: float = 0.005  # SIM-derived, see
                                                            # SIM_GRAVITY_SETTLE_VELOCITY_THRESHOLD_MPS
                                                            # in startup_alignment.py -- NOT
                                                            # hardware-validated yet
    gravity_settle_hold_s: float = 0.15
    gravity_settle_min_time_s: float = 0.1
    gravity_settle_timeout_s: float = 10.0

    rate_hz: float = 100.0

    # 2026-09-10 (item 7): master deadman -- these are the DEFAULTS a
    # canonical hardware session should use; require_master_deadman stays
    # False here (matches bridge_master_slave.py's own opt-in default) so
    # importing this module never silently changes behavior -- callers that
    # WANT the deadman active must still pass it explicitly (H2 onward, per
    # ONSITE_RUNBOOK.md).
    require_master_deadman: bool = False
    master_deadman_button_mask: int = 1
    master_deadman_stale_s: float = 0.2

    # 2026-09-10 (item 4): channel gates. Safe default: C3 stays True (see
    # its own field's comment in bridge_cli_args for why -- matches this
    # project's pre-2026-09-10 always-on behavior for every EXISTING
    # caller); C1/C2/C4 default False (never silently on).
    c2_enable: bool = False
    c3_enable: bool = True
    c1_enable: bool = False
    c4_enable: bool = False

    # 2026-09-10 (item 7): DOB parameters -- read-only documentation of the
    # touch_slave.cpp defaults (NOT independently re-declared/enforced by
    # this dataclass; touch_slave.cpp's own --dob_torque_f_max_n etc.
    # remain the actual source at runtime). Recorded here so a session
    # manifest can state what was assumed without duplicating logic.
    dob_torque_f_max_n: float = 0.9  # touch_slave.cpp declare_parameter default
    dob_g_dis_rad_s_waist: float = 10.0
    dob_g_dis_rad_s_shoulder: float = 10.0
    dob_g_dis_rad_s_elbow: float = 5.0


FROZEN = FrozenConfig()


def bridge_cli_args(session_calib_path: str, enable_c2: bool = True,
                    gravity_settle_hold_mode: bool = False,
                    c3_enable: Optional[bool] = None,
                    require_master_deadman: Optional[bool] = None,
                    c1_enable: Optional[bool] = None,
                    c4_enable: Optional[bool] = None) -> list:
    """The canonical bridge_master_slave.py argument list, built from
    FROZEN -- every script in this directory should call this rather than
    hand-writing its own arg list, so there is exactly one place these
    numbers can drift out of sync. Every *_enable/deadman arg defaults to
    FROZEN's own field when not explicitly overridden by the caller."""
    c = FROZEN
    if c3_enable is None:
        c3_enable = c.c3_enable
    if require_master_deadman is None:
        require_master_deadman = c.require_master_deadman
    if c1_enable is None:
        c1_enable = c.c1_enable
    if c4_enable is None:
        c4_enable = c.c4_enable
    return [
        "--left-pose-topic", "/touch_left/pose", "--right-pose-topic", "/touch_right/pose",
        "--right-target-topic", "/touch_right/target_pose", "--right-enable-topic", "/touch_right/enable",
        "--left-target-topic", "/touch_left/target_pose", "--left-enable-topic", "/touch_left/enable",
        "--left-telemetry-topic", "/touch_left/cartesian_controller_telemetry",
        "--right-telemetry-topic", "/touch_right/cartesian_controller_telemetry",
        "--slave-force-topic", "/touch_right/force_cmd",
        "--enable-c2", "true" if enable_c2 else "false",
        "--left-max-offset-m", str(c.left_max_offset_m),
        "--calibration-file", session_calib_path,
        "--axis-map", c.axis_map, "--axis-scale", c.axis_scale,
        "--max-offset-xyz", c.max_offset_xyz, "--max-offset-mode", c.max_offset_mode,
        "--max-speed-mps", str(c.max_speed_mps), "--deadband-m", str(c.deadband_m),
        "--slave-gravity-comp-n", str(c.slave_gravity_comp_n), "--slave-gravity-axis", str(c.slave_gravity_axis),
        "--left-gravity-comp-n", str(c.left_gravity_comp_n), "--left-gravity-axis", str(c.left_gravity_axis),
        "--master-stiffness-n-per-m", str(c.master_stiffness_n_per_m),
        "--master-damping-n-s-per-m", str(c.master_damping_n_s_per_m),
        "--master-max-force-n", str(c.master_max_force_n),
        "--master-max-position-error-m", str(c.master_max_position_error_m),
        "--stiffness-n-per-m", str(c.slave_stiffness_n_per_m),
        "--slave-damping-n-s-per-m", str(c.slave_damping_n_s_per_m),
        "--slave-max-force-n", str(c.slave_max_force_n),
        "--slave-max-position-error-m", str(c.slave_max_position_error_m),
        "--force-budget-reserve-n", str(c.force_budget_reserve_n),
        "--force-budget-telemetry-stale-s", str(c.force_budget_telemetry_stale_s),
        "--force-budget-stall-timeout-s", str(c.force_budget_stall_timeout_s),
        "--alignment-ramp-s", str(c.alignment_ramp_s),
        "--gravity-settle-hold-mode", "true" if gravity_settle_hold_mode else "false",
        "--c3-enable", "true" if c3_enable else "false",
        "--c1-enable", "true" if c1_enable else "false",
        "--c4-enable", "true" if c4_enable else "false",
        "--require-master-deadman", "true" if require_master_deadman else "false",
        "--master-deadman-button-mask", str(c.master_deadman_button_mask),
        "--master-deadman-stale-s", str(c.master_deadman_stale_s),
        "--rate-hz", str(int(c.rate_hz)), "--wait-timeout-s", "15",
    ]


def config_fingerprint(cfg: FrozenConfig = FROZEN) -> str:
    """2026-09-10 Part B item 6: a short, stable hash of every field in
    FrozenConfig -- write this into every run's manifest/output directory
    (alongside git rev-parse HEAD and git status --porcelain) so a plot or
    report can be traced back to the EXACT parameters used, not just "the
    frozen config as of whenever this was written." Changes the instant any
    field above changes -- a mismatch between a plot's recorded fingerprint
    and frozen_config.py's CURRENT fingerprint means the config drifted
    since that run, a real signal worth surfacing, not hiding."""
    payload = json.dumps(asdict(cfg), sort_keys=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


def write_manifest(out_dir, extra: dict = None) -> dict:
    """Writes <out_dir>/run_manifest.json with the config fingerprint, the
    full config, and git provenance -- call this at the START of every
    stage script so every output directory is self-describing."""
    import subprocess
    from pathlib import Path

    def _git(*args):
        try:
            return subprocess.run(["git"] + list(args), capture_output=True, text=True,
                                  cwd=Path(__file__).resolve().parents[3], timeout=10).stdout.strip()
        except Exception as exc:
            return f"<git call failed: {exc}>"

    manifest = {
        "config_fingerprint": config_fingerprint(),
        "frozen_config": asdict(FROZEN),
        "git_head": _git("rev-parse", "HEAD"),
        "git_status_porcelain": _git("status", "--porcelain"),
        "git_branch": _git("branch", "--show-current"),
    }
    if extra:
        manifest.update(extra)
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    with open(out_path / "run_manifest.json", "w") as f:
        json.dump(manifest, f, indent=2)
    return manifest
