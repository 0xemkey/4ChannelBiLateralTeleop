#!/usr/bin/env python3
"""hardware_validation/safe_stop.py (2026-09-10, item 8) -- the session-
specific emergency stop. Wraps experiment_launch/99_stop_setup.sh's own
PID-group-based mechanism with explicit session-ID input (NEVER a
hardcoded/reused PID) and a raw fallback path.

Usage:
    python3 safe_stop.py --session <SESSION_ID> [--dry-run]

Prints the exact commands it is about to run before running them. With
--dry-run (or no --session), prints what it WOULD do and exits 0 without
touching anything -- this default matters: this script must be safe to
run accidentally.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
STOP_SCRIPT = (REPO_ROOT / "projects" / "teleop_left_master_right_slave"
              / "experiment_launch" / "99_stop_setup.sh")
RUN_ROOT = REPO_ROOT / "experiment_runs" / "teleop_setup"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", default=None,
                    help="the session ID to stop (REQUIRED to actually act -- "
                         "see experiment_runs/teleop_setup/<session>/pids/*.pgid "
                         "for what this will target). Never a hardcoded number.")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if args.session is None:
        print("[safe_stop] no --session given -- DRY RUN ONLY, printing what a real "
              "call needs, doing nothing.")
        print(f"[safe_stop] would run: bash {STOP_SCRIPT} --session <SESSION_ID>")
        print("[safe_stop] find the current session ID from the launcher that started "
              "it, or from experiment_launch/.current_session")
        sys.exit(0)

    session_dir = RUN_ROOT / args.session
    pids_dir = session_dir / "pids"

    cmd = [str(STOP_SCRIPT), "--session", args.session]
    if args.dry_run:
        cmd.append("--dry-run")

    print(f"[safe_stop] session_dir: {session_dir}")
    if pids_dir.exists():
        for f in sorted(pids_dir.glob("*.pgid")):
            print(f"[safe_stop]   {f.name}: {f.read_text().strip()}")
    else:
        print(f"[safe_stop] WARNING: {pids_dir} does not exist -- this session may "
              "never have been started, or the ID is wrong. Proceeding anyway (the "
              "underlying script no-ops safely if there is nothing to stop).")

    print(f"[safe_stop] running: {' '.join(cmd)}")
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    print(r.stdout)
    if r.stderr:
        print(r.stderr, file=sys.stderr)
    sys.exit(r.returncode)


if __name__ == "__main__":
    main()
