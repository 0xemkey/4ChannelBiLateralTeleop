#!/usr/bin/env python3
"""Runtime resolution of the CURRENTLY-RUNNING master/slave driver session
(2026-09-10 safety-hardening change, requested before H2 real-hardware
execution).

NEVER hardcode a session ID for an emergency stop. This module:

1. Reads the OS process table (`ps -eo pid,pgid,args`) to find every
   currently-running process whose command line contains
   `__node:=teleop_master_left` (or `teleop_slave_right`) -- this matches
   BOTH the `ros2 run` wrapper process and the actual touch_slave binary
   child it spawns (see sim_common.py's own comment on why `ros2 run`
   leaves two distinct PIDs); both are expected to share ONE process group
   (the wrapper is the `setsid` group leader per
   experiment_launch/common.sh's `start_component()`, and its child
   inherits that pgid by default).
2. Requires ALL matching processes for a given side to agree on exactly
   ONE pgid value. Disagreement (should not happen under normal launch,
   but is exactly the kind of thing this check exists to catch) is a
   resolution failure, not a guess.
3. Scans experiment_runs/teleop_setup/*/pids/{master,slave}.pgid for the
   ONE session directory whose recorded pgid files match BOTH derived live
   pgids exactly. Zero or more than one matching session is a resolution
   failure -- this function NEVER picks "the most likely" or "the most
   recent" session; it only returns a session that is provably the one
   currently running both devices.
4. If --driver-session is given explicitly, this becomes a VERIFICATION
   step instead of a search: the given session's own recorded pgids must
   match the live-derived pgids exactly, or resolution fails.

Fails closed in every ambiguous case: returns (verified=False, reason=...)
rather than a best guess. Callers must treat verified=False as "do not
print/use any specific stop command" -- the whole point of this change is
that a stale, reused, or merely-plausible session ID must never be handed
to an operator as "the" emergency stop target.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parents[3]
RUN_ROOT = REPO_ROOT / "experiment_runs" / "teleop_setup"

NODE_MARKERS = {
    "master": "__node:=teleop_master_left",
    "slave": "__node:=teleop_slave_right",
}


def _ps_table() -> list:
    """Returns a list of (pid:int, pgid:int, args:str) for every live process."""
    r = subprocess.run(["ps", "-eo", "pid,pgid,args"], capture_output=True, text=True, timeout=10)
    rows = []
    for line in r.stdout.splitlines()[1:]:
        parts = line.strip().split(None, 2)
        if len(parts) < 3:
            continue
        pid_s, pgid_s, args = parts
        try:
            rows.append((int(pid_s), int(pgid_s), args))
        except ValueError:
            continue
    return rows


def _derive_live_pgid(side: str, ps_rows: list) -> dict:
    marker = NODE_MARKERS[side]
    matches = [(pid, pgid, args) for pid, pgid, args in ps_rows if marker in args]
    if not matches:
        return {"side": side, "verified": False, "reason": f"no live process found with '{marker}' in its command line"}
    pgids = sorted(set(pgid for _, pgid, _ in matches))
    if len(pgids) != 1:
        return {"side": side, "verified": False,
               "reason": f"ambiguous: {len(matches)} matching process(es) disagree on pgid: {pgids}",
               "matches": [{"pid": p, "pgid": g} for p, g, _ in matches]}
    return {"side": side, "verified": True, "pgid": pgids[0],
           "pids": [p for p, _, _ in matches]}


def _read_pgid_file(session_dir: Path, name: str) -> Optional[int]:
    f = session_dir / "pids" / f"{name}.pgid"
    if not f.exists():
        return None
    try:
        return int(f.read_text().strip())
    except ValueError:
        return None


def resolve(explicit_session: Optional[str] = None) -> dict:
    ps_rows = _ps_table()
    master = _derive_live_pgid("master", ps_rows)
    slave = _derive_live_pgid("slave", ps_rows)

    if not master["verified"] or not slave["verified"]:
        return {"verified": False, "reason": "could not derive a live pgid for one or both sides",
               "master": master, "slave": slave}

    live_master_pgid = master["pgid"]
    live_slave_pgid = slave["pgid"]

    if explicit_session is not None:
        session_dir = RUN_ROOT / explicit_session
        if not session_dir.is_dir():
            return {"verified": False, "reason": f"--driver-session {explicit_session!r} does not exist under {RUN_ROOT}",
                   "master": master, "slave": slave}
        rec_master = _read_pgid_file(session_dir, "master")
        rec_slave = _read_pgid_file(session_dir, "slave")
        if rec_master != live_master_pgid or rec_slave != live_slave_pgid:
            return {"verified": False,
                   "reason": (f"--driver-session {explicit_session!r} recorded pgids "
                              f"(master={rec_master}, slave={rec_slave}) do NOT match the live "
                              f"pgids (master={live_master_pgid}, slave={live_slave_pgid}) -- "
                              "refusing to trust an unverified override"),
                   "master": master, "slave": slave}
        return {"verified": True, "session_id": explicit_session, "session_dir": str(session_dir),
               "live_master_pgid": live_master_pgid, "live_slave_pgid": live_slave_pgid,
               "source": "explicit --driver-session, verified against live pgids"}

    if not RUN_ROOT.is_dir():
        return {"verified": False, "reason": f"{RUN_ROOT} does not exist", "master": master, "slave": slave}

    candidates = []
    for session_dir in sorted(RUN_ROOT.iterdir()):
        if not session_dir.is_dir():
            continue
        rec_master = _read_pgid_file(session_dir, "master")
        rec_slave = _read_pgid_file(session_dir, "slave")
        if rec_master == live_master_pgid and rec_slave == live_slave_pgid:
            candidates.append(session_dir.name)

    if len(candidates) == 0:
        return {"verified": False,
               "reason": (f"no session directory under {RUN_ROOT} has recorded pgids matching the "
                          f"live pgids (master={live_master_pgid}, slave={live_slave_pgid}) -- the "
                          "devices may have been started outside this bookkeeping convention"),
               "master": master, "slave": slave}
    if len(candidates) > 1:
        return {"verified": False,
               "reason": (f"AMBIGUOUS: {len(candidates)} session directories all have recorded pgids "
                          f"matching the live pgids: {candidates} -- refusing to guess which one is "
                          "authoritative"),
               "master": master, "slave": slave, "candidates": candidates}

    session_id = candidates[0]
    return {"verified": True, "session_id": session_id, "session_dir": str(RUN_ROOT / session_id),
           "live_master_pgid": live_master_pgid, "live_slave_pgid": live_slave_pgid,
           "source": "derived from live PGIDs, single unambiguous session directory match"}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--driver-session", default=None,
                    help="verify this specific session id against live pgids, instead of searching")
    args = ap.parse_args()
    result = resolve(args.driver_session)
    print(json.dumps(result, indent=2))
    sys.exit(0 if result["verified"] else 1)


if __name__ == "__main__":
    main()
