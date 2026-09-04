#!/usr/bin/env python3
"""Print one anchor from the bridge's calibration file as "x y z" (metres).

Written by bridge_master_slave.py's _write_calibration() once the bridge has
captured that side's anchor. Used by run_master_left_sim.sh /
run_slave_right_sim.sh to seed a sim twin's initial joint state from the
exact snapshot the bridge is using, rather than re-sampling the live pose
topic a second time and risking a slightly different (racy/stale) reading.

Given the master -> bridge -> slave launch order this project uses, by the
time a sim twin starts, the bridge has already captured that side's anchor,
so this should normally succeed on the first read. --timeout/--poll-s exist
for the case where the calibration file is still being written to (e.g. the
other side's anchor arrives a moment later) or does not exist yet.

On timeout/missing field, exits non-zero and prints nothing to stdout, so
the caller can fall back to reading the live pose topic directly (see
read_current_pose.py), matching the existing frame-alignment fallback
behaviour.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default="~/dob_logs/teleop_calibration.json")
    ap.add_argument("--field", choices=("left_anchor", "right_anchor"), required=True)
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("--poll-s", type=float, default=0.1)
    args = ap.parse_args()

    path = os.path.expanduser(args.file)
    deadline = time.time() + args.timeout
    last_err = "no attempt made"
    while time.time() < deadline:
        try:
            with open(path) as f:
                record = json.load(f)
            value = record.get(args.field)
            if value is not None and len(value) == 3:
                print(f"{value[0]} {value[1]} {value[2]}")
                return 0
            last_err = f"{args.field!r} not yet present in {path}"
        except FileNotFoundError:
            last_err = f"{path} does not exist yet"
        except (json.JSONDecodeError, OSError) as exc:
            # Racing the bridge's atomic replace is possible in principle
            # (open() between unlink and link) but os.replace is atomic on
            # POSIX, so this should be rare; retry rather than fail hard.
            last_err = f"transient read error: {exc}"
        time.sleep(args.poll_s)

    print(f"read_calibration: {last_err} (waited {args.timeout}s)", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
