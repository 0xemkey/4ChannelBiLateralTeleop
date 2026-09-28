#!/usr/bin/env python3
"""hardware_validation/preflight.py (2026-09-10, item 8/14) -- canonical
preflight check. Read-only: queries the live ROS graph, git state, and the
canonical frozen config. Commands NOTHING.

Distinguishes the THREE preflight modes item 14 requires -- do not collapse
them into one PASS/FAIL:

  IDLE_TOPOLOGY_PASS       -- no bridge (or bilateral_4ch) running at all.
                              A clean idle state. NOT proof anything is
                              armed correctly -- there is nothing to check
                              yet.
  STAGE_LAUNCHED_UNARMED   -- bridge process found, but bridge_state is
                              READY (never armed) or the /bridge_diagnostics
                              topic itself is not yet publishing.
  ARMED_TOPOLOGY_ELIGIBLE  -- bridge found, publishing diagnostics, AND
                              ownership checks pass -- eligible to consider
                              arming, not proof that arming WILL succeed
                              (deadman freshness etc. are checked separately
                              at arm time by the bridge itself).

A PASS with no bridge running must NEVER be read as "armed ownership is
valid" -- see the mode label printed.

BUG FIX (2026-09-10, discovered while building the H2 pre-arm/post-disarm
verification tooling): every node-name match in this file previously
checked for the substring "bridge_master_slave", but the bridge's actual
ROS node name is "touch_master_slave_bridge" (bridge_master_slave.py:
`super().__init__("touch_master_slave_bridge")`) -- different word order,
never a substring match. This meant `bridge_present` was ALWAYS False
even while the bridge was actively running (so this script could report
IDLE_TOPOLOGY_PASS while armed), the per-topic ownership check would
ALWAYS flag the bridge's own legitimate publishes as "unexpected", and
UNSAFE_COMBOS's bridge+bilateral_4ch dual-publisher check could never
fire. Read-only diagnostic tooling only -- no controller behavior or
threshold changed by this fix.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).parent))
from frozen_config import FROZEN, config_fingerprint  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]

EXPECTED_OWNERS = {
    "/touch_left/target_pose": (["touch_master_slave_bridge"], "master target (C2 only)"),
    "/touch_left/enable": (["touch_master_slave_bridge"], "master enable"),
    "/touch_right/target_pose": (["touch_master_slave_bridge"], "slave target (C3)"),
    "/touch_right/enable": (["touch_master_slave_bridge"], "slave enable"),
    "/touch_right/force_cmd": (["touch_master_slave_bridge"], "slave force (gravity comp)"),
    "/touch_left/buttons": (["teleop_master_left"], "deadman/clutch button source"),
    "/bridge_diagnostics": (["touch_master_slave_bridge"], "bridge diagnostics"),
}

# 2026-09-10 command-authority ADR: bilateral_4ch.py must NEVER publish
# enable when co-launched with the bridge (--publish-enable defaults
# False now) -- if it's found here anyway, that IS the known-unsafe
# combination this preflight must catch.
UNSAFE_COMBOS = [
    (["touch_master_slave_bridge", "bilateral_4ch"], "/touch_left/enable"),
    (["touch_master_slave_bridge", "bilateral_4ch"], "/touch_right/enable"),
]


def _match(node_name, allowed):
    return any(s in node_name for s in allowed)


def _git(*args):
    try:
        return subprocess.run(["git"] + list(args), capture_output=True, text=True,
                              cwd=REPO_ROOT, timeout=10).stdout.strip()
    except Exception as exc:
        return f"<git call failed: {exc}>"


class Preflight(Node):
    def __init__(self):
        super().__init__("hardware_validation_preflight")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--require-armed-eligible", action="store_true",
                    help="exit nonzero unless mode is ARMED_TOPOLOGY_ELIGIBLE")
    args = ap.parse_args()

    print("================ hardware_validation PREFLIGHT (read-only) ================")
    print(f"git HEAD:            {_git('rev-parse', 'HEAD')}")
    print(f"git branch:          {_git('branch', '--show-current')}")
    dirty = _git("status", "--porcelain")
    print(f"dirty worktree:      {len(dirty.splitlines()) if dirty else 0} file(s) changed")
    print(f"config fingerprint:  {config_fingerprint()}")

    rclpy.init()
    node = Preflight()
    deadline = time.time() + 2.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node_names = [n for n, _ in node.get_node_names_and_namespaces()]
    bridge_present = any("touch_master_slave_bridge" in n for n in node_names)
    bilateral_present = any("bilateral_4ch" in n for n in node_names)
    print(f"\nlive nodes seen:     {node_names}")

    failures = []
    warnings = []
    for topic, (allowed, label) in EXPECTED_OWNERS.items():
        try:
            infos = node.get_publishers_info_by_topic(topic)
        except Exception as exc:
            print(f"  {topic:45s} ERROR: {exc}")
            continue
        pubs = sorted({i.node_name for i in infos})
        print(f"  {topic:45s} ({label}): publishers={pubs or '(none)'}")
        for pn in pubs:
            if allowed and not _match(pn, allowed):
                failures.append(f"{topic}: unexpected publisher '{pn}'")
        if len(pubs) > 1:
            warnings.append(f"{topic}: multiple publishers: {pubs}")

    for combo, topic in UNSAFE_COMBOS:
        try:
            infos = node.get_publishers_info_by_topic(topic)
        except Exception:
            continue
        pubs = {i.node_name for i in infos}
        matched = [n for n in combo if any(n in pn for pn in pubs)]
        if len(matched) >= 2:
            failures.append(f"{topic}: KNOWN-UNSAFE COMBINATION -- {' and '.join(matched)} both publishing")

    # Diagnostics freshness -> mode determination.
    diag_fresh = False
    try:
        infos = node.get_publishers_info_by_topic("/bridge_diagnostics")
        diag_fresh = len(infos) > 0
    except Exception:
        pass

    node.destroy_node()
    rclpy.shutdown()

    if not bridge_present:
        mode = "IDLE_TOPOLOGY_PASS"
    elif not diag_fresh:
        mode = "STAGE_LAUNCHED_UNARMED"
    elif failures:
        mode = "STAGE_LAUNCHED_UNARMED (ownership FAILURES present)"
    else:
        mode = "ARMED_TOPOLOGY_ELIGIBLE"

    print(f"\ntouch_master_slave_bridge present: {bridge_present}")
    print(f"bilateral_4ch present:       {bilateral_present}")
    print(f"\nPREFLIGHT MODE: {mode}")
    if mode == "IDLE_TOPOLOGY_PASS":
        print("  (no bridge running -- this is NOT proof armed ownership is valid, "
              "only that nothing unexpected is currently live)")

    if warnings:
        print("\nWARNINGS:")
        for w in warnings:
            print(f"  - {w}")
    if failures:
        print("\nFAILURES:")
        for f in failures:
            print(f"  - {f}")
        print("\nPREFLIGHT: FAIL")
        sys.exit(1)

    if args.require_armed_eligible and mode != "ARMED_TOPOLOGY_ELIGIBLE":
        print(f"\n--require-armed-eligible was set but mode is {mode}")
        sys.exit(2)

    print("\nPREFLIGHT: PASS")
    sys.exit(0)


if __name__ == "__main__":
    main()
