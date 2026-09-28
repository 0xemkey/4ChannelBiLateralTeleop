#!/usr/bin/env python3
"""Part F, item 18 (2026-09-10): read-only ROS-graph publisher-ownership
preflight check. Queries the LIVE ROS graph (rclpy's own
get_publishers_info_by_topic -- no subscription, no publish, no command of
any kind) and reports, for each command topic, exactly which node(s)
publish to it -- failing if an unexpected publisher is present.

Safe to run at any time, including against currently-running real hardware
nodes -- this script commands nothing.

Usage:
    python3 preflight_ownership_check.py [--enable-c2] [--c1-c4-expected]
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node

# topic -> (list of node names allowed to publish here, human label)
# "bridge_master_slave" and "bilateral_4ch" are matched as SUBSTRINGS of
# the actual node name (which includes a launch-specific suffix in some
# configurations) -- see _match().
EXPECTED_OWNERS = {
    "/touch_left/target_pose": (["bridge_master_slave"], "master target (C2 only)"),
    "/touch_left/enable": (["bridge_master_slave", "bilateral_4ch"], "master enable"),
    "/touch_right/target_pose": (["bridge_master_slave"], "slave target (C3, always)"),
    "/touch_right/enable": (["bridge_master_slave", "bilateral_4ch"], "slave enable"),
    "/touch_right/force_cmd": (["bridge_master_slave"], "slave force (gravity comp)"),
    "/touch_left/buttons": (["teleop_master_left"], "deadman/clutch button source"),
    "/bridge_diagnostics": (["bridge_master_slave"], "bridge diagnostics"),
    "/touch_left/cartesian_controller_telemetry": (["teleop_master_left"], "master telemetry"),
    "/touch_right/cartesian_controller_telemetry": (["teleop_slave_right"], "slave telemetry"),
}

# 2026-09-10 finding (CHANNEL_READINESS_MATRIX.md): bilateral_4ch.py's own
# enable publishers are COMPLETELY INDEPENDENT of the bridge's arm/FAULT/
# deadman state -- if BOTH bridge_master_slave AND bilateral_4ch are found
# publishing to the SAME enable topic simultaneously, that is flagged as a
# HARD FAIL regardless of EXPECTED_OWNERS allowing both names individually,
# because the two together is exactly the known-unsafe combination.
KNOWN_UNSAFE_COMBINATIONS = [
    (["bridge_master_slave", "bilateral_4ch"], "/touch_left/enable"),
    (["bridge_master_slave", "bilateral_4ch"], "/touch_right/enable"),
]


def _match(node_name: str, allowed_substrings) -> bool:
    return any(s in node_name for s in allowed_substrings)


class OwnershipCheck(Node):
    def __init__(self):
        super().__init__("preflight_ownership_check")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topics", nargs="*", default=list(EXPECTED_OWNERS.keys()))
    args = ap.parse_args()

    rclpy.init()
    node = OwnershipCheck()
    # Let discovery settle.
    import time
    deadline = time.time() + 2.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    failures = []
    warnings = []
    print("================ PREFLIGHT OWNERSHIP CHECK (read-only) ================")
    for topic in args.topics:
        allowed, label = EXPECTED_OWNERS.get(topic, ([], topic))
        try:
            infos = node.get_publishers_info_by_topic(topic)
        except Exception as exc:
            print(f"  {topic:50s} ERROR querying: {exc}")
            continue
        publisher_nodes = sorted({i.node_name for i in infos})
        print(f"  {topic:50s} ({label}): publishers={publisher_nodes or '(none)'}")
        for pn in publisher_nodes:
            if allowed and not _match(pn, allowed):
                failures.append(f"{topic}: unexpected publisher node '{pn}' "
                                f"(expected one of {allowed})")
        if len(publisher_nodes) > 1:
            warnings.append(f"{topic}: MULTIPLE publisher nodes present: {publisher_nodes}")

    for combo_names, topic in KNOWN_UNSAFE_COMBINATIONS:
        try:
            infos = node.get_publishers_info_by_topic(topic)
        except Exception:
            continue
        publisher_nodes = {i.node_name for i in infos}
        matched = [n for n in combo_names if any(n in pn for pn in publisher_nodes)]
        if len(matched) >= 2:
            failures.append(
                f"{topic}: KNOWN-UNSAFE COMBINATION present -- both "
                f"{' and '.join(matched)} are publishing here simultaneously "
                f"(bilateral_4ch's enable logic is independent of the bridge's "
                f"arm/FAULT/deadman state -- see CHANNEL_READINESS_MATRIX.md)"
            )

    node.destroy_node()
    rclpy.shutdown()

    print("=========================================================================")
    if warnings:
        print("WARNINGS:")
        for w in warnings:
            print(f"  - {w}")
    if failures:
        print("FAILURES (preflight FAILS):")
        for f in failures:
            print(f"  - {f}")
        print("\nPREFLIGHT: FAIL")
        sys.exit(1)
    print("\nPREFLIGHT: PASS (no unexpected publishers found)")
    sys.exit(0)


if __name__ == "__main__":
    main()
