#!/usr/bin/env python3
"""READ-ONLY master deadman diagnostic (2026-09-10, H2 deadman
investigation).

Subscribes ONLY to /touch_left/buttons (std_msgs/Int32) and prints, for
every observed CHANGE plus a periodic heartbeat, the raw integer value,
its individual bits, whether the configured deadman/clutch masks match,
and what bridge_master_slave.py's OWN pure decision function
(`deadman_gate_state`, imported directly -- not reimplemented here, so
this can never silently drift out of sync with the real bridge logic)
would compute at that instant, including its stateful latch behavior
across the whole run (mirroring `_master_deadman_held()`'s own
stateful wrapper).

ABSOLUTE: creates zero publishers, sends zero commands, arms nothing.
Safe to run at any time, including right now, with real hardware live.

Usage (run this yourself, physically at the device, and press/release
buttons as instructed):

    python3 hardware_validation/watch_master_deadman.py

Then, in order:
  1. Leave all buttons released for a few seconds.
  2. Press and hold one button, then release.
  3. Press and hold a different button (if the device has more than one),
     then release.
  4. Optionally hold a button, then physically disconnect/pause updates
     (or just wait) to observe the stale-timeout -> latch -> requires-
     explicit-rearm behavior described below.

Ctrl-C to stop. Prints a final summary of every distinct raw value seen.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

sys.path.insert(0, str(Path(__file__).parent))
from frozen_config import FROZEN  # noqa: E402

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))
from bridge_master_slave import deadman_gate_state  # noqa: E402 -- REAL bridge logic, not reimplemented

DEFAULT_TOPIC = "/touch_left/buttons"
DEFAULT_CLUTCH_MASK = 2  # bridge_master_slave.py's own --clutch-button-mask default
HEARTBEAT_S = 2.0


def _bits8(v: int) -> str:
    return format(v & 0xFF, "08b")


class DeadmanWatcher(Node):
    def __init__(self, topic: str):
        super().__init__("hardware_validation_watch_master_deadman")
        self.latest_value = None
        self.latest_t = None
        self.first_msg_t = None
        self.msg_count = 0
        self.distinct_values_seen = {}
        self.create_subscription(Int32, topic, self._on_msg, 10)

    def _on_msg(self, msg: Int32) -> None:
        self.latest_value = int(msg.data)
        self.latest_t = time.time()
        if self.first_msg_t is None:
            self.first_msg_t = self.latest_t
        self.msg_count += 1
        self.distinct_values_seen[self.latest_value] = self.distinct_values_seen.get(self.latest_value, 0) + 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument("--deadman-mask", type=int, default=FROZEN.master_deadman_button_mask)
    ap.add_argument("--clutch-mask", type=int, default=DEFAULT_CLUTCH_MASK)
    ap.add_argument("--stale-s", type=float, default=FROZEN.master_deadman_stale_s)
    ap.add_argument("--duration-s", type=float, default=120.0,
                    help="stop automatically after this long (default 120s); Ctrl-C stops earlier")
    args = ap.parse_args()

    print("================ READ-ONLY master deadman diagnostic ================")
    print(f"topic={args.topic} (std_msgs/Int32)")
    print(f"deadman_mask={args.deadman_mask} (0b{args.deadman_mask:08b})  "
          f"clutch_mask={args.clutch_mask} (0b{args.clutch_mask:08b})  stale_s={args.stale_s}")
    print("This script creates ZERO publishers. It sends no commands. Nothing is armed.")
    print("=======================================================================")
    print()
    print("Step 1: leave ALL buttons released for a few seconds now.")
    print("Step 2: press and hold ONE button, then release.")
    print("Step 3: press and hold a DIFFERENT button (if any), then release.")
    print("Watching for changes... (Ctrl-C to stop)")
    print()

    rclpy.init()
    node = DeadmanWatcher(args.topic)

    latched = False
    last_printed_value = None
    last_heartbeat_t = 0.0
    deadline = time.time() + args.duration_s

    def _print_row(now, tag):
        nonlocal latched
        held, new_latched, reason = deadman_gate_state(
            require=True, latched=latched, buttons_t=node.latest_t, now=now,
            stale_s=args.stale_s, buttons_value=(node.latest_value or 0), mask=args.deadman_mask,
        )
        latched = new_latched
        raw = node.latest_value if node.latest_value is not None else 0
        deadman_bit = bool(raw & args.deadman_mask)
        clutch_bit = bool(raw & args.clutch_mask)
        age = (now - node.latest_t) if node.latest_t is not None else None
        age_s = f"{age:.3f}s" if age is not None else "n/a (no message yet)"
        print(f"t={now - (node.first_msg_t or now):8.3f} raw={raw:<4d} bits={_bits8(raw)} "
              f"deadman_bit(mask={args.deadman_mask})={deadman_bit!s:<5} "
              f"clutch_bit(mask={args.clutch_mask})={clutch_bit!s:<5} "
              f"deadman_gate_state()_held={held!s:<5} latched={latched!s:<5} "
              f"age={age_s}"
              f"{'  reason=' + reason if reason else ''}"
              f"  {tag}")

    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.time()
            if node.latest_value != last_printed_value and node.latest_value is not None:
                _print_row(now, "<- CHANGED")
                last_printed_value = node.latest_value
                last_heartbeat_t = now
            elif now - last_heartbeat_t >= HEARTBEAT_S:
                _print_row(now, "(heartbeat)")
                last_heartbeat_t = now
    except KeyboardInterrupt:
        print("\n[watch_master_deadman] stopped by Ctrl-C")
    finally:
        print()
        print("================ summary ================")
        print(f"total messages received: {node.msg_count}")
        print("distinct raw values observed (value: count):")
        for v, c in sorted(node.distinct_values_seen.items()):
            print(f"  raw={v} (0b{v:08b}): {c} message(s)  "
                 f"deadman_bit={bool(v & args.deadman_mask)}  clutch_bit={bool(v & args.clutch_mask)}")
        if not node.distinct_values_seen:
            print("  (no messages received at all -- is teleop_master_left running and "
                 f"publishing on {args.topic}?)")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
