#!/usr/bin/env python3
"""Live terminal display of BilateralStatus -- the direct answer to "is this
actually 4-channel, or 2-channel, or unilateral?" (plan Sec 3.5).

Read-only. Subscribes to /bilateral_status (bilateral_4ch.py's output) and
redraws in place at ~5 Hz, same technique as tools/watch_sweep_status.sh and
contact_annotate.py's status line: home cursor + clear, rather than scrolling,
so the numbers stay readable during a long session.

Usage:
    python3 monitor_channels.py [--topic /bilateral_status]
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from touch_haptics.msg import BilateralStatus

CHANNEL_NAMES = ("C1 slave->master(F)", "C2 slave->master(x)",
                  "C3 master->slave(x)", "C4 master->slave(F)")
REASON_NAMES = ("OK", "ZERO_WEIGHT", "STALE", "INVALID", "BORROWED_REFUSED",
                "TANK_EMPTY", "TRIPPED", "CLUTCHED")
SAFETY_NAMES = ("NOMINAL", "DERATED", "TRIPPED")


class ChannelMonitor(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("monitor_channels")
        self.args = args
        self.latest: Optional[BilateralStatus] = None
        self.last_msg_time = 0.0
        self.start_time = time.time()
        self.create_subscription(BilateralStatus, args.topic, self._cb, 10)
        self.timer = self.create_timer(1.0 / max(args.rate_hz, 1.0), self._draw)

    def _cb(self, msg: BilateralStatus) -> None:
        self.latest = msg
        self.last_msg_time = time.time()

    def _draw(self) -> None:
        lines = []
        lines.append("=== 4-channel bilateral status ===")

        if self.latest is None:
            waited = time.time() - self.start_time
            lines.append(f"waiting {waited:.0f}s for {self.args.topic} ...")
            lines.append("(is bilateral_4ch.py running?)")
            self._render(lines)
            return

        m = self.latest
        age = time.time() - self.last_msg_time
        stale_flag = "  [STALE]" if age > 0.5 else ""
        lines.append(f"architecture: {m.architecture}{stale_flag}   safety: {SAFETY_NAMES[m.safety_state] if m.safety_state < 3 else m.safety_state}")
        lines.append("")

        for i in range(4):
            active = "ACTIVE" if m.channel_active[i] else "off   "
            reason = REASON_NAMES[m.channel_reason[i]] if m.channel_reason[i] < len(REASON_NAMES) else str(m.channel_reason[i])
            bar_len = int(round(min(max(m.channel_weight[i], 0.0), 1.0) * 20))
            bar = "#" * bar_len + "." * (20 - bar_len)
            lines.append(
                f"  {CHANNEL_NAMES[i]:<22} [{bar}] w={m.channel_weight[i]:4.2f}  {active}  ({reason})"
            )

        lines.append("")
        lines.append(
            f"  tau_e_hat (slave->master) = "
            f"[{m.tau_e_hat_nm[0]:+.4f} {m.tau_e_hat_nm[1]:+.4f} {m.tau_e_hat_nm[2]:+.4f}] N*m  "
            f"age={m.tau_e_age_s:.2f}s"
        )
        lines.append(
            f"  tau_h_hat (master->slave) = "
            f"[{m.tau_h_hat_nm[0]:+.4f} {m.tau_h_hat_nm[1]:+.4f} {m.tau_h_hat_nm[2]:+.4f}] N*m  "
            f"age={m.tau_h_age_s:.2f}s"
        )
        lines.append("")
        lines.append(
            f"  master_torque_cmd (C1 contribution) = "
            f"[{m.master_torque_cmd_nm[0]:+.4f} {m.master_torque_cmd_nm[1]:+.4f} {m.master_torque_cmd_nm[2]:+.4f}] N*m"
        )
        lines.append(
            f"  slave_torque_cmd  (C4 contribution) = "
            f"[{m.slave_torque_cmd_nm[0]:+.4f} {m.slave_torque_cmd_nm[1]:+.4f} {m.slave_torque_cmd_nm[2]:+.4f}] N*m"
        )
        lines.append("")
        budget_bar_m = int(round(min(max(m.master_budget_used, 0.0), 1.0) * 20))
        budget_bar_s = int(round(min(max(m.slave_budget_used, 0.0), 1.0) * 20))
        lines.append(
            f"  master budget [{'#'*budget_bar_m}{'.'*(20-budget_bar_m)}] {m.master_budget_used*100:5.1f}%   "
            f"slave budget [{'#'*budget_bar_s}{'.'*(20-budget_bar_s)}] {m.slave_budget_used*100:5.1f}%"
        )
        tank_bar = int(round(min(max(m.passivity_energy_j / max(self.args.tank_capacity_j, 1e-6), 0.0), 1.0) * 20))
        tank_flag = "  <<< EMPTY" if m.passivity_energy_j <= 1e-6 else ""
        lines.append(
            f"  energy tank   [{'#'*tank_bar}{'.'*(20-tank_bar)}] {m.passivity_energy_j*1000:5.1f} mJ{tank_flag}"
        )
        lines.append(f"  duty cycle (60s mean) = {m.duty_cycle_nm:.4f} N*m")
        lines.append("")
        flags = []
        if m.clutch_active:
            flags.append("CLUTCH")
        if m.grip_detected:
            flags.append("GRIP")
        if m.safety_state == 2:
            flags.append("*** TRIPPED -- ros2 param set /bilateral_4ch manual_rearm true ***")
        if flags:
            lines.append("  " + "  ".join(flags))

        self._render(lines)

    def _render(self, lines) -> None:
        sys.stdout.write("\033[H\033[J")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--topic", default="/bilateral_status")
    p.add_argument("--rate-hz", type=float, default=5.0)
    p.add_argument("--tank-capacity-j", type=float, default=0.05,
                   help="For the tank bar's full-scale reference. Must match "
                        "bilateral_4ch.py's --tank-capacity-j to read correctly.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ChannelMonitor(args)
    sys.stdout.write("\033[?25l")  # hide cursor
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        sys.stdout.write("\033[?25h\n")  # restore cursor
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
