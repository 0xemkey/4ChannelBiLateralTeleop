#!/usr/bin/env python3
"""Read-only live joint-angle monitor for manual matched-pose calibration.

Subscribes to /touch_left/joint_angles and /touch_right/joint_angles
(both plain geometry_msgs/Vector3Stamped, x=waist y=shoulder z=elbow,
radians) and prints a refreshing table of q_master, q_slave, and their
difference. Publishes nothing, commands nothing -- pure observation,
safe to run while both devices are held zero-force.

Usage (from a sourced ROS2 workspace):
    python3 live_joint_watch.py
Ctrl+C to stop.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

JOINT_NAMES = ["waist", "shoulder", "elbow"]


class JointWatch(Node):
    def __init__(self):
        super().__init__("live_joint_watch")
        self.q_master = None
        self.q_slave = None
        self.create_subscription(Vector3Stamped, "/touch_left/joint_angles",
                                  self._on_master, 10)
        self.create_subscription(Vector3Stamped, "/touch_right/joint_angles",
                                  self._on_slave, 10)
        self.create_timer(0.2, self._print)

    def _on_master(self, msg):
        self.q_master = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _on_slave(self, msg):
        self.q_slave = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _print(self):
        if self.q_master is None or self.q_slave is None:
            print("waiting for both /touch_left/joint_angles and /touch_right/joint_angles ...")
            return
        m, s = self.q_master, self.q_slave
        dq = tuple(m[i] - s[i] for i in range(3))
        sys.stdout.write("\033[H\033[J")  # clear screen
        print("LIVE JOINT ANGLES (rad)  -- read-only, no commands published -- Ctrl+C to stop\n")
        print(f"{'':10s} {'waist':>10s} {'shoulder':>10s} {'elbow':>10s}")
        print(f"{'master':10s} {m[0]:10.4f} {m[1]:10.4f} {m[2]:10.4f}")
        print(f"{'slave':10s} {s[0]:10.4f} {s[1]:10.4f} {s[2]:10.4f}")
        print(f"{'delta(m-s)':10s} {dq[0]:10.4f} {dq[1]:10.4f} {dq[2]:10.4f}")
        print(f"\n{'':10s} {'':>10s} {'':>10s} {'':>10s}  (deg, for reference)")
        print(f"{'delta(deg)':10s} {math.degrees(dq[0]):10.2f} {math.degrees(dq[1]):10.2f} {math.degrees(dq[2]):10.2f}")
        print(f"\n||delta_q|| = {math.sqrt(sum(v*v for v in dq)):.4f} rad")


def main():
    rclpy.init()
    node = JointWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
