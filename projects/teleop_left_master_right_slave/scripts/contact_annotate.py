#!/usr/bin/env python3
"""SINGLE-SESSION DIAGNOSTIC TOOL -- delete after the separability session.

Not a feature. No GUI, no config, no persistence. It exists so one recorded
teleop session can be split into "in contact" and "not in contact" without
anyone eyeballing an unmarked trace afterwards.

Publishes std_msgs/Bool on /contact_annotation at the same rate the DOB
diagnostics come out (~200 Hz), carrying the CURRENT state continuously rather
than only on edges. Continuous publication is what makes alignment trivial:
every /dob_diagnostics sample has an annotation sample within ~5 ms, so the
analysis never has to interpolate across a long gap or guess.

WHY NOT THE SPARE STYLUS BUTTON
-------------------------------
There is one: touch_slave uses button 1 as deadman and button 2 as clutch, and
the master runs with require_deadman:=false, so mask 1 is genuinely free. It
would have been less new infrastructure, which is what the task's preference
order asks for.

It is still the wrong choice here. Pressing a button on the master stylus
applies force TO THE MASTER -- the exact arm whose disturbance estimate is
being measured. The annotation would inject a torque transient into
tau_dis_hat at precisely the moment it marks, contaminating the boundary
between the two segments this session exists to compare. A keyboard sits
outside the measured mechanical path and cannot do that.

CLOCK
-----
Uses this node's ROS clock via get_clock().now(). touch_slave stamps
/dob_diagnostics from its own node clock the same way. With use_sim_time
false on both, that is one system clock, so timestamps are directly
comparable with no offset. Do not swap this for time.time().

USAGE
  ros2 run ... no -- just:  python3 contact_annotate.py
  Press ENTER to toggle between CONTACT and CLEAR. Ctrl-C to stop.

  Toggle-on-Enter rather than hold-spacebar deliberately: holding a key needs
  raw terminal mode, which breaks Ctrl-C handling and silently drops keys when
  the terminal loses focus. A toggle is unambiguous, and the current state is
  printed after every change so it cannot drift unnoticed.
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ContactAnnotator(Node):
    def __init__(self) -> None:
        super().__init__("contact_annotator")
        self.pub = self.create_publisher(Bool, "/contact_annotation", 10)
        self.state = False
        self.lock = threading.Lock()
        # 200 Hz, matching touch_slave's publish_hz so the two streams
        # interleave one-for-one in the bag.
        self.create_timer(1.0 / 200.0, self._tick)

    def _tick(self) -> None:
        msg = Bool()
        with self.lock:
            msg.data = self.state
        self.pub.publish(msg)

    def toggle(self) -> bool:
        with self.lock:
            self.state = not self.state
            return self.state


def main() -> None:
    rclpy.init()
    node = ContactAnnotator()

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    print("=" * 58)
    print(" contact annotation  ->  /contact_annotation  (200 Hz)")
    print("=" * 58)
    print(" ENTER toggles.  Ctrl-C stops.")
    print()
    print(" Press ENTER the instant contact STARTS, and again the")
    print(" instant it ENDS. Do not touch the master stylus to do it --")
    print(" that would inject force into the signal being measured.")
    print()
    print("   state: CLEAR")
    try:
        while True:
            sys.stdin.readline()
            on = node.toggle()
            print(f"   state: {'CONTACT' if on else 'CLEAR'}")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        # Leave the stream in a defined state rather than whatever the last
        # toggle happened to be -- a bag ending mid-CONTACT would otherwise
        # label the trailing samples as loaded when the session had stopped.
        with node.lock:
            node.state = False
        m = Bool()
        m.data = False
        for _ in range(20):
            node.pub.publish(m)
        print("\n   state: CLEAR (released on exit)")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
