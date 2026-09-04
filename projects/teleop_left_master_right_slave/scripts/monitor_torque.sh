#!/usr/bin/env bash
set -euo pipefail

# Prints the torque-related telemetry for one Touch device.
#
# IMPORTANT, read before trusting these numbers:
#
#   The Phantom Omni / Touch has optical encoders on its joints and NO force or
#   torque sensor. Every value below is a COMMANDED quantity read back from the
#   OpenHaptics driver, not a measurement of what the arm is actually feeling.
#   OpenHaptics lists HD_CURRENT_JOINT_TORQUE under both its get and its set
#   parameters, which is the giveaway: it is the command channel to the three
#   base motors.
#
#   joint_torque       what the driver is telling the 3 base motors to produce
#   gimbal_torque      zero on this hardware; the Omni/Touch gimbal is passive
#   applied_force      the Cartesian force command, after clamping
#   tracking_error     target minus actual position, in meters
#   motor_temperature  driver thermal estimate, normalized 0..1
#
#   tracking_error is the only signal here that responds to the outside world.
#   If the slave is commanded to a target and cannot reach it, something is
#   resisting it, and stiffness * tracking_error estimates that load. It is an
#   inference from a commanded spring, not a reading, and it cannot separate
#   your hand from friction, gravity, or the arm hitting its workspace limit.

SIDE="${SIDE:-right}"
MODE="${MODE:-summary}"
RATE_HZ="${RATE_HZ:-2}"
STIFFNESS_N_PER_M="${STIFFNESS_N_PER_M:-35.0}"

if [[ "${SIDE}" != "left" && "${SIDE}" != "right" ]]; then
  echo "[torque_monitor][error] SIDE must be left or right, got ${SIDE}" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

PREFIX="/touch_${SIDE}"

if ! ros2 topic info "${PREFIX}/joint_torque" 2>/dev/null | grep -Eq "Publisher count: [1-9]"; then
  echo "[torque_monitor][error] no publisher on ${PREFIX}/joint_torque" >&2
  echo "[torque_monitor][hint] start the ${SIDE} device with PUBLISH_TELEMETRY=true" >&2
  echo "[torque_monitor][hint] telemetry is off by default, so an older node will not publish it" >&2
  exit 1
fi

case "${MODE}" in
  raw)
    echo "[torque_monitor] echoing ${PREFIX}/joint_torque (commanded, not measured)"
    exec ros2 topic echo "${PREFIX}/joint_torque"
    ;;
  temperature)
    echo "[torque_monitor] echoing ${PREFIX}/motor_temperature (0..1 normalized)"
    exec ros2 topic echo "${PREFIX}/motor_temperature"
    ;;
  summary)
    echo "[torque_monitor] device=${SIDE} stiffness=${STIFFNESS_N_PER_M} N/m"
    echo "[torque_monitor] all torque values are COMMANDED, this hardware has no torque sensor"
    exec python3 - "${PREFIX}" "${RATE_HZ}" "${STIFFNESS_N_PER_M}" <<'PY'
import math
import sys

import rclpy
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from rclpy.node import Node

prefix, rate_hz, stiffness = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])


class Monitor(Node):
    def __init__(self):
        super().__init__("touch_torque_monitor")
        self.joint = None
        self.gimbal = None
        self.force = None
        self.track = None
        self.temp = None
        self.create_subscription(Vector3Stamped, f"{prefix}/joint_torque", self._j, 10)
        self.create_subscription(Vector3Stamped, f"{prefix}/gimbal_torque", self._g, 10)
        self.create_subscription(Vector3Stamped, f"{prefix}/tracking_error", self._t, 10)
        self.create_subscription(Vector3Stamped, f"{prefix}/motor_temperature", self._m, 10)
        self.create_subscription(WrenchStamped, f"{prefix}/applied_force", self._f, 10)
        self.create_timer(1.0 / max(rate_hz, 0.1), self._tick)

    def _j(self, m): self.joint = (m.vector.x, m.vector.y, m.vector.z)
    def _g(self, m): self.gimbal = (m.vector.x, m.vector.y, m.vector.z)
    def _t(self, m): self.track = (m.vector.x, m.vector.y, m.vector.z)
    def _m(self, m): self.temp = (m.vector.x, m.vector.y, m.vector.z)
    def _f(self, m): self.force = (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z)

    @staticmethod
    def _fmt(v, width=7, prec=4):
        if v is None:
            return "  (waiting)  "
        return "(" + ",".join(f"{x:+{width}.{prec}f}" for x in v) + ")"

    def _tick(self):
        line = f"joint_torque_cmd={self._fmt(self.joint)}"
        line += f" gimbal={self._fmt(self.gimbal)}"
        if self.force is not None:
            line += f" |force_cmd|={math.sqrt(sum(x * x for x in self.force)):.3f}N"
        if self.track is not None:
            err = math.sqrt(sum(x * x for x in self.track))
            line += f" track_err={err:.4f}m est_load={err * stiffness:.3f}N"
        if self.temp is not None:
            line += f" motor_temp_max={max(self.temp):.2f}"
        print(line, flush=True)


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


main()
PY
    ;;
  *)
    echo "[torque_monitor][error] MODE must be summary, raw, or temperature" >&2
    exit 1
    ;;
esac
