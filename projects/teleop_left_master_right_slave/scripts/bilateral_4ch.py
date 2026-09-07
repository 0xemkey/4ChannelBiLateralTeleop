#!/usr/bin/env python3
"""4-channel joint-space bilateral coupling between /touch_left and /touch_right.

Separate from, and does not modify, bridge_master_slave.py -- that script stays
the working Cartesian-position fallback. This one drives touch_slave's
joint-space PD path (0.5), which must be armed on BOTH nodes with
joint_space_control:=true. On a hardware build that parameter is a no-op:
TOUCH_DOB_ENABLE_TORQUE_WRITE is 0 there, so the joint-torque write always
refuses regardless of anything this script sends -- see touch_dob.hpp and
touch_slave.cpp's configureBilateral(). This script itself has no hardware
dependency at all: it only publishes ROS topics, sim or real underneath.

CHANNELS, Lawrence numbering (see BilateralStatus.msg for the same table):
    C1  slave force  -> master     C2  slave position -> master
    C3  master position -> slave   C4  master force   -> slave

WHAT CROSSES, AND WHY WEIGHTS APPLY DIFFERENTLY TO EACH PAIR
--------------------------------------------------------------
Force channels (C1, C4) are true weighted sums, matching the plan's Lawrence
form exactly:
    tau_m += -w1 * Cf * tau_e_hat      (C1, sent to touch_left/joint_torque_ext_cmd)
    tau_s += +w4 * Cf * tau_h_hat      (C4, sent to touch_right/joint_torque_ext_cmd)
touch_slave.cpp adds this straight onto its own Kp*(target-q)-Kd*qdot term, so
weight 0 sends exact zero torque contribution -- clean disable.

Position channels (C2, C3) do not have a weight term in the plan's tau_m/tau_s
formula at all; the coupling is carried entirely by the target angle itself.
There is no live, per-tick way to zero the paper's Kp/Kd on the receiving
node from here (they are node parameters, not a topic), so this script blends
the SETPOINT instead:
    q_target = q_own + weight * (q_peer - q_own)
At weight 0 this sends "hold your own current pose" -- a soft PD hold, not a
free Cartesian arm. That is a real, documented difference from
joint_space_control:=false. Genuinely free motion on a channel-disabled side
is an operator decision made at node launch, not something this script can
fake by weighting alone.

SAFETY (mandatory, not optional -- see the plan's Safety clamps section)
--------------------------------------------------------------------------
  energy tank      C1 (the channel that pushes force into the operator's
                   hand) is metered by a tank, joules. Drained by positive
                   work done ON the operator; empties -> C1 weight ramps to 0
                   and TANK_EMPTY is reported. A pumped oscillation is
                   sustained positive work, so this is a self-triggering cut.
  oscillation abort  peak-to-peak of the C1 torque contribution over a 1 s
                   sliding window. Exceeding the threshold, or 3 consecutive
                   windows of growth, latches C1=C4=0 and SAFETY_TRIPPED.
                   Requires an explicit re-arm (--manual_rearm parameter),
                   never automatic.
  ramping          every weight moves toward its preset target at a fixed
                   rate (default 0.5 s full-scale) rather than stepping. A
                   step in w1 is a step force into a human hand.

Tested against touch_slave_sim / hd_sim -- see sim/tests/test_channel_presets.py
and test_c4_tracking_improvement.py. No physical hardware required or assumed.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool

from touch_haptics.msg import BilateralStatus, ForceEstimate

JOINTS = ("waist", "shoulder", "elbow")

# C1, C2, C3, C4
PRESETS = {
    "unilateral": (0.0, 0.0, 1.0, 0.0),
    "pp": (0.0, 1.0, 1.0, 0.0),
    "pf": (1.0, 0.0, 1.0, 0.0),
    "4ch_safe": (0.5, 1.0, 1.0, 0.3),
    "4ch": (1.0, 1.0, 1.0, 1.0),
}

REASON_OK = 0
REASON_ZERO_WEIGHT = 1
REASON_STALE = 2
REASON_INVALID = 3
REASON_BORROWED_REFUSED = 4
REASON_TANK_EMPTY = 5
REASON_TRIPPED = 6
REASON_CLUTCHED = 7

SAFETY_NOMINAL = 0
SAFETY_DERATED = 1
SAFETY_TRIPPED = 2

# Per-joint torque ceilings, matching configureDob()'s kMaxTorque in
# touch_slave.cpp (f_max = 0.6 N). Used only to report a budget-used fraction;
# touch_slave.cpp's own clampJointTorque() is the actual enforcement, this
# script enforces nothing.
MAX_TORQUE_NM = (0.16200, 0.08100, 0.08100)

# Deadband #1, N*m -- a HAND-KEPT COPY of touch_slave.cpp's kThreshold array
# (computeForceEstimate()'s per-joint noise floor, "2x measured p99
# |tau_dis_hat|"). Not read from the C++ side because it is not currently
# exposed over ROS (no parameter, no topic) -- if kThreshold ever changes in
# touch_slave.cpp, this must be updated by hand or effective_threshold below
# will silently report the wrong number. [waist, shoulder, elbow].
THRESHOLD_NM = (0.07632, 0.06400, 0.05000)

# Deadband #2, N*m -- a HAND-KEPT COPY of touch_slave.cpp's
# kMinRelayTorqueNormNm (the floor applied to the already-weighted relay
# signal before Cartesian conversion). Same hand-sync caveat as THRESHOLD_NM.
MIN_RELAY_TORQUE_NORM_NM = 0.02


def effective_threshold_nm(weight: float) -> List[float]:
    """Combined deadband #1 + #2, N*m, per joint, for one force channel.

    effective_threshold[j] = THRESHOLD_NM[j] + MIN_RELAY_TORQUE_NORM_NM / weight

    At weight <= 0 the channel can never open regardless of push size --
    returned as +inf, not a large finite number, so a consumer cannot
    mistake "disabled" for "just a strict floor".

    W-VALUE RANGE FOR A REAL ~0.1 N*m TOUCH TO CLEAR THIS FLOOR (documented
    here per Phase 1.2's acceptance criterion, solving effective_threshold
    <= 0.1 for weight):
        waist:    weight >= 0.845   (0.07632 + 0.02/0.845 ~= 0.1000)
        shoulder: weight >= 0.556   (moot -- masked out of C1 by default)
        elbow:    weight >= 0.400   (0.05000 + 0.02/0.400  ~= 0.1000)
    Elbow is the binding joint for C1 (shoulder is masked, waist needs a
    higher weight than elbow) -- confirmed empirically 2026-09-03: at
    w1=0.4 elbow relayed real signal from a firm push while waist needed
    noticeably harder pushing to clear its own, stricter floor. A weight
    below ~0.4 means NO ordinary touch on any relayed joint can ever clear
    the combined deadband, independent of whether the DOB detected it.
    """
    out = []
    for threshold in THRESHOLD_NM:
        if weight > 0.0:
            out.append(threshold + MIN_RELAY_TORQUE_NORM_NM / weight)
        else:
            out.append(float("inf"))
    return out


def analyze_window(samples: List[tuple]) -> tuple:
    """Pure function, no ROS dependency -- unit-tested directly in
    sim/tests/test_oscillation_guard.py without needing rclpy or a sim
    process running, the same "messaging logic, not control math, so test
    it standalone" precedent as touch_force_estimate.hpp's
    computeForceEstimate().

    samples: list of (t, [x, y, z]) tuples spanning one ~1s window.
    Returns (peak_to_peak_norm, max_zero_crossing_rate_hz).

    The zero-crossing rate is the Phase 1.3 secondary check: computed
    per-axis (a sign change on ANY axis counts), taking the max across
    axes. A monotonic human push (escalating in one direction) has a
    near-zero crossing rate no matter how large its pp is; a genuine
    mechanical oscillation crosses zero repeatedly per second. Samples
    within a small epsilon of zero are excluded from the sign sequence
    before counting, so resting-noise dither around zero is not counted as
    crossings.
    """
    if not samples:
        return 0.0, 0.0
    vals = [_norm(v) for _, v in samples]
    pp = max(vals) - min(vals)
    duration = max(samples[-1][0] - samples[0][0], 1e-3)
    n_axes = len(samples[0][1])
    max_crossings = 0
    eps = 1e-6
    for axis in range(n_axes):
        signs = []
        for _, v in samples:
            x = v[axis]
            if x > eps:
                signs.append(1)
            elif x < -eps:
                signs.append(-1)
            # else: drop near-zero samples from the sign sequence
        crossings = sum(1 for i in range(1, len(signs)) if signs[i] != signs[i - 1])
        max_crossings = max(max_crossings, crossings)
    return pp, max_crossings / duration


def _vec3(v) -> Vector3Stamped:
    msg = Vector3Stamped()
    msg.vector.x, msg.vector.y, msg.vector.z = (float(v[0]), float(v[1]), float(v[2]))
    return msg


def _norm(v) -> float:
    return math.sqrt(sum(x * x for x in v))


class BilateralController(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("bilateral_4ch")
        self.args = args

        if args.architecture == "custom":
            target = (args.w1, args.w2, args.w3, args.w4)
        elif args.architecture in PRESETS:
            target = PRESETS[args.architecture]
        else:
            raise SystemExit(
                f"unknown --architecture={args.architecture!r}; choose from "
                f"{sorted(PRESETS)} or 'custom' with --w1..--w4"
            )
        self.target_weights: List[float] = list(target)
        self.weights: List[float] = list(target)  # ramped state, starts at target
        self.architecture = args.architecture

        disabled = {
            name.strip() for name in args.c1_disable_joints.split(",") if name.strip()
        }
        unknown = disabled - set(JOINTS)
        if unknown:
            raise SystemExit(
                f"--c1-disable-joints: unknown joint(s) {sorted(unknown)}; "
                f"choose from {JOINTS}"
            )
        self.c1_joint_mask = [0.0 if name in disabled else 1.0 for name in JOINTS]
        if disabled:
            self.get_logger().warn(
                f"C1 relay excludes joint(s) {sorted(disabled)} "
                f"(forced to zero torque contribution, see --c1-disable-joints)"
            )

        # Latest peer inputs, each with its own wall-clock arrival stamp for
        # staleness -- same convention as bridge_master_slave.py.
        self.left_fe: Optional[ForceEstimate] = None
        self.right_fe: Optional[ForceEstimate] = None
        self.left_fe_time = 0.0
        self.right_fe_time = 0.0

        self.left_q: Optional[List[float]] = None
        self.right_q: Optional[List[float]] = None
        self.left_q_time = 0.0
        self.right_q_time = 0.0
        self._prev_left_q: Optional[List[float]] = None
        self._prev_left_q_time = 0.0
        self.left_qdot = [0.0, 0.0, 0.0]

        # Energy tank, C1 only (force delivered TO the operator's hand).
        self.tank_j = args.tank_capacity_j

        # Oscillation abort. window_samples holds (t, tau_c1_vector) pairs
        # within the last second (the full 3-vector, not just its norm --
        # needed for analyze_window()'s per-axis zero-crossing count, Phase
        # 1.3); window_pp_history holds the peak-to-peak of each completed
        # 1 s window, newest last, capped at 3.
        self._window_samples: List[tuple] = []
        self._window_start = time.time()
        self._window_pp_history: List[float] = []
        self._last_zero_crossing_hz = 0.0
        self.tripped = False

        self.create_subscription(
            ForceEstimate, args.left_force_topic, self._left_fe_cb, 10)
        self.create_subscription(
            ForceEstimate, args.right_force_topic, self._right_fe_cb, 10)
        self.create_subscription(
            Vector3Stamped, args.left_joint_angles_topic, self._left_q_cb, 10)
        self.create_subscription(
            Vector3Stamped, args.right_joint_angles_topic, self._right_q_cb, 10)

        self.left_target_pub = self.create_publisher(
            Vector3Stamped, args.left_joint_target_topic, 10)
        self.right_target_pub = self.create_publisher(
            Vector3Stamped, args.right_joint_target_topic, 10)
        self.left_ext_pub = self.create_publisher(
            ForceEstimate, args.left_joint_torque_ext_topic, 10)
        self.right_ext_pub = self.create_publisher(
            ForceEstimate, args.right_joint_torque_ext_topic, 10)
        self.status_pub = self.create_publisher(
            BilateralStatus, args.status_topic, 20)

        # C1/C4 REAL BUG, fixed 2026-09-07: the Cartesian relay block in
        # touch_slave.cpp (both master and slave) is gated on
        # `command.enabled`, the SAME flag /enable controls -- with nothing
        # else running, this stays false forever (enabled_on_start_ defaults
        # false, and neither run_master_left.sh nor run_slave_right.sh ever
        # sets it). bridge_master_slave.py already publishes the SLAVE's own
        # /enable unconditionally whenever C3 is armed, so C4 (into the
        # slave) was never affected -- but the MASTER's /enable is published
        # ONLY when the bridge runs with --enable-c2 or --reflect-enable,
        # neither of which this script requires. Without one of those, C1
        # silently did nothing on real hardware regardless of weight --
        # confirmed live: same class of defect as the applied_force
        # telemetry bug found the same week (a real precondition, not
        # obviously visible from this script's own code). This script is
        # exactly the node responsible for getting C1/C4 onto these
        # devices, so it now ensures its own precondition rather than
        # requiring a separate manual publisher per side. `command.enabled`
        # has no staleness/timeout (touch_slave.cpp's enableCallback just
        # latches the last value), so a low-rate publish is enough, but this
        # publishes every tick to match bridge_master_slave.py's own
        # existing convention and to self-heal if the receiving node
        # restarts mid-session.
        self.left_enable_pub = self.create_publisher(
            Bool, args.left_enable_topic, 10)
        self.right_enable_pub = self.create_publisher(
            Bool, args.right_enable_topic, 10)

        # Manual re-arm: `ros2 param set /bilateral_4ch manual_rearm true`.
        # Consumed and reset to false the instant it clears a trip, so it
        # cannot be left permanently true and silently defeat the abort.
        self.declare_parameter("manual_rearm", False)
        self.add_on_set_parameters_callback(self._on_param)

        self.duty_cycle_nm = 0.0
        self.rate_hz = max(args.rate_hz, 1.0)
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().warn(
            f"bilateral_4ch started: architecture={self.architecture!r} "
            f"target_weights(C1..C4)={self.target_weights} rate={self.rate_hz:.0f} Hz "
            f"tank={args.tank_capacity_j:.3f} J trip_pp={args.trip_pp_nm:.3f} N*m"
        )
        # Phase 1.1: which channels are active must be explicitly visible in
        # the logs, not inferrable only by reading the weight list above. A
        # weight of exactly 0.0 is a SILENT full disable (clean by design --
        # see the module docstring -- but silent is exactly the problem: C4
        # sat at w4=0.0 for an entire real-hardware session in 2026-09-03
        # before anyone noticed it had never been enabled at all).
        channel_names = ("C1 (slave force -> master)", "C2 (slave pos -> master)",
                          "C3 (master pos -> slave)", "C4 (master force -> slave)")
        for name, w in zip(channel_names, self.target_weights):
            if w <= 0.0:
                self.get_logger().warn(f"{name}: DISABLED (weight=0.0)")
            else:
                self.get_logger().info(f"{name}: active, weight={w:.3f}")
        for name, w, thr in (
            ("C1", self.target_weights[0], effective_threshold_nm(self.target_weights[0])),
            ("C4", self.target_weights[3], effective_threshold_nm(self.target_weights[3])),
        ):
            self.get_logger().info(
                f"{name} effective combined deadband (waist/shoulder/elbow), N*m: "
                f"{thr[0]:.4f}/{thr[1]:.4f}/{thr[2]:.4f} at weight={w:.3f}"
            )

    # ---- subscriptions ---------------------------------------------------

    def _left_fe_cb(self, msg: ForceEstimate) -> None:
        self.left_fe = msg
        self.left_fe_time = time.time()

    def _right_fe_cb(self, msg: ForceEstimate) -> None:
        self.right_fe = msg
        self.right_fe_time = time.time()

    def _left_q_cb(self, msg: Vector3Stamped) -> None:
        q = [msg.vector.x, msg.vector.y, msg.vector.z]
        now = time.time()
        if self._prev_left_q is not None:
            step_dt = max(now - self._prev_left_q_time, 1e-4)
            raw_qdot = [(q[i] - self._prev_left_q[i]) / step_dt for i in range(3)]
            alpha = min(max(self.args.qdot_alpha, 0.0), 1.0)
            self.left_qdot = [
                (1.0 - alpha) * self.left_qdot[i] + alpha * raw_qdot[i] for i in range(3)
            ]
        self._prev_left_q = list(q)
        self._prev_left_q_time = now
        self.left_q = q
        self.left_q_time = now

    def _right_q_cb(self, msg: Vector3Stamped) -> None:
        self.right_q = [msg.vector.x, msg.vector.y, msg.vector.z]
        self.right_q_time = time.time()

    def _on_param(self, params) -> SetParametersResult:
        for p in params:
            # p.value is already the unwrapped Python bool here (rclpy's
            # Parameter.value property), not a raw ParameterValue message --
            # the earlier p.value.bool_value crashed every real call with
            # AttributeError, so manual_rearm never actually worked.
            if p.name == "manual_rearm" and p.value and self.tripped:
                self.tripped = False
                self._window_pp_history.clear()
                self._window_samples.clear()
                self.get_logger().warn("manual_rearm: oscillation trip cleared")
        return SetParametersResult(successful=True)

    # ---- main loop ---------------------------------------------------

    def _tick(self) -> None:
        now = time.time()
        stale = self.args.stale_timeout_s

        left_fe_fresh = self.left_fe is not None and (now - self.left_fe_time) <= stale
        right_fe_fresh = self.right_fe is not None and (now - self.right_fe_time) <= stale
        left_q_fresh = self.left_q is not None and (now - self.left_q_time) <= stale
        right_q_fresh = self.right_q is not None and (now - self.right_q_time) <= stale

        left_fe_valid = left_fe_fresh and bool(self.left_fe.valid)
        right_fe_valid = right_fe_fresh and bool(self.right_fe.valid)

        # ---- ramp weights toward target, 0.5 s full-scale by default ----
        max_step = self.dt / max(self.args.ramp_time_s, 1e-3)
        for i in range(4):
            delta = self.target_weights[i] - self.weights[i]
            if abs(delta) <= max_step:
                self.weights[i] = self.target_weights[i]
            else:
                self.weights[i] += math.copysign(max_step, delta)

        # Tripped and tank-empty are ADDITIONAL gates on top of the ramped
        # weight, not replacements for it -- so a trip clears back to
        # whatever weight the ramp had already reached, not to the preset.
        tank_empty = self.tank_j <= 0.0
        w1_eff = 0.0 if (self.tripped or tank_empty) else self.weights[0]
        w4_eff = 0.0 if self.tripped else self.weights[3]
        w2_eff = self.weights[1]
        w3_eff = self.weights[2]

        # See the constructor comment above self.left_enable_pub: C1 cannot
        # reach the master, and C4 cannot reach the slave, unless that
        # node's own command.enabled is true. Tied to the TARGET weight
        # (self.weights, pre-trip/pre-tank), not w1_eff/w4_eff -- enable
        # should stay true through a ramp-down or a trip (which should zero
        # the relayed FORCE, not silently disable the device for whatever
        # else might legitimately want it enabled), matching how
        # bridge_master_slave.py never toggles the slave's enable off just
        # because tracking is momentarily degraded.
        enable_msg = Bool()
        enable_msg.data = self.target_weights[0] > 0.0
        self.left_enable_pub.publish(enable_msg)
        enable_msg2 = Bool()
        enable_msg2.data = self.target_weights[3] > 0.0
        self.right_enable_pub.publish(enable_msg2)

        # ---- C1: slave contact torque -> master ----
        tau_e_hat = list(self.right_fe.tau_ext_deadbanded_nm) if right_fe_valid else [0.0, 0.0, 0.0]
        tau_c1 = [
            w1_eff * self.args.cf * v * m for v, m in zip(tau_e_hat, self.c1_joint_mask)
        ]

        # ---- C4: master hand torque -> slave ----
        tau_h_hat = list(self.left_fe.tau_ext_deadbanded_nm) if left_fe_valid else [0.0, 0.0, 0.0]
        tau_c4 = [w4_eff * self.args.cf * v for v in tau_h_hat]

        # ---- energy tank, drained by positive work C1 does on the master ----
        # power = tau . qdot at the master joint, W. Only the C1 contribution
        # counts -- C2's spring is not a force channel in this accounting.
        power_w = sum(tau_c1[i] * self.left_qdot[i] for i in range(3))
        if power_w > 0.0:
            self.tank_j = max(0.0, self.tank_j - power_w * self.dt)
        else:
            recharge_w = self.args.tank_capacity_j / max(self.args.tank_recharge_time_s, 1e-3)
            self.tank_j = min(self.args.tank_capacity_j, self.tank_j + recharge_w * self.dt)

        # ---- oscillation abort: peak-to-peak + zero-crossing rate of tau_c1
        # over 1 s windows ----
        mag_c1 = _norm(tau_c1)
        self._window_samples.append((now, list(tau_c1)))
        self._window_samples = [(t, v) for t, v in self._window_samples if now - t <= 1.0]
        if now - self._window_start >= 1.0:
            pp, zero_crossing_hz = analyze_window(self._window_samples)
            self._last_zero_crossing_hz = zero_crossing_hz
            self._window_pp_history.append(pp)
            if len(self._window_pp_history) > 3:
                self._window_pp_history.pop(0)
            self._window_start = now

            # Bug fixed 2026-09-02: "growing" alone (3 strictly-increasing
            # windows) false-triggered on the ordinary onset of ANY new
            # contact -- a signal ramping up from zero always looks like 3
            # increasing windows for its first few seconds, confirmed live in
            # sim (tripped identically on an abrupt step AND a gentle 3s
            # ramp, at pp=0.0165 N*m against a 0.4 N*m threshold). The actual
            # danger this check exists for is a REAL oscillation building up
            # toward the hard limit, not any signal turning on -- so also
            # require the latest window to have reached a meaningful
            # fraction of trip_pp_nm before three-rising-windows counts as
            # "growing". A genuinely dangerous ramp will cross this floor
            # well before it reaches trip_pp_nm itself; a normal contact
            # onset that plateaus below it won't.
            #
            # Second bug found 2026-09-03/04: even with the floor above, a
            # human DELIBERATELY escalating push force to find a threshold
            # produces the exact same "3 rising windows above the floor"
            # shape as a real growing oscillation -- confirmed live, this
            # tripped bilateral_4ch three separate times during ordinary
            # verification testing that day. A monotonic push and a genuine
            # oscillation are NOT distinguishable by magnitude alone; they
            # ARE distinguishable by periodicity. Added a secondary
            # zero-crossing-rate check (analyze_window() above): a monotonic
            # escalating push crosses zero ~0 times regardless of how large
            # or fast-rising it gets, while a real mechanical oscillation
            # crosses zero repeatedly per second. Only gates the "growing"
            # early-warning branch -- the hard pp > trip_pp_nm ceiling below
            # is an absolute backstop and stays ungated by periodicity on
            # purpose, since a large-enough push should still cut regardless
            # of whether it happens to be periodic.
            kGrowingFloorFrac = 0.25
            growing = (
                len(self._window_pp_history) == 3
                and self._window_pp_history[0] < self._window_pp_history[1] < self._window_pp_history[2]
                and self._window_pp_history[2] > kGrowingFloorFrac * self.args.trip_pp_nm
                and zero_crossing_hz >= self.args.min_oscillation_hz
            )
            if pp > self.args.trip_pp_nm or growing:
                if not self.tripped:
                    self.tripped = True
                    self.get_logger().error(
                        f"OSCILLATION ABORT: pp={pp:.4f} N*m "
                        f"(threshold {self.args.trip_pp_nm:.4f}) growing={growing} "
                        f"zero_crossing_hz={zero_crossing_hz:.2f} "
                        f"(min {self.args.min_oscillation_hz:.2f}). "
                        f"C1/C4 latched off. Clear with "
                        f"'ros2 param set /{self.get_name()} manual_rearm true'"
                    )

        # ---- publish position targets (C2/C3 setpoint blend) ----
        if left_q_fresh and right_q_fresh:
            left_target = [
                self.left_q[i] + w2_eff * (self.right_q[i] - self.left_q[i]) for i in range(3)
            ]
            right_target = [
                self.right_q[i] + w3_eff * (self.left_q[i] - self.right_q[i]) for i in range(3)
            ]
            self.left_target_pub.publish(_vec3(left_target))
            self.right_target_pub.publish(_vec3(right_target))

        # ---- publish force contributions (C1/C4), relayed ForceEstimate ----
        self._publish_ext(self.left_ext_pub, tau_c1, valid=right_fe_valid)
        self._publish_ext(self.right_ext_pub, tau_c4, valid=left_fe_valid)

        # ---- duty cycle, 60 s running mean of |tau_c1| + |tau_c4| ----
        cmd_mag = mag_c1 + _norm(tau_c4)
        alpha = self.dt / 60.0
        self.duty_cycle_nm = (1.0 - alpha) * self.duty_cycle_nm + alpha * cmd_mag

        self._publish_status(
            now, left_fe_fresh, right_fe_fresh, left_q_fresh, right_q_fresh,
            left_fe_valid, right_fe_valid, tank_empty,
            w1_eff, w2_eff, w3_eff, w4_eff, tau_e_hat, tau_h_hat, tau_c1, tau_c4,
        )

    def _publish_ext(self, pub, tau_deadbanded, valid: bool) -> None:
        msg = ForceEstimate()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(3):
            msg.tau_ext_deadbanded_nm[i] = float(tau_deadbanded[i])
            msg.tau_ext_nm[i] = float(tau_deadbanded[i])
        msg.valid = bool(valid)
        msg.borrowed_params = False
        pub.publish(msg)

    def _channel_reason(self, weight_target: float, fresh: bool, valid: bool,
                        tank_empty: bool) -> int:
        if self.tripped:
            return REASON_TRIPPED
        if weight_target <= 0.0:
            return REASON_ZERO_WEIGHT
        if tank_empty:
            return REASON_TANK_EMPTY
        if not fresh:
            return REASON_STALE
        if not valid:
            return REASON_INVALID
        return REASON_OK

    def _publish_status(
        self, now, left_fe_fresh, right_fe_fresh, left_q_fresh, right_q_fresh,
        left_fe_valid, right_fe_valid, tank_empty,
        w1_eff, w2_eff, w3_eff, w4_eff, tau_e_hat, tau_h_hat, tau_c1, tau_c4,
    ) -> None:
        msg = BilateralStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.architecture = self.architecture

        msg.channel_weight = [float(w1_eff), float(w2_eff), float(w3_eff), float(w4_eff)]

        reasons = [
            self._channel_reason(self.weights[0], right_fe_fresh, right_fe_valid, tank_empty),
            self._channel_reason(self.weights[1], left_q_fresh and right_q_fresh, True, False),
            self._channel_reason(self.weights[2], left_q_fresh and right_q_fresh, True, False),
            self._channel_reason(self.weights[3], left_fe_fresh, left_fe_valid, False),
        ]
        msg.channel_reason = [int(r) for r in reasons]
        msg.channel_active = [
            bool(w1_eff > 0.0 and reasons[0] == REASON_OK),
            bool(w2_eff > 0.0 and reasons[1] == REASON_OK),
            bool(w3_eff > 0.0 and reasons[2] == REASON_OK),
            bool(w4_eff > 0.0 and reasons[3] == REASON_OK),
        ]

        msg.tau_e_hat_nm = [float(v) for v in tau_e_hat]
        msg.tau_h_hat_nm = [float(v) for v in tau_h_hat]
        msg.tau_e_age_s = float(now - self.right_fe_time) if self.right_fe is not None else -1.0
        msg.tau_h_age_s = float(now - self.left_fe_time) if self.left_fe is not None else -1.0

        # NOTE: these are the force-channel contributions THIS script sent,
        # not the node's true final command -- that also includes the local
        # Kp/Kd PD term computed inside touch_slave.cpp's hdCallback, which
        # this script has no visibility into. Documented, not measured.
        msg.master_torque_cmd_nm = [float(v) for v in tau_c1]
        msg.slave_torque_cmd_nm = [float(v) for v in tau_c4]

        msg.master_budget_used = max(
            (abs(tau_c1[i]) / MAX_TORQUE_NM[i] for i in range(3)), default=0.0)
        msg.slave_budget_used = max(
            (abs(tau_c4[i]) / MAX_TORQUE_NM[i] for i in range(3)), default=0.0)

        msg.passivity_energy_j = float(self.tank_j)
        msg.damping_injected_nms_per_rad = 0.0  # not implemented in this scope
        msg.duty_cycle_nm = float(self.duty_cycle_nm)
        msg.clutch_active = False
        msg.grip_detected = False
        msg.safety_state = SAFETY_TRIPPED if self.tripped else SAFETY_NOMINAL

        # Phase 1.2: combined deadband #1+#2, per channel, at the CURRENT
        # ramped weight (not the target) -- during a ramp this reflects what
        # is actually required right now, not where it's heading.
        msg.effective_threshold_c1_nm = [float(v) for v in effective_threshold_nm(w1_eff)]
        msg.effective_threshold_c4_nm = [float(v) for v in effective_threshold_nm(w4_eff)]
        # Phase 1.3 diagnostic: last completed window's C1 zero-crossing rate.
        msg.c1_zero_crossing_hz = float(self._last_zero_crossing_hz)

        self.status_pub.publish(msg)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--architecture", default="pp",
                   help=f"one of {sorted(PRESETS)} or 'custom' (use --w1..--w4)")
    p.add_argument("--w1", type=float, default=0.0, help="C1 weight, custom architecture only")
    p.add_argument("--w2", type=float, default=1.0, help="C2 weight, custom architecture only")
    p.add_argument("--w3", type=float, default=1.0, help="C3 weight, custom architecture only")
    p.add_argument("--w4", type=float, default=0.0, help="C4 weight, custom architecture only")
    p.add_argument("--cf", type=float, default=1.0, dest="cf",
                   help="Force-channel scale factor. 1.0 is the passive choice for identical "
                        "devices at 1:1 scaling; any other value breaks power conservation "
                        "and must be a deliberate, documented choice (plan Sec 2.1).")
    p.add_argument("--c1-disable-joints", default="shoulder",
                   help="Comma-separated joint names (from waist,shoulder,elbow) whose C1 "
                        "contribution is forced to zero before weighting/relay. Default "
                        "excludes shoulder: the right-arm DOB replay verification found its "
                        "noise/friction floor meets or exceeds its own torque budget at every "
                        "filter cutoff tested (2026-09-01), so its force_ext_hat is not fit "
                        "for relaying to the master. Waist and elbow have workable headroom. "
                        "Pass '' to relay all three once/if the shoulder fit is fixed.")

    p.add_argument("--left-force-topic", default="/touch_left/force_ext_hat")
    p.add_argument("--right-force-topic", default="/touch_right/force_ext_hat")
    p.add_argument("--left-joint-angles-topic", default="/touch_left/joint_angles")
    p.add_argument("--right-joint-angles-topic", default="/touch_right/joint_angles")
    p.add_argument("--left-joint-target-topic", default="/touch_left/joint_target")
    p.add_argument("--right-joint-target-topic", default="/touch_right/joint_target")
    p.add_argument("--left-joint-torque-ext-topic", default="/touch_left/joint_torque_ext_cmd")
    p.add_argument("--right-joint-torque-ext-topic", default="/touch_right/joint_torque_ext_cmd")
    p.add_argument("--left-enable-topic", default="/touch_left/enable",
                   help="Published true whenever w1>0 (C1 needs the master's own "
                        "command.enabled to write the relayed Cartesian force). "
                        "Same topic bridge_master_slave.py's --enable-c2/--reflect-enable "
                        "would otherwise publish -- this script no longer depends on either.")
    p.add_argument("--right-enable-topic", default="/touch_right/enable",
                   help="Published true whenever w4>0. Usually redundant with the "
                        "position bridge's own unconditional slave enable, kept for when "
                        "this script runs without that bridge (e.g. sim tests).")
    p.add_argument("--status-topic", default="/bilateral_status")

    p.add_argument("--rate-hz", type=float, default=200.0,
                   help="Must be >= the slower node's stale_timeout_s^-1 or channels will "
                        "read STALE between ticks.")
    p.add_argument("--stale-timeout-s", type=float, default=0.1)
    p.add_argument("--ramp-time-s", type=float, default=0.5,
                   help="Full-scale weight ramp time. A step in w1 is a step force into a hand.")
    p.add_argument("--qdot-alpha", type=float, default=0.2,
                   help="Low-pass on the master joint-velocity estimate used by the energy tank.")

    p.add_argument("--tank-capacity-j", type=float, default=0.05)
    p.add_argument("--tank-recharge-time-s", type=float, default=2.0,
                   help="Seconds to refill the tank from empty when C1 is doing no positive work.")
    p.add_argument("--trip-pp-nm", type=float, default=0.4,
                   help="Peak-to-peak |tau_c1| in a 1 s window that latches the oscillation abort. "
                        "N*m in this joint-space realization -- reinterpreted from the plan's "
                        "Cartesian-domain 0.4 N figure, see the safety-clamp note in the module "
                        "docstring.")
    p.add_argument("--min-oscillation-hz", type=float, default=1.5,
                   help="Phase 1.3 secondary check: the 'growing' early-warning trip condition "
                        "additionally requires the C1 torque vector's zero-crossing rate over the "
                        "last 1s window to reach at least this many Hz, so a human's deliberately "
                        "escalating (monotonic, ~0 Hz) push is not misclassified as a developing "
                        "oscillation. The hard pp > trip_pp_nm ceiling is unaffected by this and "
                        "stays an unconditional backstop. See analyze_window().")

    return p.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = BilateralController(args)
    exit_code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except SystemExit as exc:
        exit_code = exc.code if isinstance(exc.code, int) else 1
    finally:
        try:
            zero = ForceEstimate()
            zero.valid = False
            node.left_ext_pub.publish(zero)
            node.right_ext_pub.publish(zero)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
