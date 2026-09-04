#!/usr/bin/env python3
"""Direct left-master to right-slave teleoperation bridge.

Maps left Touch pose deltas onto right touch_slave target poses. The right node
renders a spring/damper toward that target, so the right arm follows the left.

Differences from the older scripts/bridge_touch_left_to_right_slave.py:

  clutching        hold the clutch button to reposition your hand without
                   moving the slave; the anchor is retaken on release
  axis map         one --axis-map string instead of separate invert flags,
                   matching the convention used elsewhere in this repo
  rate limit       caps how fast the target may travel, so a dropped pose
                   message cannot turn into a jump
  keep-alive       target and enable are republished every tick, because the
                   right node zeroes force on commands older than its
                   stale_timeout_s (0.05 s by default)
  telemetry        optional tracking-error readout, which is the only usable
                   proxy for external load on this hardware
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def _bool_arg(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def parse_triple(spec: str, fallback: float, name: str) -> list[float]:
    """Parse '1.0,0.6,1.0' into three floats. A single value applies to all axes.

    Lets one axis be scaled or limited independently, which matters because the
    Omni's workspace is not a cube: it is appreciably shorter vertically than it
    is wide, so a 1:1 map that fits in X will run out of travel in Y.
    """
    text = str(spec or "").strip()
    if not text:
        return [fallback] * 3
    parts = [p.strip() for p in text.split(",") if p.strip()]
    try:
        values = [float(p) for p in parts]
    except ValueError as exc:
        raise ValueError(f"invalid --{name}={spec!r}: not a number") from exc
    if len(values) == 1:
        return values * 3
    if len(values) != 3:
        raise ValueError(f"invalid --{name}={spec!r}: give one value or three")
    return values


def parse_axis_map(spec: str) -> list[tuple[int, float]]:
    """Parse a map like '-x,z,y' into [(source_index, sign), ...] for slave xyz."""
    parsed: list[tuple[int, float]] = []
    for token in [part.strip().lower() for part in str(spec).split(",") if part.strip()]:
        sign = 1.0
        if token.startswith("+"):
            token = token[1:]
        elif token.startswith("-"):
            sign = -1.0
            token = token[1:]
        if token not in AXIS_INDEX:
            raise ValueError(f"invalid --axis-map={spec!r}: bad axis {token!r}")
        parsed.append((AXIS_INDEX[token], sign))
    if len(parsed) != 3:
        raise ValueError(f"invalid --axis-map={spec!r}: expected exactly three axes")
    seen = [src for src, _ in parsed]
    if sorted(seen) != [0, 1, 2]:
        raise ValueError(f"invalid --axis-map={spec!r}: each of x,y,z must appear exactly once")
    return parsed


def apply_axis_map(amap: list[tuple[int, float]], v: list[float]) -> list[float]:
    """master delta -> slave delta."""
    return [sign * v[src] for src, sign in amap]


def apply_axis_map_inverse(amap: list[tuple[int, float]], v: list[float]) -> list[float]:
    """slave-frame vector -> master frame.

    The forward map is slave[i] = sign_i * master[src_i]. Signs are +/-1, so the
    inverse is master[src_i] = sign_i * slave[i]. Needed to push the operator's
    hand back along the axis they are actually driving.
    """
    out = [0.0, 0.0, 0.0]
    for i, (src, sign) in enumerate(amap):
        out[src] = sign * v[i]
    return out


class MasterSlaveBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("touch_master_slave_bridge")
        self.args = args
        self.axis_map = parse_axis_map(args.axis_map)
        # Both are in SLAVE axis order, applied after the axis map, because the
        # thing being fitted is the slave's workspace.
        self.axis_scale = parse_triple(args.axis_scale, args.scale, "axis-scale")
        self.max_offset = parse_triple(args.max_offset_xyz, args.max_offset_m, "max-offset-xyz")
        self.absolute_axes = sorted({
            AXIS_INDEX[c] for c in str(args.absolute_axes or "").strip().lower()
            if c in AXIS_INDEX
        })
        # Per-axis trim (2026-09-04, Phase 3 fine-tuning): y and z showed a
        # roughly CONSTANT (not motion-dependent) gap between master and
        # slave in a Phase 2 capture -- ~0.010-0.013m on y, ~0.025-0.035m on
        # z, matching teleop_calibration.json's own right_minus_mapped_left
        # diagnostic almost exactly. That value was previously recorded but
        # never applied (see _write_calibration's "for diagnostics" note).
        # A single shared --absolute-trim-m cannot correct two axes with two
        # different offsets at once, so this adds a per-axis version of the
        # same --max-offset-xyz pattern already used elsewhere in this file.
        self.absolute_trim = parse_triple(
            args.absolute_trim_xyz, args.absolute_trim_m, "absolute-trim-xyz")

        self.left_anchor: Optional[list[float]] = None
        self.right_anchor: Optional[list[float]] = None
        self.latest_left: Optional[list[float]] = None
        self.latest_right: Optional[list[float]] = None
        self.left_buttons = 0
        self.clutch_active = False
        self.last_target: Optional[list[float]] = None
        self.tracking_error: Optional[list[float]] = None
        self.last_report = 0.0
        self.start_time = time.time()
        self.last_wait_notice = 0.0
        self.ready_announced = False
        self.force_sub_warned = False

        # Force reflection state.
        self.reflect_engaged = False
        self.reflect_force = [0.0, 0.0, 0.0]
        self.reflect_excess = 0.0
        self.prev_target: Optional[list[float]] = None
        self.target_speed = 0.0
        self.master_enable_sent = False
        self.prev_left: Optional[list[float]] = None
        self.master_vel = [0.0, 0.0, 0.0]
        self.prev_right: Optional[list[float]] = None
        self.slave_speed = 0.0
        self.stalled = False
        self.limit_notice = [0.0, 0.0, 0.0]
        # Furthest each axis was asked to go, and furthest it actually got.
        # A gap between them is the slave falling short of your hand.
        self.peak_cmd = [0.0, 0.0, 0.0]
        self.peak_act = [0.0, 0.0, 0.0]

        self.create_subscription(PoseStamped, args.left_pose_topic, self._left_cb, 10)
        self.create_subscription(PoseStamped, args.right_pose_topic, self._right_cb, 10)
        self.create_subscription(Int32, args.left_buttons_topic, self._buttons_cb, 10)
        if args.report or args.reflect_enable:
            self.create_subscription(
                Vector3Stamped, args.tracking_error_topic, self._tracking_cb, 10
            )

        self.target_pub = self.create_publisher(PoseStamped, args.right_target_topic, 10)
        self.enable_pub = self.create_publisher(Bool, args.right_enable_topic, 10)

        # C2 (slave position -> master): pulls the master's own spring toward
        # wherever the slave actually is, the reverse direction of C3. Off by
        # default -- this changes what force the operator's hand feels, so it
        # needs an explicit opt-in rather than coming along for free with the
        # rest of the bridge.
        self.left_target_pub = None
        self.left_enable_pub = None
        self.last_left_target: Optional[list[float]] = None
        if args.enable_c2:
            self.left_target_pub = self.create_publisher(PoseStamped, args.left_target_topic, 10)
            self.left_enable_pub = self.create_publisher(Bool, args.left_enable_topic, 10)

        self.master_force_pub = None
        self.master_enable_pub = None
        if args.reflect_enable:
            self.master_force_pub = self.create_publisher(WrenchStamped, args.master_force_topic, 10)
            self.master_enable_pub = self.create_publisher(Bool, args.master_enable_topic, 10)

        # Gravity compensation for the slave. The Omni is not gravity
        # compensated, so on the vertical axis the tracking spring has to lift
        # the arm's own weight before it moves at all -- which is why that axis
        # runs out of range first. A constant bias on force_cmd is added to the
        # spring inside the node, freeing the spring to do the tracking.
        self.slave_force_pub = None
        if args.slave_gravity_comp_n != 0.0:
            self.slave_force_pub = self.create_publisher(
                WrenchStamped, args.slave_force_topic, 10)

        # Symmetric fix for C2, added 2026-09-03: the master has the exact
        # same unweighted-vertical-axis problem as the slave (confirmed
        # live -- C2's commanded target updated correctly on all 3 axes,
        # but the master's own spring could not visibly follow on Y even
        # after raising LEFT_STIFFNESS_N_PER_M from 15 to 28 N/m; Z and X
        # were fine at the same stiffness, isolating this to gravity, not
        # general spring weakness). Same mechanism as slave_force_pub above.
        self.left_force_pub = None
        if args.left_gravity_comp_n != 0.0:
            self.left_force_pub = self.create_publisher(
                WrenchStamped, args.left_force_topic, 10)

        self.timer = self.create_timer(1.0 / max(args.rate_hz, 1.0), self._tick)

        self.get_logger().warn(
            f"bridge started: {args.left_pose_topic} -> {args.right_target_topic} "
            f"axis_map={args.axis_map!r} axis_scale={self.axis_scale} "
            f"max_offset_xyz={[round(v, 4) for v in self.max_offset]} m "
            f"max_speed={args.max_speed_mps} m/s clutch_mask={args.clutch_button_mask}"
        )
        if args.slave_gravity_comp_n != 0.0:
            self.get_logger().warn(
                f"slave gravity compensation {args.slave_gravity_comp_n:+.3f} N on "
                f"{'xyz'[args.slave_gravity_axis]}, helping the vertical axis carry its own weight"
            )
        if args.left_gravity_comp_n != 0.0:
            self.get_logger().warn(
                f"master (left) gravity compensation {args.left_gravity_comp_n:+.3f} N on "
                f"{'xyz'[args.left_gravity_axis]}, helping C2's vertical tracking carry the "
                "master's own weight"
            )
        if args.enable_c2:
            self.get_logger().warn(
                f"C2 ON: slave position -> {args.left_target_topic}, "
                f"left_max_offset={args.left_max_offset_m} m -- the master will now feel a "
                "spring pulling toward the slave's actual position, not just render whatever "
                "force_cmd/reflection publishes"
            )
        if args.reflect_enable:
            self.get_logger().warn(
                f"force reflection ON: gain={args.reflect_gain} N/m "
                f"deadband={args.reflect_deadband_m} m release_ratio={args.reflect_release_ratio} "
                f"max={args.reflect_max_n} N alpha={args.reflect_alpha} "
                f"velocity_comp={args.reflect_velocity_comp} s -> {args.master_force_topic}"
            )
            # The master node zeroes force on commands older than its
            # stale_timeout_s (0.05 s by default). Publishing below ~40 Hz makes
            # the reflected force stutter instead of holding.
            if args.rate_hz < 40.0:
                self.get_logger().error(
                    f"rate_hz={args.rate_hz} is too low for force reflection; "
                    "the master will drop force between ticks. Use 100 Hz or more."
                )

    # ---- subscriptions -------------------------------------------------

    def _left_cb(self, msg: PoseStamped) -> None:
        self.latest_left = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        if self.left_anchor is None:
            self.left_anchor = list(self.latest_left)
            self.get_logger().info(f"left anchor {self.left_anchor}")
            self._write_calibration()

    def _right_cb(self, msg: PoseStamped) -> None:
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # Slave speed, smoothed. This is what separates "lagging behind" from
        # "physically stopped", which is the difference between free motion and
        # an obstacle.
        if self.prev_right is not None:
            step = math.sqrt(sum((pos[i] - self.prev_right[i]) ** 2 for i in range(3)))
            sa = min(max(self.args.reflect_speed_alpha, 0.0), 1.0)
            self.slave_speed = (1.0 - sa) * self.slave_speed + sa * (step * 200.0)
        self.prev_right = list(pos)
        self.latest_right = pos
        if self.right_anchor is None:
            self.right_anchor = list(pos)
            self.get_logger().info(f"right anchor {self.right_anchor}")
            self._write_calibration()

    def _write_calibration(self) -> None:
        """Record the anchor pair to a shared file, so anything launched after
        the bridge (a sim twin, a diagnostic script) can read the exact
        starting reference the bridge is using, instead of re-sampling the
        live topics itself and risking a stale/racy read.

        Real hardware is never moved by this -- it is a log of wherever each
        device already was when its anchor was taken, written for diagnostics
        and for seeding sim twins (see run_slave_right_sim.sh /
        run_master_left_sim.sh). Two devices have independent physical
        origins, so there is no absolute pose they should "match"; what this
        adds is a single, well-defined snapshot of the pair, captured at a
        known moment (guaranteed fresh by the master -> bridge -> slave
        launch order), rather than each downstream script sampling the live
        topics again later and possibly getting a different, inconsistent
        reading.
        """
        path = os.path.expanduser(self.args.calibration_file)
        record: dict = {
            "timestamp": time.time(),
            "axis_map": self.args.axis_map,
            "axis_scale": self.axis_scale,
            "left_anchor": self.left_anchor,
            "right_anchor": self.right_anchor,
        }
        if self.left_anchor is not None and self.right_anchor is not None:
            # Diagnostic only, not used to move anything: what the right
            # anchor "would be" if the two devices shared one origin and the
            # current axis_map/scale were correct. A large gap here is a
            # useful data point for the still-open axis-mapping investigation
            # (see the axis-mapping-unresolved-bug memory note) even though it
            # is not, by itself, proof the mapping is wrong.
            mapped_left = apply_axis_map(self.axis_map, self.left_anchor)
            mapped_left = [mapped_left[i] * self.axis_scale[i] for i in range(3)]
            record["mapped_left_anchor"] = mapped_left
            record["right_minus_mapped_left"] = [
                self.right_anchor[i] - mapped_left[i] for i in range(3)
            ]
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(
                dir=os.path.dirname(path) or ".", prefix=".calib_", suffix=".tmp")
            with os.fdopen(fd, "w") as f:
                json.dump(record, f, indent=2)
            os.replace(tmp_path, path)
            self.get_logger().info(f"calibration written to {path}")
        except OSError as exc:
            self.get_logger().warn(f"could not write calibration file {path}: {exc}")

    def _buttons_cb(self, msg: Int32) -> None:
        self.left_buttons = int(msg.data)
        engaged = bool(self.left_buttons & int(self.args.clutch_button_mask))
        if engaged and not self.clutch_active:
            self.clutch_active = True
            self.get_logger().info("clutch engaged: slave frozen, move your hand freely")
        elif not engaged and self.clutch_active:
            self.clutch_active = False
            # Re-anchor both sides so the slave does not jump on re-engage.
            if self.latest_left is not None:
                self.left_anchor = list(self.latest_left)
            if self.last_target is not None:
                self.right_anchor = list(self.last_target)
            self.get_logger().info("clutch released: re-anchored")

    def _tracking_cb(self, msg: Vector3Stamped) -> None:
        self.tracking_error = [msg.vector.x, msg.vector.y, msg.vector.z]

    # ---- main loop -----------------------------------------------------

    def _tick(self) -> None:
        # Keep-alive: the right node drops force on stale commands.
        enable = Bool()
        enable.data = True
        self.enable_pub.publish(enable)
        self._publish_gravity_comp()
        self._publish_left_gravity_comp()

        if self.left_anchor is None or self.right_anchor is None or self.latest_left is None:
            self._report_waiting()
            return

        if not self.ready_announced:
            self.ready_announced = True
            self.get_logger().info("both devices are publishing, teleoperation live")
            if self.args.reflect_enable and self.count_subscribers(self.args.master_force_topic) == 0:
                self.get_logger().warn(
                    f"nothing subscribed to {self.args.master_force_topic}; the master will not "
                    "render force. Is run_master_left.sh running?"
                )

        if self.clutch_active:
            # Hold the last target so the slave stays put while you reposition.
            # Reflected force is released too: the hand is off task during a
            # clutch, and a live push would fight the repositioning.
            if self.last_target is not None:
                self._publish_target(self.last_target)
            self._decay_and_publish_force()
            if self.args.enable_c2 and self.left_target_pub is not None and self.last_left_target is not None:
                self._publish_left_target(self.last_left_target)
            return

        # Master velocity, smoothed, for the damping term. Same derivative-noise
        # caution as target_speed: this is differentiated encoder data.
        if self.prev_left is not None:
            sa = min(max(self.args.reflect_speed_alpha, 0.0), 1.0)
            for i in range(3):
                raw_v = (self.latest_left[i] - self.prev_left[i]) * self.args.rate_hz
                self.master_vel[i] = (1.0 - sa) * self.master_vel[i] + sa * raw_v
        self.prev_left = list(self.latest_left)

        delta = [self.latest_left[i] - self.left_anchor[i] for i in range(3)]
        if abs(delta[0]) < self.args.deadband_m:
            delta[0] = 0.0
        if abs(delta[1]) < self.args.deadband_m:
            delta[1] = 0.0
        if abs(delta[2]) < self.args.deadband_m:
            delta[2] = 0.0

        mapped = apply_axis_map(self.axis_map, delta)
        mapped = [mapped[i] * self.axis_scale[i] for i in range(3)]

        if str(self.args.max_offset_mode) == "norm":
            # Original behaviour. Shrinks the whole vector so its length fits,
            # which couples the axes but also keeps total travel bounded --
            # per-axis limits allow up to sqrt(3) times more combined reach.
            norm = math.sqrt(sum(v * v for v in mapped))
            if norm > self.args.max_offset_m and norm > 1e-9:
                shrink = self.args.max_offset_m / norm
                mapped = [v * shrink for v in mapped]
        else:
            # Clamp each axis on its own. A norm clamp couples the axes: run out
            # of travel on one and the others get squashed too, which reads as
            # the slave mysteriously refusing to follow.
            for i in range(3):
                limit = self.max_offset[i]
                if limit > 0.0:
                    if mapped[i] > limit:
                        mapped[i] = limit
                        self._note_limit(i)
                    elif mapped[i] < -limit:
                        mapped[i] = -limit
                        self._note_limit(i)

        target = [self.right_anchor[i] + mapped[i] for i in range(3)]

        # Absolute axes ignore the anchor and command the master's own
        # coordinate, so the slave holds the same height as your hand rather
        # than the same displacement. Clutching no longer re-zeroes these axes,
        # which is the point: the offset is what we are removing.
        if self.absolute_axes:
            abs_master = apply_axis_map(self.axis_map, self.latest_left)
            for i in self.absolute_axes:
                target[i] = abs_master[i] * self.axis_scale[i] + self.absolute_trim[i]

        target = self._rate_limit(target)

        # Commanded target speed, used to excuse the servo lag that free motion
        # always produces. Without this the operator feels drag in empty space.
        if self.prev_target is not None:
            step = math.sqrt(sum((target[i] - self.prev_target[i]) ** 2 for i in range(3)))
            raw_speed = step * self.args.rate_hz
            # Low-pass it. This is a position derivative at rate_hz, so encoder
            # quantisation lands straight in it. Unfiltered, the noise rides
            # through the threshold into the force and buzzes at loop rate --
            # and gets worse as reflect_gain rises.
            sa = min(max(self.args.reflect_speed_alpha, 0.0), 1.0)
            self.target_speed = (1.0 - sa) * self.target_speed + sa * raw_speed
        self.prev_target = list(target)

        for i in range(3):
            self.peak_cmd[i] = max(self.peak_cmd[i], abs(mapped[i]))
            if self.latest_right is not None and self.right_anchor is not None:
                self.peak_act[i] = max(
                    self.peak_act[i], abs(self.latest_right[i] - self.right_anchor[i]))

        self.last_target = target
        self._publish_target(target)
        self._update_force_reflection(target)
        if self.args.enable_c2 and self.left_target_pub is not None:
            self._update_c2()
        self._maybe_report(target)

    def _update_c2(self) -> None:
        """C2: slave position -> master target, the reverse direction of C3.

        Undoes axis_map/axis_scale to express the slave's own displacement
        (from its anchor) in the master's frame, then adds that to the
        master's anchor -- so the master's spring pulls toward wherever the
        slave actually is, not just wherever it was last commanded to go.
        This is what lets a stalled/obstructed slave be felt on the master
        without needing a real force sensor (that is what --reflect-enable
        already does via tracking error; this is the position-channel
        counterpart the channel table calls C2).
        """
        if self.latest_right is None:
            return
        slave_disp = [self.latest_right[i] - self.right_anchor[i] for i in range(3)]
        unscaled = [
            slave_disp[i] / self.axis_scale[i] if abs(self.axis_scale[i]) > 1e-9 else 0.0
            for i in range(3)
        ]
        master_disp = apply_axis_map_inverse(self.axis_map, unscaled)
        limit = float(self.args.left_max_offset_m)
        if limit > 0.0:
            for i in range(3):
                if master_disp[i] > limit:
                    master_disp[i] = limit
                elif master_disp[i] < -limit:
                    master_disp[i] = -limit
        left_target = [self.left_anchor[i] + master_disp[i] for i in range(3)]
        self.last_left_target = left_target
        self._publish_left_target(left_target)

    def _publish_left_target(self, target: list[float]) -> None:
        enable = Bool()
        enable.data = True
        self.left_enable_pub.publish(enable)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.master_frame
        msg.pose.position.x = float(target[0])
        msg.pose.position.y = float(target[1])
        msg.pose.position.z = float(target[2])
        msg.pose.orientation.w = 1.0
        self.left_target_pub.publish(msg)

    # ---- force reflection ----------------------------------------------

    def _slave_error(self, target: list[float]) -> Optional[list[float]]:
        """target - actual, in slave frame.

        Prefers the node's own tracking_error topic, which is computed inside the
        200 Hz servo loop against the exact target the servo saw. Falls back to
        differencing our commanded target with the reported pose.
        """
        if self.tracking_error is not None:
            return list(self.tracking_error)
        if self.latest_right is not None:
            return [target[i] - self.latest_right[i] for i in range(3)]
        return None

    def _update_force_reflection(self, target: list[float]) -> None:
        if not self.args.reflect_enable or self.master_force_pub is None:
            return

        error = self._slave_error(target)
        if error is None or self.clutch_active:
            self._decay_and_publish_force()
            return

        err_norm = math.sqrt(sum(v * v for v in error))

        # TRIGGER. The slave always lags a little while it accelerates, so the
        # threshold is the free-motion lag we expect at this speed plus a fixed
        # floor. Only error beyond that counts as an obstruction.
        threshold = self.args.reflect_deadband_m + self.args.reflect_velocity_comp * self.target_speed

        # Stall gate. An obstacle means the slave was told to move and did not.
        # Free motion carries error too -- the servo always lags -- but the slave
        # is still travelling, so error alone cannot tell the two apart. Requiring
        # the slave to be stopped is what makes free space genuinely force-free.
        if self.args.reflect_require_stall:
            keeping_up = max(
                float(self.args.reflect_stall_floor_mps),
                float(self.args.reflect_stall_ratio) * self.target_speed,
            )
            self.stalled = self.slave_speed < keeping_up
        else:
            self.stalled = True

        # Hysteresis: engage at the threshold, release lower down, so error
        # hovering at the boundary does not chatter the force on and off.
        if self.reflect_engaged:
            engaged = err_norm > threshold * self.args.reflect_release_ratio
        else:
            engaged = err_norm > threshold
        engaged = engaged and self.stalled

        if engaged != self.reflect_engaged:
            self.reflect_engaged = engaged
            if engaged:
                self.get_logger().info(
                    f"resistance detected: error={err_norm:.4f} m threshold={threshold:.4f} m"
                )
            else:
                self.get_logger().info("resistance cleared")

        if not engaged:
            self.reflect_excess = 0.0
            self._decay_and_publish_force()
            return

        excess = err_norm - threshold * self.args.reflect_release_ratio
        self.reflect_excess = max(excess, 0.0)

        # Direction: the error points where the slave still needs to go, which is
        # into the obstruction. Map it back to the master frame and negate, so the
        # push opposes the hand that is driving into the constraint.
        direction = [v / err_norm for v in error] if err_norm > 1e-9 else [0.0, 0.0, 0.0]
        magnitude = min(self.args.reflect_gain * self.reflect_excess, self.args.reflect_max_n)
        slave_frame = [-direction[i] * magnitude for i in range(3)]
        raw = apply_axis_map_inverse(self.axis_map, slave_frame)

        # Viscous damping on the master's own motion. OFF by default (0.0), and
        # that default is deliberate.
        #
        # Damping is the textbook cure for a ringing teleoperation loop, but it
        # only dissipates energy when the velocity it opposes is current. Here
        # master_vel is a low-passed derivative of encoder position, and the
        # force it produces arrives after another ~30 ms of round trip. Late
        # enough, "damping" is in phase with the motion and pumps the
        # oscillation instead. A closed-loop simulation of this exact code showed
        # peak-to-peak force rising from 0.007 N to 0.65 N as damping went from
        # 0 to 4.0 N s/m.
        #
        # Left available because the simulation's hand model is a guess and real
        # hardware may disagree. If you try it, start at 0.5 and stop
        # immediately if the vibration grows.
        damping = float(self.args.reflect_master_damping)
        if damping > 0.0:
            raw = [raw[i] - damping * self.master_vel[i] for i in range(3)]

        alpha = min(max(self.args.reflect_alpha, 0.0), 1.0)
        smoothed = [
            (1.0 - alpha) * self.reflect_force[i] + alpha * raw[i] for i in range(3)
        ]
        self.reflect_force = self._slew_limit(smoothed)
        self._publish_master_force(self.reflect_force)

    def _slew_limit(self, target_force: list[float]) -> list[float]:
        """Cap how fast the rendered force may change.

        The master is a haptic device: force moves your hand, your hand moves the
        master, that moves the slave target, which changes the error and the
        force. That loop closes through the hardware and is what oscillates. A
        slew cap bounds the loop gain at high frequency without softening the
        steady-state wall, which a lower gain would.
        """
        limit = float(self.args.reflect_slew_n_per_s) / max(self.args.rate_hz, 1.0)
        if limit <= 0.0:
            return target_force
        delta = [target_force[i] - self.reflect_force[i] for i in range(3)]
        mag = math.sqrt(sum(v * v for v in delta))
        if mag <= limit or mag < 1e-12:
            return target_force
        shrink = limit / mag
        return [self.reflect_force[i] + delta[i] * shrink for i in range(3)]

    def _decay_and_publish_force(self) -> None:
        """Ramp down rather than cutting to zero, so releasing a wall is smooth."""
        alpha = min(max(self.args.reflect_alpha, 0.0), 1.0)
        self.reflect_force = [(1.0 - alpha) * v for v in self.reflect_force]
        if all(abs(v) < 1e-4 for v in self.reflect_force):
            self.reflect_force = [0.0, 0.0, 0.0]
        self._publish_master_force(self.reflect_force)

    def _publish_master_force(self, force: list[float]) -> None:
        if self.master_force_pub is None:
            return
        if self.master_enable_pub is not None:
            enable = Bool()
            enable.data = True
            self.master_enable_pub.publish(enable)
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.master_frame
        msg.wrench.force.x = float(force[0])
        msg.wrench.force.y = float(force[1])
        msg.wrench.force.z = float(force[2])
        self.master_force_pub.publish(msg)

    def _note_limit(self, axis: int) -> None:
        """Warn, at most once per axis per 5 s, when travel is being clipped."""
        now = time.time()
        if now - self.limit_notice[axis] < 5.0:
            return
        self.limit_notice[axis] = now
        name = "xyz"[axis]
        self.get_logger().warn(
            f"slave {name} travel clipped at {self.max_offset[axis]*1000:.0f} mm. "
            f"The master is asking for more range than this axis is allowed. "
            f"Lower its AXIS_SCALE or raise its MAX_OFFSET_XYZ entry."
        )

    def _missing_topics(self) -> list[str]:
        missing = []
        if self.latest_left is None:
            missing.append(self.args.left_pose_topic)
        if self.right_anchor is None:
            missing.append(self.args.right_pose_topic)
        return missing

    def _report_waiting(self) -> None:
        """Say what is missing, once a second, and give up after the timeout."""
        now = time.time()
        waited = now - self.start_time
        missing = self._missing_topics()

        if now - self.last_wait_notice >= 1.0:
            self.last_wait_notice = now
            self.get_logger().warn(
                f"waiting {waited:.0f}s for {', '.join(missing)}"
            )

        timeout = float(self.args.wait_timeout_s)
        if timeout > 0.0 and waited > timeout:
            self.get_logger().error(
                f"no publisher on {', '.join(missing)} after {timeout:.0f}s. "
                "Start run_master_left.sh and run_slave_right.sh first, or raise WAIT_TIMEOUT_S."
            )
            raise SystemExit(1)

    def _rate_limit(self, target: list[float]) -> list[float]:
        if self.last_target is None or self.args.max_speed_mps <= 0.0:
            return target
        step_limit = self.args.max_speed_mps / max(self.args.rate_hz, 1.0)
        move = [target[i] - self.last_target[i] for i in range(3)]
        dist = math.sqrt(sum(v * v for v in move))
        if dist <= step_limit or dist < 1e-12:
            return target
        shrink = step_limit / dist
        return [self.last_target[i] + move[i] * shrink for i in range(3)]

    def _publish_gravity_comp(self) -> None:
        """Constant upward bias, republished every tick so it cannot go stale."""
        if self.slave_force_pub is None:
            return
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.slave_frame
        force = [0.0, 0.0, 0.0]
        force[self.args.slave_gravity_axis] = float(self.args.slave_gravity_comp_n)
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
        self.slave_force_pub.publish(msg)

    def _publish_left_gravity_comp(self) -> None:
        """Master-side counterpart to _publish_gravity_comp(), for C2. Same
        mechanism, same reasoning: a constant bias on the master's own
        force_cmd, so C2's spring is not spending its whole budget holding
        the master's own weight up before it can track the slave."""
        if self.left_force_pub is None:
            return
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.master_frame
        force = [0.0, 0.0, 0.0]
        force[self.args.left_gravity_axis] = float(self.args.left_gravity_comp_n)
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
        self.left_force_pub.publish(msg)

    def _publish_target(self, target: list[float]) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.right_frame
        msg.pose.position.x = float(target[0])
        msg.pose.position.y = float(target[1])
        msg.pose.position.z = float(target[2])
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def _maybe_report(self, target: list[float]) -> None:
        if not self.args.report:
            return
        now = time.time()
        if now - self.last_report < self.args.report_interval_s:
            return
        self.last_report = now
        line = f"[bridge] target=({target[0]:+.4f},{target[1]:+.4f},{target[2]:+.4f})"
        if self.latest_right is not None:
            lag = [target[i] - self.latest_right[i] for i in range(3)]
            lag_norm = math.sqrt(sum(v * v for v in lag))
            line += f" follow_error={lag_norm:.4f} m"
            if self.args.stiffness_n_per_m > 0.0:
                # Reaction force the spring is exerting to hold that error.
                # This is an estimate from a commanded spring, not a measurement.
                line += f" est_load={lag_norm * self.args.stiffness_n_per_m:.3f} N"
        if self.tracking_error is not None:
            te = self.tracking_error
            line += f" node_tracking_error=({te[0]:+.4f},{te[1]:+.4f},{te[2]:+.4f})"
        if self.args.reflect_enable:
            fn = math.sqrt(sum(v * v for v in self.reflect_force))
            state = "ENGAGED" if self.reflect_engaged else "free"
            thr = self.args.reflect_deadband_m + self.args.reflect_velocity_comp * self.target_speed
            line += (
                f" | reflect={state} threshold={thr:.4f}m"
                f" excess={self.reflect_excess:.4f}m force={fn:.3f}N"
                f" slave_v={self.slave_speed:.3f} cmd_v={self.target_speed:.3f}"
                f" {'STALLED' if self.stalled else 'moving '}"
            )
        self.get_logger().info(line)

        # Reach summary. Says plainly whether an axis stopped short, and why.
        parts = []
        for i, name in enumerate("xyz"):
            if self.peak_cmd[i] < 0.005:
                continue
            reached = self.peak_act[i] / self.peak_cmd[i] if self.peak_cmd[i] > 1e-6 else 0.0
            note = ""
            if self.peak_cmd[i] >= self.max_offset[i] * 0.98:
                note = "@LIMIT"          # our clamp is the ceiling
            elif reached < 0.75:
                note = "SHORT"           # commanded fine, slave could not get there
            parts.append(
                f"{name}: asked {self.peak_cmd[i]*1000:5.1f} got {self.peak_act[i]*1000:5.1f} "
                f"({reached:4.2f}){note}"
            )
        if parts:
            self.get_logger().info("  reach mm  " + "  |  ".join(parts))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--left-pose-topic", default="/touch_left/pose")
    parser.add_argument("--left-buttons-topic", default="/touch_left/buttons")
    parser.add_argument("--right-pose-topic", default="/touch_right/pose")
    parser.add_argument("--right-target-topic", default="/touch_right/target_pose")
    parser.add_argument("--right-enable-topic", default="/touch_right/enable")
    parser.add_argument("--enable-c2", type=_bool_arg, default=False,
                         help="publish a slave-position-derived target to --left-target-topic, "
                         "so the master's own spring pulls toward the slave's actual position "
                         "(C2 in the channel table). OFF by default: this changes what force "
                         "the operator's hand feels, so it must be turned on deliberately, and "
                         "run_master_left.sh must also be started with stiffness/damping set "
                         "for its own spring to have anything to render against.")
    parser.add_argument("--left-target-topic", default="/touch_left/target_pose")
    parser.add_argument("--left-enable-topic", default="/touch_left/enable")
    parser.add_argument("--left-max-offset-m", type=float, default=0.08,
                         help="per-axis clamp on how far C2 may pull the master's target from "
                         "its own anchor, independent of the slave-side max-offset clamps")
    parser.add_argument("--tracking-error-topic", default="/touch_right/tracking_error")
    parser.add_argument("--right-frame", default="touch_right_base")
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--axis-map", default="x,y,z", help="master delta -> slave xyz, e.g. '-x,z,y'")
    parser.add_argument("--axis-scale", default="",
                        help="Per-slave-axis scale, e.g. '1.0,0.6,1.0' to shrink vertical travel. "
                             "Empty uses --scale for all three.")
    parser.add_argument("--max-offset-m", type=float, default=0.12)
    parser.add_argument("--max-offset-xyz", default="",
                        help="Per-slave-axis travel limit in metres, e.g. '0.12,0.06,0.12'. "
                             "Empty uses --max-offset-m for all three.")
    parser.add_argument("--absolute-axes", default="",
                        help="Axes that should match the master's ABSOLUTE coordinate rather than "
                             "its displacement, e.g. 'y' to make the slave hold the same height. "
                             "Empty keeps the normal anchor-relative behaviour on every axis.")
    parser.add_argument("--absolute-trim-m", type=float, default=0.0,
                        help="Constant offset added to ALL absolute axes uniformly, to correct a "
                             "known mounting or base-height difference between the two devices. "
                             "Use --absolute-trim-xyz instead if different axes need different "
                             "trims (the common case, since the anchor gap this corrects is rarely "
                             "the same on every axis).")
    parser.add_argument("--absolute-trim-xyz", default="",
                        help="Per-axis version of --absolute-trim-m, e.g. '0,0.012,0.030'. Empty "
                             "uses --absolute-trim-m for all three, same fallback pattern as "
                             "--max-offset-xyz/--max-offset-m.")
    parser.add_argument("--max-offset-mode", choices=("axis", "norm"), default="axis",
                        help="'axis' limits each axis independently. 'norm' restores the original "
                             "vector-length clamp, which bounds total reach more tightly.")
    parser.add_argument("--max-speed-mps", type=float, default=0.5)
    parser.add_argument("--deadband-m", type=float, default=0.0002)
    parser.add_argument("--clutch-button-mask", type=int, default=2)
    parser.add_argument("--stiffness-n-per-m", type=float, default=35.0,
                        help="Right node spring gain, used only to estimate load from follow error.")
    parser.add_argument("--report", action="store_true")
    parser.add_argument("--report-interval-s", type=float, default=0.5)
    parser.add_argument("--slave-force-topic", default="/touch_right/force_cmd")
    parser.add_argument("--slave-frame", default="touch_right_base")
    parser.add_argument("--slave-gravity-comp-n", type=float, default=0.0,
                        help="Constant bias force on the slave, N. Offsets the arm's own weight so "
                             "the tracking spring does not spend its authority holding altitude. "
                             "Start around 0.15 and raise until the arm stops sagging.")
    parser.add_argument("--slave-gravity-axis", type=int, default=1, choices=(0, 1, 2),
                        help="Which slave axis is up. 1 (y) for a standard Touch.")
    parser.add_argument("--left-force-topic", default="/touch_left/force_cmd")
    parser.add_argument("--left-gravity-comp-n", type=float, default=0.0,
                        help="C2's counterpart to --slave-gravity-comp-n: constant bias force "
                        "on the master, N, so its own spring is not spending its authority "
                        "holding the master's weight up before it can track the slave "
                        "(--enable-c2). Start around 0.15 and raise until the arm stops sagging.")
    parser.add_argument("--left-gravity-axis", type=int, default=1, choices=(0, 1, 2),
                        help="Which master axis is up. 1 (y) for a standard Touch.")
    parser.add_argument("--calibration-file", default="~/dob_logs/teleop_calibration.json",
                         help="where to record the left/right anchor pair as they are "
                         "captured, for sim-twin seeding and axis-mapping diagnostics "
                         "(see _write_calibration)")
    parser.add_argument("--wait-timeout-s", type=float, default=30.0,
                        help="Give up if both device poses have not arrived in this long. 0 waits forever.")

    reflect = parser.add_argument_group("force reflection (master feels slave resistance)")
    reflect.add_argument("--reflect-enable", action="store_true",
                         help="Push back on the master when the slave cannot reach its target.")
    reflect.add_argument("--master-force-topic", default="/touch_left/force_cmd")
    reflect.add_argument("--master-enable-topic", default="/touch_left/enable")
    reflect.add_argument("--master-frame", default="touch_left_base")
    reflect.add_argument("--reflect-gain", type=float, default=25.0,
                         help="N per meter of excess tracking error. This is the felt stiffness "
                              "of the master. Approaching the slave spring gain makes the "
                              "master/slave loop oscillate; back off if it growls.")
    reflect.add_argument("--reflect-deadband-m", type=float, default=0.004,
                         help="Fixed part of the trigger threshold. Below this, no force.")
    reflect.add_argument("--reflect-velocity-comp", type=float, default=0.05,
                         help="Seconds. Threshold grows by this times the commanded target speed, "
                              "excusing the servo lag that free motion always produces.")
    reflect.add_argument("--reflect-release-ratio", type=float, default=0.6,
                         help="Hysteresis. Once engaged, release below threshold*ratio.")
    reflect.add_argument("--reflect-max-n", type=float, default=1.5,
                         help="Clamp on reflected force in N. Only bites if it is <= the master's "
                              "own max_force_n, so raise both together.")
    reflect.add_argument("--reflect-alpha", type=float, default=0.15,
                         help="Low-pass on the reflected force. Lower is smoother and more stable.")
    reflect.add_argument("--reflect-speed-alpha", type=float, default=0.15,
                         help="Low-pass on the commanded target speed that sets the trigger "
                              "threshold. Unfiltered it injects derivative noise into the force.")
    reflect.add_argument("--reflect-master-damping", type=float, default=0.0,
                         help="N s/m opposing the master's velocity while engaged. OFF by default: "
                              "the velocity estimate lags, so this can pump the oscillation rather "
                              "than damp it. If you try it, start at 0.5.")
    reflect.add_argument("--reflect-require-stall", type=_bool_arg, default=True,
                         help="Only render force when the slave is commanded to move but is not "
                              "moving. This is what keeps free space force-free.")
    reflect.add_argument("--reflect-stall-ratio", type=float, default=0.5,
                         help="Slave counts as stalled below this fraction of commanded speed.")
    reflect.add_argument("--reflect-stall-floor-mps", type=float, default=0.004,
                         help="Speed below which the slave counts as stopped regardless of command.")
    reflect.add_argument("--reflect-slew-n-per-s", type=float, default=8.0,
                         help="Max rate of change of rendered force, N/s. Bounds the hardware "
                              "loop at high frequency. 0 disables.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = MasterSlaveBridge(args)
    exit_code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Raised when the process is signalled (SIGTERM, or Ctrl-C under some
        # RMW builds). A clean exit, not a crash worth a traceback.
        pass
    except SystemExit as exc:
        exit_code = exc.code if isinstance(exc.code, int) else 1
    finally:
        try:
            # Zero the master force before dropping enable, so the stylus does
            # not hold a push on the way out.
            if node.master_force_pub is not None:
                node._publish_master_force([0.0, 0.0, 0.0])
                time.sleep(0.05)
            disable = Bool()
            disable.data = False
            node.enable_pub.publish(disable)
            if node.master_enable_pub is not None:
                node.master_enable_pub.publish(disable)
            time.sleep(0.1)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
