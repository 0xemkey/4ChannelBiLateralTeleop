"""Diagnostic (not a pytest assertion) for the C4 // TODO in touch_slave.cpp.

Reproduces test_joint_space_write_gated.py's test_write_armed_tracks_joint_target
scenario as exactly as possible (same SimNode params, same JOINT_TARGET_OFFSET/
DRIVE_S/HOLD_S, same hold/drive loop shape on the publish side), while also
capturing dob_diagnostics' joint_pd_* fields (added 2026-09-03 specifically for
this) to answer the TODO's own question: during elbow's wrong-direction ticks,
is joint_pd_gravity_ff_nm[2] (not the error term) what dictates
joint_pd_cmd_nm[2]'s sign, or is joint_pd_cmd_nm[2] instead pinned at the clamp
ceiling?

IMPORTANT, learned the hard way while building this: the publish loop's own
per-tick CPU profile matters. An earlier version of this script subscribed to
dob_diagnostics on the SAME node driving the tight publish loop, which -- even
with explicit rate-limiting -- measurably changed the outcome (a consistent
~+0.85 here vs. a consistent -0.60 to -0.65 cluster from the real pytest test,
same code, same scenario, five runs each). Root cause was never fully pinned
down (possibly CPU contention with the sim server's own real-time thread), but
the fix is structural, not a tuning knob: the diagnostic subscription now runs
on a SEPARATE node spun in a background thread via its own executor, and the
main thread's hold/drive loop is kept byte-for-byte identical in shape to
_drive_and_measure()'s (one publish + one spin_once(timeout_sec=0.02) on a
subscription-free node per tick).

There is also genuine, confirmed bimodal variance in the sim's own
uncontrolled settle before the hold phase even starts (confirmed by calling
_drive_and_measure() directly in a loop, bypassing pytest and this script
both): q_initial lands in one of at least two qualitatively different
clusters run to run. This script retries automatically until it lands in the
cluster known to fail (elbow initial angle < 0.81 rad; the passing cluster
sits at elbow ~0.83-0.85).

Run directly: python3 diag_c4_gravity_coupling.py
"""
from __future__ import annotations

import math
import sys
import threading
import time

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from sim_common import SimNode, unique_ns  # noqa: E402

from touch_haptics.msg import DobDiagnostics  # noqa: E402
from geometry_msgs.msg import Vector3Stamped  # noqa: E402

JOINT_TARGET_OFFSET = (0.05, -0.04, 0.03)
DRIVE_S = 2.5
HOLD_S = 1.5
ELBOW_CLAMP_NM = 1.5 * 0.135  # dob_torque_f_max_n=1.5 (this test's override) * l2
FAILING_CLUSTER_ELBOW_MAX = 0.81  # empirically: passing cluster is 0.83-0.85
MAX_ATTEMPTS = 15


def _run_once():
    """One trial. Returns (q_initial, q_final, q_target, diag_log)."""
    import rclpy
    from rclpy.node import Node as RclpyNode
    from rclpy.executors import SingleThreadedExecutor

    ns = unique_ns("diagc4")
    with SimNode(
        ns, device="Left",
        params={"joint_space_control": "true", "dob_torque_f_max_n": "1.5"},
    ) as sim:
        if not rclpy.ok():
            rclpy.init()

        # Publish/joint_angles node -- kept exactly as lean as
        # _drive_and_measure()'s `driver`, no other subscriptions, so the
        # main loop's per-tick cost matches the real test's.
        driver = RclpyNode(f"driver_{ns}")
        target_pub = driver.create_publisher(Vector3Stamped, sim.topic("joint_target"), 10)
        q_log: list = []
        driver.create_subscription(
            Vector3Stamped, sim.topic("joint_angles"), q_log.append, 50)

        # Diagnostics node -- separate node, separate executor, spun on a
        # background daemon thread so it never shares timing with the main
        # thread's publish loop.
        diag_node = RclpyNode(f"diag_{ns}")
        diag_log: list = []
        diag_node.create_subscription(
            DobDiagnostics, sim.topic("dob_diagnostics"), diag_log.append, 400)
        diag_executor = SingleThreadedExecutor()
        diag_executor.add_node(diag_node)
        diag_thread = threading.Thread(target=diag_executor.spin, daemon=True)
        diag_thread.start()

        try:
            deadline = time.time() + 2.0
            while not q_log and time.time() < deadline:
                rclpy.spin_once(driver, timeout_sec=0.05)
            if not q_log:
                raise TimeoutError("no joint_angles received")
            hold = (q_log[-1].vector.x, q_log[-1].vector.y, q_log[-1].vector.z)

            hold_deadline = time.time() + HOLD_S
            while time.time() < hold_deadline:
                msg = Vector3Stamped()
                msg.vector.x, msg.vector.y, msg.vector.z = hold
                target_pub.publish(msg)
                rclpy.spin_once(driver, timeout_sec=0.02)

            q_initial = (q_log[-1].vector.x, q_log[-1].vector.y, q_log[-1].vector.z)
            q_target = tuple(q_initial[i] + JOINT_TARGET_OFFSET[i] for i in range(3))

            diag_log.clear()  # only the drive phase matters for the table below
            drive_deadline = time.time() + DRIVE_S
            while time.time() < drive_deadline:
                msg = Vector3Stamped()
                msg.vector.x, msg.vector.y, msg.vector.z = q_target
                target_pub.publish(msg)
                rclpy.spin_once(driver, timeout_sec=0.02)

            q_final = (q_log[-1].vector.x, q_log[-1].vector.y, q_log[-1].vector.z)
            diag_snapshot = list(diag_log)
        finally:
            diag_executor.shutdown()
            diag_thread.join(timeout=2.0)
            driver.destroy_node()
            diag_node.destroy_node()

    return q_initial, q_final, q_target, diag_snapshot


def main() -> None:
    for attempt in range(1, MAX_ATTEMPTS + 1):
        q_initial, q_final, q_target, diag_log = _run_once()
        print(f"attempt {attempt}: q_initial={q_initial}")
        if q_initial[2] < FAILING_CLUSTER_ELBOW_MAX:
            print("  -> landed in the failing cluster, analyzing this run")
            break
        print("  -> passing cluster (elbow >= "
              f"{FAILING_CLUSTER_ELBOW_MAX}), retrying")
    else:
        print(f"never landed in the failing cluster after {MAX_ATTEMPTS} "
              "attempts -- analyzing the last run anyway")

    gap0 = math.sqrt(sum((q_target[i] - q_initial[i]) ** 2 for i in range(3)))
    gapf = math.sqrt(sum((q_target[i] - q_final[i]) ** 2 for i in range(3)))
    closed = 1.0 - gapf / gap0 if gap0 > 1e-6 else 0.0
    print()
    print(f"q_initial={q_initial}")
    print(f"q_target ={q_target}")
    print(f"q_final  ={q_final}")
    print(f"closed_fraction={closed:.3f}")
    print(f"diag samples during drive: {len(diag_log)}")
    print(f"elbow clamp ceiling at this test's params: {ELBOW_CLAMP_NM:.4f} N*m")
    print()

    # Per-tick elbow table: q3, gravity_ff, error_term, cmd (post-clamp),
    # write_ok, target-error sign, and whether q3 moved opposite that sign
    # since the previous sample.
    prev_q3 = None
    wrong_dir_ticks = []
    header = (
        f"{'t':>6} {'q3':>9} {'err':>9} {'grav_ff':>9} {'err_term':>9} "
        f"{'cmd':>9} {'ok':>4} {'dq3':>9} {'wrong_dir':>10}"
    )
    print(header)
    t0 = None
    for d in diag_log:
        if not d.joint_pd_active:
            continue
        stamp = d.header.stamp.sec + d.header.stamp.nanosec * 1e-9
        if t0 is None:
            t0 = stamp
        q3 = d.q_rad[2]
        err = q_target[2] - q3
        grav_ff = d.joint_pd_gravity_ff_nm[2]
        err_term = d.joint_pd_error_term_nm[2]
        cmd = d.joint_pd_cmd_nm[2]
        ok = d.joint_pd_write_ok
        dq3 = (q3 - prev_q3) if prev_q3 is not None else 0.0
        # Ignore single-encoder-quantum dither (~0.00041 rad on this arm) --
        # confirmed in an earlier run of this script to be quantization noise,
        # not genuine direction reversal.
        wrong_dir = (
            prev_q3 is not None and abs(dq3) > 0.0006 and (dq3 > 0) != (err > 0)
        )
        if wrong_dir:
            wrong_dir_ticks.append((stamp - t0, q3, err, grav_ff, err_term, cmd, ok))
        if wrong_dir or (len(diag_log) < 60) or (int((stamp - t0) * 200) % 10 == 0):
            print(
                f"{stamp - t0:6.3f} {q3:9.5f} {err:9.5f} {grav_ff:9.5f} "
                f"{err_term:9.5f} {cmd:9.5f} {str(ok):>4} {dq3:9.5f} "
                f"{'<--' if wrong_dir else '':>10}"
            )
        prev_q3 = q3

    print()
    print(f"total wrong-direction elbow ticks: {len(wrong_dir_ticks)} / "
          f"{sum(1 for d in diag_log if d.joint_pd_active)}")
    if wrong_dir_ticks:
        grav_dominant = sum(
            1 for (_, _, _, g, e, c, ok) in wrong_dir_ticks
            if ok and abs(g) > abs(e) and (abs(c - g) < abs(c - e))
        )
        pinned = sum(
            1 for (_, _, _, g, e, c, ok) in wrong_dir_ticks
            if ok and abs(abs(c) - ELBOW_CLAMP_NM) < 1e-4
        )
        refused = sum(1 for (_, _, _, _, _, _, ok) in wrong_dir_ticks if not ok)
        print(f"  of these: gravity-FF-dominant={grav_dominant}, "
              f"pinned-at-clamp={pinned}, write-refused={refused}")
    else:
        print("  no wrong-direction ticks recorded despite a failing "
              "closed_fraction -- elbow moved monotonically but too SLOWLY "
              "toward target, not backwards. Check the sparse table above for "
              "whether grav_ff or err_term dominates cmd throughout, and "
              "whether err_term stays small/noisy rather than growing with "
              "the (large, persistent) error.")


if __name__ == "__main__":
    main()
