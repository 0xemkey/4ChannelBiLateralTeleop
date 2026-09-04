#!/usr/bin/env python3
"""Offline check of the plan's transparency claim (Sec 3.5): does 4-channel
actually transmit more force on contact AND track better in free motion than
P-F, at matched architecture?

    transmitted impedance   |tau_e_hat| / |v_master|, during contact samples
                             (BilateralStatus.tau_e_hat_nm already nonzero only
                             where the observer's own deadband cleared it, so
                             "contact" here means norm > ~0, no extra threshold
                             is invented). Should read materially HIGHER under
                             pf/4ch than pp -- that is C1's signature.

    free-motion tracking error   |pose_slave - pose_master|, during samples
                             with tau_e_hat == 0. Should read LOWER under 4ch
                             than pf -- that is specifically C4's contribution
                             (feed-forward of hand torque tightens tracking
                             even where nothing is estimated as contact).

UNITS NOTE: this compares a joint-torque-derived quantity against a Cartesian
tip velocity, which are not physically commensurate as an impedance -- N*m
divided by m/s is not an SI impedance unit for this system. It is still valid
as a RELATIVE metric across architecture segments recorded with the same
formula, which is the only comparison this script (and the plan's claim) make.
Do not quote transmitted_impedance as an absolute physical number.

Reads /bilateral_status (bilateral_4ch.py's output, carries the architecture
label per-sample so a single bag spanning several presets can be split
automatically) plus /touch_left/pose and /touch_right/pose. Same rosbag2_py
structure as analyze_contact_separability.py -- no launch_testing.

Usage:
    python3 analyze_transparency.py <bag_dir> [--status-topic T] \\
        [--left-pose-topic T] [--right-pose-topic T]
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np


def read_bag(path: str, topics: dict):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise SystemExit(f"needs a sourced ROS 2 environment: {exc}")

    import os
    storage = "sqlite3"
    if not os.path.exists(os.path.join(path, "metadata.yaml")):
        print("NOTE: no metadata.yaml in the bag -- reading anyway with "
              "storage_id=sqlite3; the recorder was killed rather than stopped.")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=storage),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = set(topics.values())
    missing = wanted - set(types)
    if missing:
        raise SystemExit(f"bag is missing topic(s) {sorted(missing)}. Present: {sorted(types)}")

    msg_cls = {n: get_message(t) for n, t in types.items() if n in wanted}
    out = {name: [] for name in topics}
    inv = {v: k for k, v in topics.items()}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in inv:
            out[inv[topic]].append((t_ns, deserialize_message(data, msg_cls[topic])))
    for name in out:
        out[name].sort(key=lambda r: r[0])
    return out


def _nearest_preceding(t_ns: int, times: np.ndarray):
    """Index of the rightmost entry with times[i] <= t_ns, or None."""
    idx = int(np.searchsorted(times, t_ns, side="right")) - 1
    return idx if idx >= 0 else None


def _velocity_series(pose_samples):
    """[(t_ns, vx,vy,vz, |v|)] via backward finite difference of position."""
    out = []
    prev = None
    for t_ns, m in pose_samples:
        pos = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        if prev is not None:
            pt_ns, ppos = prev
            dt = (t_ns - pt_ns) / 1e9
            if dt > 1e-6:
                v = tuple((pos[i] - ppos[i]) / dt for i in range(3))
                out.append((t_ns, *v, math.sqrt(sum(c * c for c in v))))
        prev = (t_ns, pos)
    return out


def _pct(values, q):
    return float(np.percentile(values, q)) if values else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--status-topic", default="/bilateral_status")
    ap.add_argument("--left-pose-topic", default="/touch_left/pose")
    ap.add_argument("--right-pose-topic", default="/touch_right/pose")
    args = ap.parse_args()

    data = read_bag(args.bag, {
        "status": args.status_topic,
        "left_pose": args.left_pose_topic,
        "right_pose": args.right_pose_topic,
    })
    status = data["status"]
    left_pose = data["left_pose"]
    right_pose = data["right_pose"]

    print(f"bag           : {args.bag}")
    print(f"  {args.status_topic}: {len(status)} samples")
    print(f"  {args.left_pose_topic}: {len(left_pose)} samples")
    print(f"  {args.right_pose_topic}: {len(right_pose)} samples")
    if not status:
        raise SystemExit("no BilateralStatus samples -- was bilateral_4ch.py running "
                          "and publishing while this bag was recorded?")
    if not left_pose or not right_pose:
        raise SystemExit("no pose samples on one or both arms")

    left_vel = _velocity_series(left_pose)
    if not left_vel:
        raise SystemExit("could not compute a master velocity series (too few pose samples)")
    left_vel_t = np.array([r[0] for r in left_vel], dtype=np.int64)

    left_pose_t = np.array([t for t, _ in left_pose], dtype=np.int64)
    right_pose_t = np.array([t for t, _ in right_pose], dtype=np.int64)

    # Group by architecture, in case the bag spans a preset sweep.
    by_arch: dict = {}
    for t_ns, m in status:
        by_arch.setdefault(m.architecture, []).append((t_ns, m))

    print()
    header = (f"{'architecture':<12} {'n_contact':>9} {'Z_t (rel.)':>11} "
              f"{'n_free':>7} {'track_err_mm':>13} {'note'}")
    print(header)
    print("-" * len(header))

    for arch, samples in sorted(by_arch.items()):
        contact_ratios = []
        free_errors_mm = []
        for t_ns, m in samples:
            tau_e_norm = math.sqrt(sum(v * v for v in m.tau_e_hat_nm))

            vi = _nearest_preceding(t_ns, left_vel_t)
            v_master = left_vel[vi][4] if vi is not None else 0.0

            if tau_e_norm > 1e-9:
                if v_master > 1e-4:  # avoid dividing by near-zero velocity
                    contact_ratios.append(tau_e_norm / v_master)
            else:
                li = _nearest_preceding(t_ns, left_pose_t)
                ri = _nearest_preceding(t_ns, right_pose_t)
                if li is not None and ri is not None:
                    lp = left_pose[li][1].pose.position
                    rp = right_pose[ri][1].pose.position
                    err = math.sqrt(
                        (lp.x - rp.x) ** 2 + (lp.y - rp.y) ** 2 + (lp.z - rp.z) ** 2)
                    free_errors_mm.append(err * 1000.0)

        z_t = _pct(contact_ratios, 50)
        track = _pct(free_errors_mm, 50)
        note = ""
        if not contact_ratios:
            note = "no contact samples"
        elif not free_errors_mm:
            note = "no free-motion samples" if note == "" else note
        print(
            f"{arch:<12} {len(contact_ratios):>9} {z_t:>11.4f} "
            f"{len(free_errors_mm):>7} {track:>13.2f} {note}"
        )

    print()
    print("Expected ordering, per the plan's transparency claim (Sec 3.5):")
    print("  Z_t(rel.):        pp  <  pf  ~=  4ch      (C1's signature)")
    print("  track_err_mm:     pf  >  4ch               (C4's signature)")
    print("A violated ordering is a finding, not a script bug -- report it as-is.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
