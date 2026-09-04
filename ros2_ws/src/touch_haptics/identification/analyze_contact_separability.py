#!/usr/bin/env python3
"""Does contact separate from the disturbance floor? Answer, per joint, as a number.

This is the gating measurement for the whole 4-channel question. The offline
replay established what the floor IS (waist 0.065, shoulder 0.046, elbow 0.037
N*m at g_dis = 2*pi*5, gravity compensated). What it could not establish is
whether real contact rises clearly ABOVE that floor, because a replay of
free-motion captures contains no contact.

So: record a teleop session with /dob_diagnostics and /contact_annotation, and
compare |tau_dis_hat| in the annotated-contact samples against the annotated-
clear ones. The ratio between them is the answer.

WHAT THE ANSWER MEANS
---------------------
  ratio near 1     the observer cannot tell contact from its own noise floor.
                   More identification will not fix that -- it is an SNR limit,
                   and the 4-channel plan needs rethinking rather than more data.
  ratio >> 1       contact is separable. The remaining steps (right-arm
                   identification, torque calibration, opening the gates) are
                   worth the hardware time.

Reported PER JOINT, never averaged into one verdict: the three joints have
different M_n and different floor-to-authority ratios (40 / 57 / 46 % at
f_max = 0.6 N), and there is no reason to expect them to behave alike.

WHAT THIS SCRIPT DELIBERATELY DOES NOT DO
-----------------------------------------
It does not tune threshold_nm or g_dis. It reports separation and false-positive
counts; deciding what those imply is the operator's call, consistent with every
other script in this pipeline.

Usage:
    python3 analyze_contact_separability.py <bag_dir> [--dob-topic T] [--ann-topic T]
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

# Floor from tools/dob_replay_cpp.cpp at 5 Hz, gravity compensated. The live
# unloaded segment should land near these; a large gap is itself a finding.
REPLAY_FLOOR_NM = {"waist": 0.06541, "shoulder": 0.04582, "elbow": 0.03731}
JOINTS = ("waist", "shoulder", "elbow")


def read_bag(path: str, dob_topic: str, ann_topic: str):
    """Return (dob_samples, ann_samples), each a list of (t_ns, payload)."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise SystemExit(f"needs a sourced ROS 2 environment: {exc}")

    # storage_id is given explicitly rather than left blank. Autodetection
    # needs metadata.yaml, which a recorder that was killed rather than
    # stopped never wrote -- and then the failure reads as "no plugin found",
    # which sounds like a missing dependency rather than a truncated bag.
    import os
    storage = "sqlite3"
    if not os.path.exists(os.path.join(path, "metadata.yaml")):
        print("NOTE: no metadata.yaml in the bag -- it was killed rather than")
        print("      stopped cleanly. Reading anyway with storage_id=sqlite3;")
        print("      the data is intact, only the index is missing.")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=storage),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for want in (dob_topic, ann_topic):
        if want not in types:
            raise SystemExit(
                f"bag has no topic {want!r}. Present: {sorted(types)}")

    msg_cls = {n: get_message(t) for n, t in types.items()}
    dob, ann = [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == dob_topic:
            dob.append((t_ns, deserialize_message(data, msg_cls[topic])))
        elif topic == ann_topic:
            ann.append((t_ns, deserialize_message(data, msg_cls[topic])))
    dob.sort(key=lambda r: r[0])
    ann.sort(key=lambda r: r[0])
    return dob, ann


def label_samples(dob, ann):
    """Label each DOB sample by the most recent annotation AT OR BEFORE it.

    Nearest-preceding, not nearest-in-either-direction. A sample belongs to
    whatever state was last declared; letting a sample be pulled into a state
    that had not started yet would smear the contact boundary in both
    directions and inflate the very ratio this script measures.

    Samples before the first annotation are returned as None -- EXCLUDED, not
    guessed. At session start the operator may not have begun annotating, and
    labelling that stretch 'clear' by default would silently fold unknown data
    into the floor estimate.
    """
    if not ann:
        raise SystemExit("no annotation messages in the bag -- nothing to split on")

    a_t = np.array([t for t, _ in ann], dtype=np.int64)
    a_v = np.array([bool(m.data) for _, m in ann])
    first = a_t[0]

    labels = []
    for t_ns, _ in dob:
        if t_ns < first:
            labels.append(None)
            continue
        # rightmost annotation with a_t <= t_ns
        labels.append(bool(a_v[np.searchsorted(a_t, t_ns, side="right") - 1]))
    return labels


def pct(v, q):
    return float(np.percentile(v, q)) if len(v) else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--dob-topic", default="/touch_left/dob_diagnostics")
    ap.add_argument("--ann-topic", default="/contact_annotation")
    args = ap.parse_args()

    dob, ann = read_bag(args.bag, args.dob_topic, args.ann_topic)
    print(f"bag       : {args.bag}")
    print(f"  {args.dob_topic}: {len(dob)} samples")
    print(f"  {args.ann_topic}: {len(ann)} samples")
    if not dob:
        raise SystemExit("no DOB samples")

    span = (dob[-1][0] - dob[0][0]) / 1e9
    print(f"  duration : {span:.1f} s  ({len(dob)/max(span,1e-9):.0f} Hz)")

    n_conf = sum(1 for _, m in dob if getattr(m, "configured", False))
    if n_conf != len(dob):
        print(f"\n  WARNING: only {n_conf}/{len(dob)} samples have configured=true.")
        print("  Unconfigured samples carry zeros and would drag every statistic")
        print("  toward zero; they are excluded below.")

    labels = label_samples(dob, ann)
    n_pre = sum(1 for l in labels if l is None)
    if n_pre:
        print(f"\n  {n_pre} sample(s) before the first annotation -- EXCLUDED "
              f"(unlabelled, not assumed clear)")

    clear_idx = [i for i, l in enumerate(labels)
                 if l is False and getattr(dob[i][1], "configured", False)]
    cont_idx = [i for i, l in enumerate(labels)
                if l is True and getattr(dob[i][1], "configured", False)]

    print(f"\n  unloaded : {len(clear_idx)} samples")
    print(f"  contact  : {len(cont_idx)} samples")
    if not clear_idx or not cont_idx:
        raise SystemExit(
            "need samples in BOTH states -- toggle the annotation during the run")

    # Count annotated contact episodes; one long press is much weaker evidence
    # than several repeats, and the reader should be able to see which it was.
    a_v = [bool(m.data) for _, m in ann]
    episodes = sum(1 for i in range(1, len(a_v)) if a_v[i] and not a_v[i - 1])
    print(f"  contact episodes annotated: {episodes}")

    # Sanity gate on the annotation stream itself. A human toggling a key
    # produces a handful of episodes over a session. Hundreds means the stream
    # is alternating sample-to-sample, and the only thing that does that is TWO
    # publishers disagreeing -- e.g. a leftover contact_annotate.py left running
    # alongside a second one. That happened during this harness's own smoke
    # test, and the segmentation it produces is meaningless while still looking
    # numerically plausible, so it is checked rather than trusted.
    if episodes > 0.02 * max(len(a_v), 1) and episodes > 20:
        print()
        print("  *** ANNOTATION STREAM LOOKS CONTAMINATED ***")
        print(f"  {episodes} rising edges in {len(a_v)} messages -- far more than a")
        print("  human can toggle. Almost certainly more than one publisher on")
        print("  /contact_annotation. Check with:")
        print("      ros2 topic info /contact_annotation      # expect exactly 1")
        print("      pgrep -af '[c]ontact_annotate'           # bracket avoids self-match")
        print("  Segmentation below is NOT trustworthy until that is fixed.")
        print()

    results = {}
    print("\n=== |tau_dis_hat| by segment, N*m ===")
    print(f"{'joint':>9} {'unload p99':>11} {'unload max':>11} "
          f"{'contact p99':>12} {'contact max':>12} {'ratio':>8} {'vs replay':>10}")
    for j, name in enumerate(JOINTS):
        c = np.array([abs(dob[i][1].tau_dis_hat_nm[j]) for i in clear_idx])
        k = np.array([abs(dob[i][1].tau_dis_hat_nm[j]) for i in cont_idx])
        c99, k99 = pct(c, 99), pct(k, 99)
        ratio = k99 / c99 if c99 > 1e-12 else float("nan")
        vs = c99 / REPLAY_FLOOR_NM[name]
        results[name] = dict(c99=c99, cmax=float(c.max()), k99=k99,
                             kmax=float(k.max()), ratio=ratio, vs_replay=vs)
        print(f"{name:>9} {c99:11.5f} {c.max():11.5f} {k99:12.5f} {k.max():12.5f} "
              f"{ratio:7.2f}x {vs:9.2f}x")

    print("\n  'vs replay' compares the live unloaded p99 against the offline")
    print("  prediction (waist 0.0654, shoulder 0.0458, elbow 0.0373 N*m).")
    print("  Near 1.0 means the replay predicted live behaviour well. Far from")
    print("  1.0 means live conditions differ from the identification captures")
    print("  -- which would undercut thresholds derived from that replay.")

    # Gate behaviour: firing during UNLOADED motion is a false positive, and
    # that count is as informative as the ratio.
    print("\n=== contact_suspected gate ===")
    print(f"{'joint':>9} {'unloaded fires':>16} {'contact fires':>16}")
    for j, name in enumerate(JOINTS):
        fp = sum(1 for i in clear_idx if dob[i][1].contact_suspected[j])
        tp = sum(1 for i in cont_idx if dob[i][1].contact_suspected[j])
        results[name]["fp"] = (fp, len(clear_idx))
        results[name]["tp"] = (tp, len(cont_idx))
        print(f"{name:>9} {fp:>7}/{len(clear_idx):<8} {tp:>7}/{len(cont_idx):<8}")

    print("\n=== verdict, per joint ===")
    for name in JOINTS:
        r = results[name]
        fp, fpn = r["fp"]
        tp, tpn = r["tp"]
        fp_rate = 100.0 * fp / max(fpn, 1)
        tp_rate = 100.0 * tp / max(tpn, 1)

        if not np.isfinite(r["ratio"]):
            head = "indeterminate -- unloaded floor is ~0, ratio undefined"
        elif r["ratio"] >= 3.0:
            head = f"separation {r['ratio']:.1f}x -- contact clearly clears the floor"
        elif r["ratio"] >= 2.0:
            head = f"separation {r['ratio']:.1f}x -- usable, with margin to spare"
        elif r["ratio"] >= 1.5:
            head = (f"separation {r['ratio']:.1f}x -- MARGINAL. Contact peaks only "
                    f"modestly clear the unloaded p99")
        else:
            head = (f"separation {r['ratio']:.1f}x -- NO usable separation. Contact "
                    f"is not distinguishable from this joint's own floor")
        print(f"\n  {name}: {head}.")
        print(f"    gate fired {fp}/{fpn} ({fp_rate:.1f}%) while UNLOADED, "
              f"{tp}/{tpn} ({tp_rate:.1f}%) during contact.")
        if fp_rate > 5.0:
            print(f"    The unloaded firing rate is high: threshold_nm={name} may be")
            print("    too low for live conditions, OR this joint's floor is close to")
            print("    an SNR limit. This data alone cannot separate those two.")
        if tp_rate < 20.0 and np.isfinite(r["ratio"]):
            print("    Gate rarely fired during annotated contact -- either the")
            print("    contact was lighter than the threshold, or the threshold is")
            print("    set above the achievable contact signal.")

    print("\n" + "=" * 62)
    print("No threshold_nm or g_dis value was tuned or written by this script.")
    print("What these ratios imply for the 6-step path is the operator's call.")
    print("=" * 62)
    return 0


if __name__ == "__main__":
    sys.exit(main())
