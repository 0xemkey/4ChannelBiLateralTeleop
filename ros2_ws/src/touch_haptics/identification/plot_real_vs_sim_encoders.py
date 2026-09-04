#!/usr/bin/env python3
"""Real vs. visual (simulated) encoder comparison plots.

Phase 1 of the follow-on plan to docs/FOUR_CHANNEL_TODO.md: before trying to
separate the DOB's noise floor into friction / gravity / environmental parts,
capture what the RAW real-vs-simulated divergence looks like. The "visual"
encoder is touch_slave_sim (see ../sim/), driven by the exact same
target_pose/force_cmd commands as the real device (run_master_left_sim.sh /
run_slave_right_sim.sh remap their inputs to the real topics, their outputs to
a parallel /touch_*_sim/ tree). Because both twins see identical commands,
whatever they disagree on afterwards is friction, gravity-model error, base
tilt, and genuine contact, all still mixed together -- deliberately not
separated here. See analyze_contact_separability.py / fit_friction.py /
fit_gravity.py for that next step.

Reads one rosbag2 recording (record_real_vs_sim_bag.sh's output) and, for
each arm present, plots position and Cartesian force over time -- 3D and
X/Y/Z separately -- real and visual overlaid. Position is zero-referenced to
each stream's OWN first sample (see the comment at plot_arm) so a mismatched
sim/real home pose does not masquerade as noise.

CONTACT EPISODES. If /contact_annotation is in the bag (see
contact_annotate.py), each contiguous CONTACT run is treated as one "case" --
e.g. one of your deliberate force applications -- and given its own color in
every 3D plot (real = solid, visual = dashed, same hue per episode). Free
motion between episodes is drawn in light gray for context. A per-episode
peak-force bar chart is also produced.

DEGENERATE SERIES. A stream that never moves/never renders force (e.g. the
master when nothing commands it) collapses to a single point. A plain line
plot draws that as literally nothing, which reads as "broken" rather than
"flat at zero". Such series are drawn as a marked point with a text label
instead.

Usage:
    python3 plot_real_vs_sim_encoders.py <bag_dir> [--out-dir DIR]
                                          [--arms left,right]

Output: PNG files under <out-dir>/ (default: <bag_dir>/plots/).
Kept local -- nothing here is uploaded or published.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

EPISODE_COLORS = (
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
)
CLEAR_COLOR_REAL = "#b0b0b0"
CLEAR_COLOR_SIM = "#d8d8d8"
DEGENERATE_POS_EPS_MM = 0.5
DEGENERATE_FORCE_EPS_N = 0.01


def read_bag(path: str, wanted_topics: list[str]):
    """Return {topic: (t_s ndarray, msgs list)}, t_s relative to the first
    sample across ALL requested topics so every plot shares one time origin.
    Missing topics are silently omitted (an arm's sim twin, or the annotation
    topic, may not have been recorded for a given session).
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise SystemExit(f"needs a sourced ROS 2 environment: {exc}")

    storage = "sqlite3"
    if not os.path.exists(os.path.join(path, "metadata.yaml")):
        print("NOTE: no metadata.yaml -- bag was killed rather than stopped "
              "cleanly. Reading anyway with storage_id=sqlite3.")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=storage),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    present = [t for t in wanted_topics if t in types]
    missing = [t for t in wanted_topics if t not in types]
    if missing:
        print(f"NOTE: bag has no topic(s) {missing} -- skipping those plots.")
    if not present:
        raise SystemExit(f"none of {wanted_topics} present in {path!r}")

    msg_cls = {n: get_message(t) for n, t in types.items() if n in present}
    raw: dict[str, list[tuple[int, object]]] = {t: [] for t in present}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in raw:
            raw[topic].append((t_ns, deserialize_message(data, msg_cls[topic])))

    t0 = min(t for series in raw.values() for t, _ in series) if any(raw.values()) else 0
    out = {}
    for topic, series in raw.items():
        series.sort(key=lambda r: r[0])
        t_s = np.array([(t - t0) * 1e-9 for t, _ in series])
        out[topic] = (t_s, [m for _, m in series])
    return out


def _xyz_from_pose(msgs) -> np.ndarray:
    return np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z]
                      for m in msgs])


def _xyz_from_wrench(msgs) -> np.ndarray:
    return np.array([[m.wrench.force.x, m.wrench.force.y, m.wrench.force.z]
                      for m in msgs])


AXIS_NAMES = ("x", "y", "z")
AXIS_COLORS_REAL = ("#c0392b", "#27ae60", "#2980b9")
AXIS_COLORS_SIM = ("#f1948a", "#82e0aa", "#85c1e9")


# ---------------------------------------------------------------------------
# Contact episodes
# ---------------------------------------------------------------------------

def build_episodes(ann_t: np.ndarray, ann_v: np.ndarray,
                    merge_gap_s: float = 0.3) -> list[tuple[float, float]]:
    """Contiguous CONTACT (True) runs from /contact_annotation, as (t_start,
    t_end) pairs. One run per deliberate force application, in order.

    Episodes separated by less than merge_gap_s are merged into one. Holding
    ENTER in contact_annotate.py rather than a clean press/release can trigger
    terminal key-repeat, which shows up as a burst of sub-second CONTACT/CLEAR
    toggles a few tens of ms apart -- not real separate presses. A real
    deliberate release-then-press-again is expected to be at least on the
    order of a second, so a short default threshold catches the repeat-burst
    case without merging genuinely distinct presses together.
    """
    raw = []
    in_ep = False
    ep_start = 0.0
    for t, v in zip(ann_t, ann_v):
        if v and not in_ep:
            in_ep = True
            ep_start = t
        elif not v and in_ep:
            in_ep = False
            raw.append((ep_start, t))
    if in_ep:
        raw.append((ep_start, ann_t[-1]))

    if not raw:
        return raw
    merged = [raw[0]]
    for t0, t1 in raw[1:]:
        prev_t0, prev_t1 = merged[-1]
        if t0 - prev_t1 < merge_gap_s:
            merged[-1] = (prev_t0, t1)
        else:
            merged.append((t0, t1))
    return merged


def episode_index_for(t_array: np.ndarray, episodes: list[tuple[float, float]]) -> np.ndarray:
    """-1 for clear/no-annotation, else the episode's position in the list."""
    idx = np.full(len(t_array), -1, dtype=int)
    for k, (t0, t1) in enumerate(episodes):
        idx[(t_array >= t0) & (t_array <= t1)] = k
    return idx


def _segments(idx: np.ndarray) -> list[tuple[int, int, int]]:
    """(episode_id, start, end) contiguous slices of idx, end exclusive,
    extended by one sample so adjoining segments still connect visually."""
    if len(idx) == 0:
        return []
    out = []
    start = 0
    cur = idx[0]
    for i in range(1, len(idx)):
        if idx[i] != cur:
            out.append((cur, start, min(i + 1, len(idx))))
            start = i
            cur = idx[i]
    out.append((cur, start, len(idx)))
    return out


def _episode_color(k: int) -> str:
    return EPISODE_COLORS[k % len(EPISODE_COLORS)]


def _is_degenerate(xyz: np.ndarray, eps: float) -> bool:
    if xyz is None or len(xyz) == 0:
        return True
    return bool(np.all(np.ptp(xyz, axis=0) < eps))


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def _plot_3d(fig_path, xyz_real, xyz_sim, label, title,
             ep_idx_real=None, ep_idx_sim=None, eps=DEGENERATE_POS_EPS_MM):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(7, 6.5))
    ax = fig.add_subplot(111, projection="3d")
    handles, labels = [], []

    def draw(xyz, ep_idx, is_real):
        if xyz is None or not len(xyz):
            return
        if _is_degenerate(xyz, eps):
            m = ax.scatter([xyz[0, 0]], [xyz[0, 1]], [xyz[0, 2]],
                            color="#c0392b" if is_real else "#f1948a",
                            marker="o" if is_real else "x", s=60,
                            label=f"{'real' if is_real else 'visual (sim)'} {label} (no change)")
            handles.append(m); labels.append(m.get_label())
            return
        linestyle = "-" if is_real else "--"
        if ep_idx is None:
            color = "#c0392b" if is_real else "#f1948a"
            (ln,) = ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=color,
                             linestyle=linestyle, label=f"{'real' if is_real else 'visual (sim)'} {label}")
            handles.append(ln); labels.append(ln.get_label())
            return
        seen = set()
        clear_color = CLEAR_COLOR_REAL if is_real else CLEAR_COLOR_SIM
        for ep_id, s, e in _segments(ep_idx):
            color = clear_color if ep_id < 0 else _episode_color(ep_id)
            lw = 1.0 if ep_id < 0 else 2.2
            alpha = 0.6 if ep_id < 0 else 1.0
            tag = "clear" if ep_id < 0 else f"press {ep_id + 1}"
            lbl = f"{'real' if is_real else 'visual'} {tag}"
            (ln,) = ax.plot(xyz[s:e, 0], xyz[s:e, 1], xyz[s:e, 2], color=color,
                             linestyle=linestyle, linewidth=lw, alpha=alpha,
                             label=lbl if lbl not in seen else None)
            if lbl not in seen:
                handles.append(ln); labels.append(lbl); seen.add(lbl)

    draw(xyz_real, ep_idx_real, True)
    draw(xyz_sim, ep_idx_sim, False)

    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_title(title)
    if handles:
        ax.legend(handles, labels, fontsize=7, ncol=2, loc="upper left",
                   bbox_to_anchor=(1.02, 1.0))
    fig.tight_layout()
    fig.savefig(fig_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _plot_xyz(fig_path, t_real, xyz_real, t_sim, xyz_sim, ylabel, title,
              episodes=None, eps=DEGENERATE_POS_EPS_MM):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(9, 7.5), sharex=True)
    real_flat = _is_degenerate(xyz_real, eps)
    sim_flat = _is_degenerate(xyz_sim, eps) if xyz_sim is not None else True

    for i, ax in enumerate(axes):
        if episodes:
            for k, (t0, t1) in enumerate(episodes):
                ax.axvspan(t0, t1, color=_episode_color(k), alpha=0.12)
        if len(xyz_real):
            ax.plot(t_real, xyz_real[:, i], color=AXIS_COLORS_REAL[i],
                    label="real" + (" (flat)" if real_flat else ""))
        if xyz_sim is not None and len(xyz_sim):
            ax.plot(t_sim, xyz_sim[:, i], color=AXIS_COLORS_SIM[i],
                    linestyle="--", label="visual (sim)" + (" (flat)" if sim_flat else ""))
        ax.set_ylabel(f"{ylabel} {AXIS_NAMES[i]}")
        ax.legend(loc="upper right", fontsize=8)
    if episodes:
        for k, (t0, _t1) in enumerate(episodes):
            axes[0].text(t0, axes[0].get_ylim()[1], f" press {k + 1}",
                         fontsize=7, va="top", color=_episode_color(k))
    axes[-1].set_xlabel("time (s)")
    axes[0].set_title(title)
    fig.tight_layout()
    fig.savefig(fig_path, dpi=150)
    plt.close(fig)


def _plot_noise(fig_path, t_pos, pos_noise_mm, t_force, force_noise_n, title):
    """Combined per-axis panel: position residual and force residual overlaid
    on a twin y-axis, one subplot per axis -- the "sum of both, point-to-point
    position difference and force calculated from encoder" panel from the
    plan."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t_pos, pos_noise_mm[:, i], color="#8e44ad",
                label="position noise (mm)")
        ax.set_ylabel(f"{AXIS_NAMES[i]} pos noise (mm)", color="#8e44ad")
        ax2 = ax.twinx()
        ax2.plot(t_force, force_noise_n[:, i], color="#d35400",
                 label="force noise (N)")
        ax2.set_ylabel(f"{AXIS_NAMES[i]} force noise (N)", color="#d35400")
    axes[-1].set_xlabel("time (s)")
    axes[0].set_title(title)
    fig.tight_layout()
    fig.savefig(fig_path, dpi=150)
    plt.close(fig)


def _plot_force_by_episode(fig_path, t_force_real, xyz_force_real,
                            t_force_sim, xyz_force_sim, episodes, title):
    """Bar chart: peak |F| (N) per contact episode, real vs visual."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not episodes or not len(xyz_force_real):
        return

    norm_real = np.linalg.norm(xyz_force_real, axis=1) if len(xyz_force_real) else np.empty(0)
    norm_sim = (np.linalg.norm(xyz_force_sim, axis=1)
                if xyz_force_sim is not None and len(xyz_force_sim) else np.empty(0))

    peaks_real, peaks_sim = [], []
    for t0, t1 in episodes:
        m_real = (t_force_real >= t0) & (t_force_real <= t1)
        peaks_real.append(float(norm_real[m_real].max()) if m_real.any() else 0.0)
        if len(norm_sim):
            m_sim = (t_force_sim >= t0) & (t_force_sim <= t1)
            peaks_sim.append(float(norm_sim[m_sim].max()) if m_sim.any() else 0.0)
        else:
            peaks_sim.append(0.0)

    n = len(episodes)
    x = np.arange(n)
    w = 0.35
    fig, ax = plt.subplots(figsize=(max(6, n * 1.2), 5))
    bars_r = ax.bar(x - w / 2, peaks_real, w, label="real", color="#c0392b")
    bars_s = ax.bar(x + w / 2, peaks_sim, w, label="visual (sim)", color="#f1948a")
    for k in range(n):
        ax.axvspan(x[k] - 0.5, x[k] + 0.5, color=_episode_color(k), alpha=0.08)
    ax.set_xticks(x)
    ax.set_xticklabels([f"press {k + 1}" for k in range(n)])
    ax.set_ylabel("peak |F| (N)")
    ax.set_title(title)
    ax.legend()
    ax.bar_label(bars_r, fmt="%.2f", fontsize=7)
    ax.bar_label(bars_s, fmt="%.2f", fontsize=7)
    fig.tight_layout()
    fig.savefig(fig_path, dpi=150)
    plt.close(fig)


def _interp_to(t_ref: np.ndarray, t_src: np.ndarray, xyz_src: np.ndarray) -> np.ndarray:
    """Resample xyz_src (sampled at t_src) onto t_ref by per-axis linear
    interpolation. Real and sim run as separate processes and are not
    tick-synchronized, so this is required before any point-to-point
    difference is meaningful."""
    if len(t_src) < 2 or len(t_ref) == 0:
        return np.zeros((len(t_ref), 3))
    out = np.empty((len(t_ref), 3))
    for i in range(3):
        out[:, i] = np.interp(t_ref, t_src, xyz_src[:, i])
    return out


def plot_arm(bag_data: dict, arm: str, out_dir: str, is_master: bool,
             episodes: list[tuple[float, float]]) -> None:
    pose_topic = f"/touch_{arm}/pose"
    pose_sim_topic = f"/touch_{arm}_sim/pose"
    force_topic = f"/touch_{arm}/applied_force"
    force_sim_topic = f"/touch_{arm}_sim/applied_force"

    if pose_topic not in bag_data:
        print(f"[{arm}] no {pose_topic} in bag -- skipping this arm entirely")
        return

    t_pos_real, pose_real_msgs = bag_data[pose_topic]
    xyz_pos_real = _xyz_from_pose(pose_real_msgs) * 1000.0  # m -> mm
    if len(xyz_pos_real):
        xyz_pos_real = xyz_pos_real - xyz_pos_real[0]

    t_pos_sim = np.empty(0)
    xyz_pos_sim = None
    if pose_sim_topic in bag_data:
        t_pos_sim, pose_sim_msgs = bag_data[pose_sim_topic]
        xyz_pos_sim = _xyz_from_pose(pose_sim_msgs) * 1000.0
        if len(xyz_pos_sim):
            # Zero-referenced to each stream's OWN first sample, not a shared
            # origin. The sim always resets to a fixed internal home pose
            # (ArmParams::reset(), q = {0, 0.45, 0.9}); the real device starts
            # wherever it physically was sitting. Comparing absolute Cartesian
            # position would report that placement mismatch as "noise" -- a
            # rigid offset, not dynamics. Comparing CHANGES from each arm's own
            # start (what was actually asked for) removes it.
            xyz_pos_sim = xyz_pos_sim - xyz_pos_sim[0]

    t_force_real = np.empty(0)
    xyz_force_real = np.empty((0, 3))
    if force_topic in bag_data:
        t_force_real, force_real_msgs = bag_data[force_topic]
        xyz_force_real = _xyz_from_wrench(force_real_msgs)

    t_force_sim = np.empty(0)
    xyz_force_sim = None
    if force_sim_topic in bag_data:
        t_force_sim, force_sim_msgs = bag_data[force_sim_topic]
        xyz_force_sim = _xyz_from_wrench(force_sim_msgs)

    role = "master" if is_master else "slave"
    prefix = os.path.join(out_dir, arm)

    ep_idx_pos_real = episode_index_for(t_pos_real, episodes) if episodes else None
    ep_idx_pos_sim = episode_index_for(t_pos_sim, episodes) if episodes and len(t_pos_sim) else None

    _plot_3d(f"{prefix}_position_3d.png", xyz_pos_real, xyz_pos_sim,
              "displacement", f"{role} ({arm}) displacement from session start, 3D, real vs visual",
              ep_idx_pos_real, ep_idx_pos_sim)
    _plot_xyz(f"{prefix}_position_xyz.png", t_pos_real, xyz_pos_real,
              t_pos_sim, xyz_pos_sim, "displacement (mm)",
              f"{role} ({arm}) displacement from session start vs time, real vs visual",
              episodes=episodes)

    if len(xyz_force_real):
        ep_idx_force_real = episode_index_for(t_force_real, episodes) if episodes else None
        ep_idx_force_sim = episode_index_for(t_force_sim, episodes) if episodes and len(t_force_sim) else None

        _plot_3d(f"{prefix}_force_3d.png", xyz_force_real, xyz_force_sim,
                  "force", f"{role} ({arm}) force, 3D, real vs visual",
                  ep_idx_force_real, ep_idx_force_sim, eps=DEGENERATE_FORCE_EPS_N)
        _plot_xyz(f"{prefix}_force_xyz.png", t_force_real, xyz_force_real,
                  t_force_sim, xyz_force_sim, "force (N)",
                  f"{role} ({arm}) force vs time, real vs visual",
                  episodes=episodes, eps=DEGENERATE_FORCE_EPS_N)
        if episodes:
            _plot_force_by_episode(f"{prefix}_force_by_episode.png",
                                    t_force_real, xyz_force_real,
                                    t_force_sim, xyz_force_sim, episodes,
                                    f"{role} ({arm}) peak force per case, real vs visual")
    else:
        print(f"[{arm}] no {force_topic} in bag -- skipping force plots")

    if is_master and xyz_pos_sim is not None and xyz_force_sim is not None and len(xyz_force_real):
        pos_sim_on_real = _interp_to(t_pos_real, t_pos_sim, xyz_pos_sim)
        pos_noise_mm = xyz_pos_real - pos_sim_on_real

        force_sim_on_real = _interp_to(t_force_real, t_force_sim, xyz_force_sim)
        force_noise_n = xyz_force_real - force_sim_on_real

        _plot_noise(f"{prefix}_noise_xyz.png", t_pos_real, pos_noise_mm,
                    t_force_real, force_noise_n,
                    f"{role} ({arm}) noise: real - visual, position & force")
    elif is_master:
        print(f"[{arm}] missing sim pose/force -- skipping the master noise panel")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--out-dir", default=None,
                     help="default: <bag>/plots or <bag_parent>/<bag_name>_plots")
    ap.add_argument("--arms", default="left,right",
                     help="comma-separated arm names; 'left' is treated as master")
    ap.add_argument("--ann-topic", default="/contact_annotation")
    ap.add_argument("--merge-gap-s", type=float, default=0.3,
                     help="merge contact episodes separated by less than this "
                          "many seconds (catches ENTER key-repeat bursts)")
    args = ap.parse_args()

    arms = [a.strip() for a in args.arms.split(",") if a.strip()]
    out_dir = args.out_dir or (
        args.bag.rstrip("/") + "_plots"
    )
    os.makedirs(out_dir, exist_ok=True)

    topics = [args.ann_topic]
    for arm in arms:
        topics += [f"/touch_{arm}/pose", f"/touch_{arm}_sim/pose",
                   f"/touch_{arm}/applied_force", f"/touch_{arm}_sim/applied_force"]
    bag_data = read_bag(args.bag, topics)

    episodes: list[tuple[float, float]] = []
    if args.ann_topic in bag_data:
        ann_t, ann_msgs = bag_data[args.ann_topic]
        ann_v = np.array([bool(m.data) for m in ann_msgs])
        episodes = build_episodes(ann_t, ann_v, merge_gap_s=args.merge_gap_s)
        print(f"{len(episodes)} contact episode(s) found in {args.ann_topic}")
        for k, (t0, t1) in enumerate(episodes):
            print(f"  press {k + 1}: {t0:.2f}s -> {t1:.2f}s ({t1 - t0:.2f}s)")
    else:
        print(f"NOTE: no {args.ann_topic} in bag -- plots will not be split into cases")

    for arm in arms:
        plot_arm(bag_data, arm, out_dir, is_master=(arm == "left"), episodes=episodes)

    print(f"plots written to {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
