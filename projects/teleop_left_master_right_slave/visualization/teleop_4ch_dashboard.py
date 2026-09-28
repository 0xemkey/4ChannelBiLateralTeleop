#!/usr/bin/env python3
"""Matplotlib dashboard and deterministic headless synthetic artifacts."""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from telemetry_core import COLORS, FORCES, SOURCES, config, metrics, relative, replay, signal, synthetic, validate


def parser():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument('--source', choices=SOURCES, default='synthetic')
    p.add_argument('--config')
    p.add_argument('--file', type=Path)
    p.add_argument('--artifacts', type=Path)
    p.add_argument('--duration', type=float, default=11)
    return p


def value_text(sample, key, now):
    value, status = signal(sample, key, now)
    if status != 'OK':
        return status
    if isinstance(value, list):
        return ' '.join(f'{v:+.3f}' for v in value)
    return f'{value:.3f}' if isinstance(value, float) else str(value)


def draw_scene(ax, samples, source, cfg):
    ax.clear()
    sample = samples[-1]
    now = sample.get('now', sample['t'])
    ax.figure.set_facecolor('#101419')
    ax.set_facecolor('#101419')
    ax.set_title(SOURCES[source] + ' | dual-arm force view', fontsize=14, color='white', pad=18)
    master, slave, mapped, error = relative(sample, now)
    arm_points = {}
    for key, color, label in [('master_arm', '#18dce5', 'Master arm'), ('slave_arm', '#ff922b', 'Slave arm')]:
        arm, status = signal(sample, key, now)
        if status != 'OK':
            continue
        arm = np.asarray(arm)
        arm_points[key] = arm
        ax.plot(*arm.T, color=color, linewidth=7, solid_capstyle='round', label=label)
        ax.plot(*arm.T, color='#d8e0e8', linewidth=2, alpha=.55)
        ax.scatter(*arm[:-1].T, color=color, edgecolor='white', linewidth=.8, s=[85, 70, 62, 52])
        ax.scatter(*arm[-1], color=color, edgecolor='white', linewidth=1.5, s=110)
        base = arm[0]
        ax.bar3d(base[0]-.025, base[1]-.025, -.008, .05, .05, .012, color=color, alpha=.45)
    for key, color in [('master_arm', '#18dce5'), ('slave_arm', '#ff922b')]:
        trail = []
        for row in samples[-300:]:
            arm, status = signal(row, key, row['t'])
            if status == 'OK':
                trail.append(arm[-1])
        if len(trail) > 1:
            ax.plot(*np.asarray(trail).T, color=color, linewidth=1.4, alpha=.5)
    xx, zz = np.meshgrid([.08, .24], [0, .15])
    if source == 'synthetic':
        ax.plot_surface(xx, np.full_like(xx, .047), zz, color='#7e8791', alpha=.26)
    endpoint = arm_points.get('slave_arm', np.asarray([[.16, .045, .065]]))[-1]
    force_labels = []
    for key, color, label, offset in [('applied', '#ff334f', 'Applied force', -.004),
                                      ('dob_cartesian', 'white', 'DOB predicted', .004)]:
        force, status = signal(sample, key, now)
        if status == 'OK':
            arrow = np.array(force) * cfg['force_m_per_n']
            arrow *= min(1, cfg['max_arrow_m']/max(np.linalg.norm(arrow), 1e-12))
            origin = endpoint + [offset, 0, 0]
            ax.quiver(*origin, *arrow, color=color, linewidth=3, arrow_length_ratio=.22)
            force_labels.append(f'{label}: {np.linalg.norm(force):.3f} N')
        else:
            force_labels.append(f'{label}: {status}')
    ax.text2D(.03, .9, '\n'.join(force_labels), transform=ax.transAxes, color='white', fontsize=10,
              bbox={'facecolor': '#1c232b', 'edgecolor': '#59636e', 'alpha': .9, 'pad': 7})
    ax.set(xlim=(-.29, .29), ylim=(-.12, .14), zlim=(-.01, .22), xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)')
    ax.view_init(elev=25, azim=-64)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.label.set_color('#d8e0e8')
        axis.set_tick_params(colors='#aeb8c2')
        axis.pane.set_facecolor((.08, .1, .12, 1))
        axis.pane.set_edgecolor((.3, .34, .38, 1))
    ax.text2D(.03, .03, f"Arrow scale {cfg['force_m_per_n']} m/N | cap {cfg['max_arrow_m']} m", transform=ax.transAxes, fontsize=9, color='#aeb8c2')
    legend = ax.legend(loc='upper right', fontsize=8, facecolor='#1c232b', edgecolor='#59636e')
    for text in legend.get_texts():
        text.set_color('white')


def draw_dashboard(fig, samples, source, cfg):
    fig.clear()
    sample = samples[-1]
    now = sample.get('now', sample['t'])
    fig.suptitle(f"{SOURCES[source]} | Four-channel telemetry | config {cfg['fingerprint']}", fontsize=15)
    grid = fig.add_gridspec(4, 3, top=.92, bottom=.06, hspace=.65, wspace=.34)
    status = fig.add_subplot(grid[:, 0])
    status.axis('off')
    lines = []
    for key in ('bridge_state', 'C1_state', 'C2_state', 'C3_state', 'C4_state', 'gravity_alpha', 'coupling_alpha', 'deadman', 'master_enabled', 'slave_enabled', 'clamped', 'target'):
        lines.append(f'{key}: {value_text(sample, key, now)}')
    lines.append('')
    for key in (*FORCES, 'dob_joint'):
        lines.append(f'{key}: {value_text(sample, key, now)}')
    ages = [now - v['t'] for v in sample['signals'].values() if v.get('value') is not None]
    lines.append(f"Max signal age: {max(ages):.3f} s" if ages else 'Age: N/A')
    report = metrics(samples, cfg['expected_hz'])
    lines.append('Configured mapping: ' + str(cfg['mapping']))
    for key, value in report.items():
        if isinstance(value, list):
            value = ' '.join('N/A' if v is None else f'{v:.4f}' for v in value)
        elif isinstance(value, float):
            value = f'{value:.4f}'
        lines.append(f'{key}: {value if value is not None else "N/A"}')
    status.text(0, 1, '\n'.join(lines), va='top', fontsize=8, family='monospace', linespacing=1.45)
    t = [s['t'] for s in samples]
    def series(key, component=None):
        values = [signal(s, key, s['t'])[0] for s in samples]
        return [np.nan if v is None else (v[component] if component is not None else np.linalg.norm(v) if isinstance(v, list) else v) for v in values]
    rel = [relative(s) for s in samples]
    for axis in range(3):
        ax = fig.add_subplot(grid[axis, 1])
        for index, label, color in [(0, 'master', 'teal'), (1, 'slave', 'darkorange'), (2, 'mapped', 'gray')]:
            ax.plot(t, [r[index][axis] if r[index] is not None else np.nan for r in rel], label=label, color=color)
        ax.set_title('XYZ'[axis] + ' anchor-relative (m)', fontsize=10)
        ax.grid(alpha=.2)
        if axis == 0:
            ax.legend(fontsize=6, ncol=3)
    ax = fig.add_subplot(grid[3, 1])
    for i in range(3):
        ax.plot(t, [r[3][i] if r[3] is not None else np.nan for r in rel], label='XYZ'[i])
    ax.set_title('Tracking error (m)', fontsize=10)
    ax.legend(fontsize=6, ncol=3)
    for row, keys, title in [(0, ('applied', 'dob_cartesian'), 'Applied vs DOB predicted | magnitude (N)'),
                             (1, ('force_compare',), 'Applied vs DOB predicted | XYZ (N)'),
                             (2, ('spring', 'damping', 'auxiliary', 'C1', 'C4'), 'Force decomposition (N)'),
                             (3, ('deadman', 'master_enabled', 'slave_enabled'), 'Deadman and enable state')]:
        ax = fig.add_subplot(grid[row, 2])
        for key in keys:
            if key == 'force_compare':
                for i, axis_name in enumerate('XYZ'):
                    ax.plot(t, series('applied', i), label=f'Applied {axis_name}', linewidth=2)
                    ax.plot(t, series('dob_cartesian', i), '--', label=f'DOB {axis_name}', linewidth=1.4)
            else:
                style = '--' if key == 'dob_cartesian' else '-'
                color = '#222222' if key == 'dob_cartesian' else COLORS.get(key)
                ax.plot(t, series(key), style, label=key, color=color, linewidth=2 if key in ('applied', 'dob_cartesian') else 1.2)
        ax.set_title(title, fontsize=10)
        ax.legend(fontsize=6, ncol=3)
        ax.grid(alpha=.2)


def artifacts(folder, cfg, duration):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    folder.mkdir(parents=True, exist_ok=True)
    samples = [synthetic(float(t)) for t in np.arange(0, duration, 1/cfg['expected_hz'])]
    (folder / 'synthetic.json').write_text(json.dumps(samples))
    fig = plt.figure(figsize=(17, 11))
    draw_dashboard(fig, samples, 'synthetic', cfg)
    fig.savefig(folder / 'dashboard_synthetic.png', dpi=120)
    plt.close(fig)
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(projection='3d')
    draw_scene(ax, samples[:276], 'synthetic', cfg)
    fig.savefig(folder / 'scene_synthetic.png', dpi=120)
    frames = []
    for i in range(1, len(samples), 10):
        draw_scene(ax, samples[:i], 'synthetic', cfg)
        ax.set_title(f"SYNTHETIC DEMO | {samples[i-1]['t']:.1f}s | " + value_text(samples[i-1], 'bridge_state', samples[i-1]['t']))
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba()).copy()).convert('RGB'))
    frames[0].save(folder / 'synthetic_sequence.gif', save_all=True, append_images=frames[1:], duration=200, loop=0)
    plt.close(fig)
    (folder / 'metrics.json').write_text(json.dumps(metrics(samples), indent=2))


def main():
    args = parser().parse_args()
    cfg = config(args.config)
    validate(args.source, cfg)
    if args.artifacts:
        if args.source != 'synthetic':
            raise SystemExit('Artifact generation requires --source synthetic')
        artifacts(args.artifacts, cfg, args.duration)
        return
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    observer = None
    rows = replay(args.file) if args.source == 'replay' and args.file else None
    if args.source == 'replay' and rows is None:
        raise SystemExit('--file is required for replay')
    if args.source == 'simulation':
        from ros_adapter import Observer
        observer = Observer(cfg, args.source)
    fig = plt.figure(figsize=(17, 11))
    scene = plt.figure(figsize=(8, 7))
    ax = scene.add_subplot(projection='3d')
    history = []
    started = time.monotonic()
    def update(_):
        elapsed = time.monotonic() - started
        if observer:
            sample = observer.poll()
        elif rows is not None:
            index = min(np.searchsorted([r['t'] for r in rows], rows[0]['t'] + elapsed, side='right')-1, len(rows)-1)
            sample = rows[index]
        else:
            sample = synthetic(elapsed % args.duration)
        if history and sample['t'] < history[-1]['t']:
            history.clear()
        if rows is not None:
            history[:] = rows[max(0, index-599):index+1]
        elif args.source == 'synthetic':
            previous = history[-1]['t'] if history else -1/cfg['expected_hz']
            history.extend(synthetic(float(t)) for t in np.arange(previous+1/cfg['expected_hz'], sample['t'], 1/cfg['expected_hz']))
            if not history:
                history.append(sample)
        elif not history or sample['t'] > history[-1]['t']:
            history.append(sample)
        else:
            history[-1] = sample
        del history[:-600]
        draw_dashboard(fig, history, args.source, cfg)
        draw_scene(ax, history, args.source, cfg)
        scene.canvas.draw_idle()
    animation = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    try:
        plt.show()
    finally:
        if observer:
            observer.close()


if __name__ == '__main__':
    main()
