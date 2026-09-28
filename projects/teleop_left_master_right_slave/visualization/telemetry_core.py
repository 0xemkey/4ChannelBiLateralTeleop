"""Read-only normalized telemetry, file replay, and anchor-relative metrics."""
import csv
import hashlib
import json
import os
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent
SOURCES = {'synthetic': 'SYNTHETIC DEMO', 'simulation': 'LIVE SIM', 'replay': 'REPLAY'}
FORCES = ('spring', 'damping', 'auxiliary', 'C1', 'C4', 'unclamped', 'primary_clamped', 'final_command', 'applied', 'dob_cartesian')
COLORS = dict(zip(FORCES, ('green', 'royalblue', 'gold', 'magenta', 'purple', 'gray', 'teal', 'pink', 'red', 'white')))


def config(path=None):
    with open(path or ROOT / 'config/visualizer_topics.yaml') as stream:
        value = yaml.safe_load(stream)
    value['fingerprint'] = hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()[:12]
    return value


def validate(source, cfg, ros=False):
    if source not in SOURCES:
        raise ValueError('Only synthetic, simulation and replay are supported')
    if cfg['domain'] != 42 or (ros and os.environ.get('ROS_DOMAIN_ID') != '42'):
        raise ValueError('ROS processes require ROS_DOMAIN_ID=42')
    for topic in [cfg['markers'], cfg['demo_topic'], *[v['topic'] for v in cfg['subscriptions'].values()]]:
        parts = topic.split('/')
        if not topic.startswith('/teleop_4ch_sim/') or any(p in parts for p in ('teleop_master_left', 'teleop_slave_right', '..')):
            raise ValueError('Topic outside isolated visualization namespace: ' + topic)
    if cfg['markers'] != '/teleop_4ch_sim/visualization/markers' or cfg['demo_topic'] != '/teleop_4ch_sim/visualization/demo':
        raise ValueError('Display publication topics are fixed')


def signal(sample, key, now, timeout=.5):
    item = sample.get('signals', {}).get(key)
    if item is None:
        return None, 'N/A'
    if item.get('status', 'OK') != 'OK':
        return None, item['status']
    if now - item['t'] > timeout:
        return None, 'STALE'
    value = item.get('value')
    if value is None:
        return None, 'N/A'
    if isinstance(value, (list, float, int)) and not np.all(np.isfinite(value)):
        return None, 'INVALID'
    return value, 'OK'


def put(sample, key, value, stamp, status='OK'):
    sample.setdefault('signals', {})[key] = {'value': value, 't': float(stamp), 'status': status}


def relative(sample, now=None):
    now = sample['t'] if now is None else now
    def get(key):
        value, status = signal(sample, key, now)
        return None if status != 'OK' else np.asarray(value, dtype=float)
    master, slave = get('master_delta'), get('slave_delta')
    if master is None:
        p, a = get('master_position'), get('master_anchor')
        master = p - a if p is not None and a is not None else None
    if slave is None:
        p, a = get('slave_position'), get('slave_anchor')
        slave = p - a if p is not None and a is not None else None
    mapped = get('mapped_master_delta')
    if mapped is None and master is not None:
        mapped = np.asarray(sample['mapping']) @ master
    error = slave - mapped if slave is not None and mapped is not None else None
    return master, slave, mapped, error


def metrics(samples, expected_hz=50):
    rows = [(s, relative(s)) for s in samples]
    rows = [(s, r) for s, r in rows if r[3] is not None]
    if not rows:
        return {'status': 'N/A'}
    t = np.array([s['t'] for s, _ in rows])
    error = np.array([r[3] for _, r in rows])
    mapped = np.array([r[2] for _, r in rows])
    slave = np.array([r[1] for _, r in rows])
    dt = np.diff(t)
    result = {'rmse_m': np.sqrt(np.mean(error**2, axis=0)).tolist(),
              'peak_error_m': float(np.max(np.linalg.norm(error, axis=1))),
              'effective_hz': float(1 / np.mean(dt)) if len(dt) and np.mean(dt) > 0 else None,
              'jitter_s': float(np.std(dt)) if len(dt) else None,
              'sample_loss': int(np.maximum(np.rint(dt * expected_hz) - 1, 0).sum()),
              'lag_s': [], 'gain': []}
    # Timestamp interpolation estimates delay without assuming equal source rates.
    for axis in range(3):
        x, y = mapped[:, axis], slave[:, axis]
        if len(t) < 10 or np.ptp(x) < 1e-6:
            result['lag_s'].append(None)
            result['gain'].append(None)
            continue
        result['gain'].append(float(np.polyfit(x, y, 1)[0]))
        delays = np.arange(-.5, .501, 1 / expected_hz)
        scores = []
        for delay in delays:
            mask = (t - delay >= t[0]) & (t - delay <= t[-1])
            a = np.interp(t[mask] - delay, t, x)
            b = y[mask]
            scores.append(np.mean((a - b)**2) if len(a) else np.inf)
        result['lag_s'].append(float(delays[np.argmin(scores)]))
    velocity = np.diff(mapped, axis=0)
    for name, mask in [('positive', velocity[:, 1] > 1e-9), ('negative', velocity[:, 1] < -1e-9)]:
        result[name + '_y_rmse_m'] = float(np.sqrt(np.mean(error[1:][mask, 1]**2))) if mask.any() else None
    for key in ('applied', 'clamped'):
        values = [signal(s, key, s['t'])[0] for s, _ in rows]
        values = [v for v in values if v is not None]
        result['applied_peak_n' if key == 'applied' else 'clamp_duty'] = (
            float(max(np.linalg.norm(v) for v in values)) if key == 'applied' else float(np.mean(values))) if values else None
    pairs = [(signal(s, 'applied', s['t'])[0], signal(s, 'dob_cartesian', s['t'])[0]) for s, _ in rows]
    pairs = [(a, d) for a, d in pairs if a is not None and d is not None]
    result['dob_force_rmse_n'] = float(np.sqrt(np.mean((np.asarray([a for a, _ in pairs]) -
                                                        np.asarray([d for _, d in pairs]))**2))) if pairs else None
    return result


def synthetic_arm(base, endpoint, handedness):
    """Deterministic display geometry for the synthetic three-link devices."""
    base = np.asarray(base, dtype=float)
    endpoint = np.asarray(endpoint, dtype=float)
    shoulder = base + [0, 0, .055]
    elbow = shoulder * .55 + endpoint * .45 + [handedness * .035, -.018, .055]
    wrist = elbow * .45 + endpoint * .55 + [handedness * .018, .012, .012]
    return np.asarray([base, shoulder, elbow, wrist, endpoint]).tolist()


def synthetic(t):
    def trajectory(x):
        u = np.clip((x - 3) / 2, 0, 1)
        v = np.clip((x - 6) / 2, 0, 1)
        return .08 * (u*u*(3-2*u) - v*v*(3-2*v))
    sample = {'t': t, 'mapping': np.eye(3).tolist(), 'signals': {}}
    ma, sa = np.array([.15, -.1, .05]), np.array([-.12, .12, .02])
    md, sd = np.array([0, trajectory(t), 0]), np.array([0, min(trajectory(t-.12), .06), 0])
    gravity = float(np.clip(t-1, 0, 1) * np.clip(10-t, 0, 1))
    coupling = float(np.clip(t-2, 0, 1) * np.clip(9-t, 0, 1))
    state = 'READY' if t < 1 else 'GRAVITY_RAMP' if t < 2 else 'GRAVITY_SETTLE' if t < 3 else 'ACTIVE' if t < 9 else 'DISABLED'
    spring = np.array([0, -40 * max(md[1]-.06, 0), 0])
    damping = np.array([0, -.1*(trajectory(t)-trajectory(t-.02))/.02, 0])
    aux = np.array([0, 0, .3 * gravity])
    total = spring + damping + aux
    applied = total * min(1, .65/max(np.linalg.norm(total), 1e-9)) if t < 9 else np.zeros(3)
    dob_cartesian = applied * .92 + np.array([0, .018*np.sin(t*3)*float(np.linalg.norm(applied) > .05), 0])
    master_ee = np.array([-.16, -.015 + md[1], .065])
    slave_ee = np.array([.16, -.015 + sd[1], .065])
    fields = dict(master_position=(ma+md).tolist(), slave_position=(sa+sd).tolist(),
                  master_anchor=ma.tolist(), slave_anchor=sa.tolist(), target=(sa+md).tolist(),
                  master_delta=md.tolist(), slave_delta=sd.tolist(), mapped_master_delta=md.tolist(),
                  spring=spring.tolist(), damping=damping.tolist(), auxiliary=aux.tolist(),
                  C1=(spring*.3).tolist(), C4=(-spring*.3).tolist(), unclamped=total.tolist(),
                  primary_clamped=applied.tolist(), final_command=applied.tolist(), applied=applied.tolist(),
                  dob_cartesian=dob_cartesian.tolist(), dob_joint=[0, float(spring[1]*.02), 0],
                  master_arm=synthetic_arm([-.21, -.09, 0], master_ee, -1),
                  slave_arm=synthetic_arm([.21, -.09, 0], slave_ee, 1),
                  gravity_alpha=gravity, coupling_alpha=coupling,
                  deadman=t < 9, master_enabled=1 <= t < 9, slave_enabled=1 <= t < 9,
                  clamped=bool(np.linalg.norm(total) > .65), bridge_state=state,
                  C1_state='SYNTHETIC', C2_state='SYNTHETIC', C3_state='SYNTHETIC', C4_state='SYNTHETIC')
    for key, value in fields.items():
        put(sample, key, value, t)
    return sample


def replay(path):
    path = Path(path)
    if path.suffix == '.csv':
        with path.open() as stream:
            rows = [{'t': float(r['t']), 'mapping': json.loads(r['mapping']), 'signals': json.loads(r['signals'])} for r in csv.DictReader(stream)]
    elif path.suffix == '.json':
        rows = json.loads(path.read_text())
    else:
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if not rows or any(b['t'] <= a['t'] for a, b in zip(rows, rows[1:])):
        raise ValueError('Replay timestamps must be strictly increasing and nonempty')
    for row in rows:
        relative(row)
    return rows
