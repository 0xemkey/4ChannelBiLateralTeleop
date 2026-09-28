import copy
import json
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from telemetry_core import config, metrics, put, relative, replay, signal, synthetic, validate
from teleop_4ch_visualizer import force_arrow


def test_offsets_scale_and_sign():
    s = synthetic(4)
    s['mapping'] = [[2, 0, 0], [0, -3, 0], [0, 0, .5]]
    for key in ('master_delta', 'slave_delta', 'mapped_master_delta'):
        del s['signals'][key]
    ma, sa = np.array([10, 20, 30]), np.array([-20, 5, -10])
    delta = np.array([.1, .2, .3])
    for key, value in [('master_anchor', ma), ('slave_anchor', sa), ('master_position', ma+delta), ('slave_position', sa+np.asarray(s['mapping'])@delta)]:
        put(s, key, value.tolist(), 4)
    np.testing.assert_allclose(relative(s)[3], 0, atol=1e-14)
    np.testing.assert_allclose(metrics([s])['rmse_m'], 0, atol=1e-14)


def test_lag_unequal_rates_and_drops():
    samples = []
    for i in range(300):
        if i % 13 == 0:
            continue
        t = i*.02
        s = synthetic(t)
        # Slave observations at 25 Hz held between arrivals; master at 50 Hz.
        ts = (i//2)*.04
        put(s, 'mapped_master_delta', [0, .04*np.sin(2*t), 0], t)
        put(s, 'slave_delta', [0, .04*np.sin(2*(ts-.12)), 0], ts)
        samples.append(s)
    result = metrics(samples)
    assert abs(result['lag_s'][1]-.12) <= .021
    assert result['sample_loss'] > 20
    assert result['jitter_s'] > 0
    assert result['positive_y_rmse_m'] > 0
    assert result['negative_y_rmse_m'] > 0


def test_missing_stale_blocked_and_nonfinite():
    s = synthetic(5)
    for key in ('C1', 'C4', 'dob_joint'):
        del s['signals'][key]
        assert signal(s, key, 5) == (None, 'N/A')
    assert signal(s, 'applied', 6) == (None, 'STALE')
    assert relative(s, 6)[3] is None
    put(s, 'C1', None, 5, 'BLOCKED')
    assert signal(s, 'C1', 5) == (None, 'BLOCKED')
    put(s, 'C4', [float('nan'), 0, 0], 5)
    assert signal(s, 'C4', 5) == (None, 'INVALID')


def test_clamp_force_direction_and_display_cap():
    samples = [synthetic(t) for t in (4., 4.1, 4.2, 4.3)]
    for i, s in enumerate(samples):
        put(s, 'clamped', i < 2, s['t'])
        put(s, 'applied', [0, -2, 0], s['t'])
    result = metrics(samples)
    assert result['clamp_duty'] == .5
    assert result['applied_peak_n'] == 2
    cfg = config()
    np.testing.assert_allclose(force_arrow([0, -100, 0], cfg), [0, -.12, 0])
    assert signal(samples[0], 'applied', 4)[0] == [0, -2, 0]


@pytest.mark.parametrize('source', ['hardware', 'live-sim', 'unknown'])
def test_reject_sources(source):
    with pytest.raises(ValueError):
        validate(source, config())


def test_domain_and_topic_isolation(monkeypatch):
    cfg = config()
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    with pytest.raises(ValueError):
        validate('simulation', cfg, ros=True)
    monkeypatch.setenv('ROS_DOMAIN_ID', '42')
    validate('simulation', cfg, ros=True)
    for topic in ('/teleop_master_left/telemetry', '/teleop_4ch_sim/teleop_slave_right/telemetry'):
        altered = copy.deepcopy(cfg)
        altered['subscriptions']['master']['topic'] = topic
        with pytest.raises(ValueError):
            validate('simulation', altered)
    cfg['markers'] = '/teleop_4ch_sim/enable'
    with pytest.raises(ValueError):
        validate('simulation', cfg)


def test_replay_preserves_timestamps(tmp_path):
    rows = [synthetic(100.1), synthetic(100.7)]
    path = tmp_path / 'existing.json'
    path.write_text(json.dumps(rows))
    assert [r['t'] for r in replay(path)] == [100.1, 100.7]
    path.write_text(json.dumps(rows[::-1]))
    with pytest.raises(ValueError):
        replay(path)


def test_headless_ui(tmp_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    from teleop_4ch_dashboard import draw_dashboard, draw_scene, value_text
    rows = [synthetic(float(t)) for t in np.arange(0, 6, .1)]
    cfg = config()
    assert value_text(rows[-1], 'dob_cartesian', rows[-1]['t']) != 'N/A'
    missing_dob = copy.deepcopy(rows[-1])
    del missing_dob['signals']['dob_cartesian']
    assert value_text(missing_dob, 'dob_cartesian', missing_dob['t']) == 'N/A'
    fig = plt.figure(figsize=(17, 11))
    draw_dashboard(fig, rows, 'synthetic', cfg)
    fig.savefig(tmp_path/'dashboard.png')
    assert np.asarray(Image.open(tmp_path/'dashboard.png')).std() > 20
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    for text in fig.axes[0].texts:
        assert fig.bbox.contains(*text.get_window_extent(renderer).get_points()[0])
    plt.close(fig)
    fig = plt.figure(figsize=(8, 7))
    draw_scene(fig.add_subplot(projection='3d'), rows, 'synthetic', cfg)
    fig.savefig(tmp_path/'scene.png')
    assert np.asarray(Image.open(tmp_path/'scene.png')).std() > 10
    plt.close(fig)


def test_offline_ros_marker_serialization():
    from builtin_interfaces.msg import Time
    from rclpy.serialization import serialize_message
    from teleop_4ch_visualizer import markers
    message = markers([synthetic(5)], 'synthetic', config(), Time())
    assert serialize_message(message)
    assert all(m.header.frame_id == config()['frame'] for m in message.markers[1:])
    assert any('SYNTHETIC DEMO' in m.text for m in message.markers)
    assert sum(m.type == m.LINE_STRIP for m in message.markers) >= 2


def test_adapter_reads_existing_fields_and_blocks_unwritten_force():
    from types import SimpleNamespace
    from ros_adapter import Observer
    observer = Observer.__new__(Observer)
    observer.sample = {'t': 0., 'mapping': config()['mapping'], 'signals': {}}
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=5, nanosec=0)),
                          spring_force_n=[1., 2., 3.], applied_force_n=[8., 9., 10.],
                          cartesian_write_active=False)
    observer.adapt(msg, {'fields': {'spring': 'spring_force_n', 'applied': 'applied_force_n', 'C1': 'nonexistent'}})
    assert signal(observer.sample, 'spring', 5) == ([1., 2., 3.], 'OK')
    assert signal(observer.sample, 'applied', 5) == (None, 'BLOCKED')
    assert signal(observer.sample, 'C1', 5) == (None, 'N/A')
