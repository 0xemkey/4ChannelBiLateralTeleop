#!/usr/bin/env python3
"""Publish display-only RViz markers in the isolated simulation domain."""
import time

import numpy as np

from telemetry_core import COLORS, FORCES, SOURCES, config, relative, replay, signal, synthetic, validate
from teleop_4ch_dashboard import parser, value_text


def force_arrow(force, cfg):
    delta = np.asarray(force, dtype=float) * cfg['force_m_per_n']
    return delta * min(1, cfg['max_arrow_m'] / max(np.linalg.norm(delta), 1e-12))


def markers(samples, source, cfg, stamp):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
    from matplotlib.colors import to_rgba
    result = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    result.markers.append(clear)
    sample = samples[-1]
    now = sample.get('now', sample['t'])
    def add(kind, points, color, scale=.004, text=''):
        m = Marker()
        m.header.frame_id = cfg['frame']
        m.header.stamp = stamp
        m.ns = 'display_only'
        m.id = len(result.markers)
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.
        m.scale.x = m.scale.y = m.scale.z = float(scale)
        m.color.r, m.color.g, m.color.b, m.color.a = to_rgba(color)
        if kind in (Marker.SPHERE, Marker.TEXT_VIEW_FACING, Marker.CUBE):
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, points[0])
        else:
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
        m.text = text
        result.markers.append(m)
        return m
    master, slave, mapped, error = relative(sample, now)
    arm_points = {}
    for key, color in [('master_arm', 'cyan'), ('slave_arm', 'darkorange')]:
        arm, status = signal(sample, key, now)
        if status == 'OK':
            arm_points[key] = np.asarray(arm)
            add(Marker.LINE_STRIP, arm, color, .009)
            for joint in arm:
                add(Marker.SPHERE, [joint], color, .013)
    for index, pos, color in [(0, master, 'cyan'), (1, slave, 'darkorange')]:
        if pos is not None:
            add(Marker.SPHERE, [pos], color, .008)
        trail = [relative(s)[index] for s in samples[-300:]]
        trail = [p for p in trail if p is not None]
        if len(trail) > 1:
            add(Marker.LINE_STRIP, trail, color, .001)
    # Origins are anchor-relative; absolute anchors remain separately labeled.
    add(Marker.SPHERE, [[0, 0, 0]], 'gray', .005)
    slave_world = arm_points.get('slave_arm')
    slave_world = slave_world[-1] if slave_world is not None else slave
    target_world = slave_world + (mapped-slave) if slave_world is not None and slave is not None and mapped is not None else mapped
    if target_world is not None:
        add(Marker.SPHERE, [target_world], (1, 1, 1, .3), .012)
    if error is not None and slave_world is not None and target_world is not None:
        add(Marker.LINE_STRIP, [slave_world, target_world], 'red' if np.linalg.norm(error) > .01 else 'green', .002)
    for axis, color in enumerate(('red', 'green', 'blue')):
        add(Marker.ARROW, [[0, 0, 0], np.eye(3)[axis]*.03], color, .001)
    if source == 'synthetic':
        plane = add(Marker.CUBE, [[0, .06, 0]], (.5, .5, .5, .25))
        plane.scale.x, plane.scale.y, plane.scale.z = .1, .001, .1
    if slave_world is not None:
        for key in ('applied', 'dob_cartesian'):
            force, status = signal(sample, key, now)
            if status == 'OK':
                offset = np.array([-.004 if key == 'applied' else .004, 0, 0])
                add(Marker.ARROW, [slave_world + offset, slave_world + offset + force_arrow(force, cfg)], COLORS[key], .004)
    text = [SOURCES[source], 'Anchor-relative frame', f"{cfg['force_m_per_n']} m/N; display cap {cfg['max_arrow_m']} m", 'config ' + cfg['fingerprint']]
    for key in ('bridge_state', 'master_anchor', 'slave_anchor', 'C1_state', 'C2_state', 'C3_state', 'C4_state', *FORCES, 'dob_joint', 'deadman'):
        text.append(key + ': ' + value_text(sample, key, now))
    add(Marker.TEXT_VIEW_FACING, [[.13, .04, .08]], 'white', .004, '\n'.join(text))
    return result


def main():
    args = parser().parse_args()
    cfg = config(args.config)
    validate(args.source, cfg, ros=True)
    from ros_adapter import Observer
    from visualization_msgs.msg import MarkerArray
    # Replay and synthetic need no telemetry subscriptions.
    if args.source == 'simulation':
        observer = Observer(cfg, args.source)
        node, rclpy = observer.node, observer.rclpy
    else:
        import rclpy
        from rclpy.node import Node
        rclpy.init(args=[])
        node = Node('teleop_marker_display', namespace='/teleop_4ch_sim/visualization', enable_rosout=False, start_parameter_services=False)
        observer = None
    publisher = node.create_publisher(MarkerArray, cfg['markers'], 10)
    rows = replay(args.file) if args.source == 'replay' and args.file else None
    if args.source == 'replay' and rows is None:
        raise SystemExit('--file required')
    start = time.monotonic()
    history = []
    def tick():
        for name, namespace in node.get_node_names_and_namespaces():
            if namespace.rstrip('/') + '/' + name in ('/teleop_master_left', '/teleop_slave_right'):
                raise RuntimeError('Forbidden live node name in isolated domain')
        elapsed = time.monotonic() - start
        if observer:
            # Outer spin dispatches the telemetry callbacks.
            import copy
            sample = copy.deepcopy(observer.sample)
            sample['now'] = sample['t'] + (time.monotonic()-observer.received if observer.received else 0)
            from telemetry_core import put
            for key in ('C1', 'C4', 'C1_state', 'C4_state'):
                if key not in sample['signals']:
                    put(sample, key, None, sample['t'], 'BLOCKED')
        elif rows is not None:
            sample = rows[min(np.searchsorted([r['t'] for r in rows], rows[0]['t']+elapsed, side='right')-1, len(rows)-1)]
        else:
            sample = synthetic(elapsed % args.duration)
        if history and sample['t'] < history[-1]['t']:
            history.clear()
        history.append(sample)
        del history[:-300]
        publisher.publish(markers(history, args.source, cfg, node.get_clock().now().to_msg()))
    node.create_timer(.05, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
