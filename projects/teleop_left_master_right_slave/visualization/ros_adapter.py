"""ROS display adapter. No command publishers or controller launch capability."""
import copy
import json
import time

from telemetry_core import put, validate


class Observer:
    def __init__(self, cfg, source):
        validate(source, cfg, ros=True)
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message
        from std_msgs.msg import String
        rclpy.init(args=[])
        self.rclpy = rclpy
        self.node = Node('teleop_visualization_observer', namespace='/teleop_4ch_sim/visualization',
                         enable_rosout=False, start_parameter_services=False)
        self.sample = {'t': 0., 'mapping': cfg['mapping'], 'signals': {}}
        self.received = None
        self.subscriptions = []
        self.cfg = cfg
        if source == 'synthetic':
            self.subscriptions.append(self.node.create_subscription(String, cfg['demo_topic'], self.demo, qos_profile_sensor_data))
        else:
            for name, adapter in cfg['subscriptions'].items():
                self.subscriptions.append(self.node.create_subscription(
                    get_message(adapter['type']), adapter['topic'],
                    lambda msg, adapter=adapter: self.adapt(msg, adapter), qos_profile_sensor_data))
        self.check_graph()

    def check_graph(self):
        for name, namespace in self.node.get_node_names_and_namespaces():
            if namespace.rstrip('/') + '/' + name in ('/teleop_master_left', '/teleop_slave_right'):
                raise RuntimeError('Live node name detected in isolated domain; observer stopped')

    def demo(self, msg):
        self.sample = json.loads(msg.data)
        self.received = time.monotonic()

    def adapt(self, msg, adapter):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0:
            return
        for key, field in adapter['fields'].items():
            if key == 'dob_cartesian':
                continue
            value = getattr(msg, field, None)
            if value is not None and not isinstance(value, (str, bool, int, float)):
                value = list(value)
            status = 'OK' if value is not None else 'N/A'
            if key == 'target' and not (getattr(msg, 'target_valid', False) and getattr(msg, 'target_fresh', False)):
                status = 'INVALID'
            if key == 'applied' and not getattr(msg, 'cartesian_write_active', False):
                status = 'BLOCKED'
            put(self.sample, key, value, stamp, status)
        self.sample['t'] = max(self.sample['t'], stamp)
        self.received = time.monotonic()

    def poll(self):
        # Drain queued telemetry before painting; GUI refresh is slower than ROS.
        for _ in range(100):
            self.rclpy.spin_once(self.node, timeout_sec=0)
        self.check_graph()
        value = copy.deepcopy(self.sample)
        if self.received is not None:
            value['now'] = value['t'] + time.monotonic() - self.received
        for key in ('C1', 'C4', 'C1_state', 'C4_state'):
            if key not in value['signals']:
                put(value, key, None, value['t'], 'BLOCKED')
        return value

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()
