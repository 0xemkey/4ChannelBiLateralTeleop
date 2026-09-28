#!/usr/bin/env python3
"""Optional display-data publisher; never publishes control messages."""
import json
import time

from telemetry_core import config, synthetic, validate


def main():
    cfg = config()
    validate('synthetic', cfg, ros=True)
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init(args=[])
    node = Node('synthetic_display_data', namespace='/teleop_4ch_sim/visualization', enable_rosout=False, start_parameter_services=False)
    publisher = node.create_publisher(String, cfg['demo_topic'], 10)
    start = time.monotonic()
    def tick():
        for name, namespace in node.get_node_names_and_namespaces():
            if namespace.rstrip('/') + '/' + name in ('/teleop_master_left', '/teleop_slave_right'):
                raise RuntimeError('Forbidden live node name in isolated domain')
        publisher.publish(String(data=json.dumps(synthetic((time.monotonic()-start) % 11))))
    node.create_timer(1/cfg['expected_hz'], tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
