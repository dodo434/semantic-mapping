#!/usr/bin/env python3
"""Measure ROS receive rate and capture-to-receive age during a fixed run."""

import argparse
import json
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


TOPICS = {
    '/semantic/body_pose': PoseStamped,
    '/semantic/movable_mask': Image,
    '/semantic/semantic_grid': OccupancyGrid,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60)
    args = parser.parse_args(remove_ros_args()[1:])
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    rclpy.init()
    node = Node('semantic_benchmark')
    samples = {topic: [] for topic in TOPICS}

    def receive(message, topic):
        when = message.header.stamp
        age = node.get_clock().now().nanoseconds - (when.sec * 1_000_000_000 + when.nanosec)
        samples[topic].append(age / 1e6)

    for topic, message_type in TOPICS.items():
        node.create_subscription(message_type, topic,
                                 lambda message, name=topic: receive(message, name),
                                 qos_profile_sensor_data)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    elapsed = time.monotonic() - start
    output = {}
    for topic, ages in samples.items():
        output[topic] = {
            'samples': len(ages),
            'receive_hz': len(ages) / elapsed,
            'capture_to_receive_ms_p50': float(np.percentile(ages, 50)) if ages else None,
            'capture_to_receive_ms_p95': float(np.percentile(ages, 95)) if ages else None,
            'negative_ages': int(np.count_nonzero(np.array(ages) < 0)),
        }
    print(json.dumps(output, indent=2))
    return int(not all(samples.values()))


if __name__ == '__main__':
    raise SystemExit(main())
