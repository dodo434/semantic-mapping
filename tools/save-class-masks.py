#!/usr/bin/env python3
"""Save exact class-ID PNGs from live ROS or from timestamped bag replay."""

import argparse
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='new directory for predictions')
    parser.add_argument('--frames', type=int, default=100)
    parser.add_argument('--timeout', type=float, default=120)
    args = parser.parse_args(remove_ros_args()[1:])
    if args.frames < 1 or args.timeout <= 0:
        parser.error('positive frame and timeout limits are required')
    args.output.mkdir(mode=0o700)
    rclpy.init()
    node = rclpy.create_node('save_semantic_masks')
    bridge = CvBridge()
    saved = set()

    def save(image):
        stamp = image.header.stamp
        name = f'{stamp.sec}.{stamp.nanosec:09d}.png'
        if name not in saved:
            labels = bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
            if not cv2.imwrite(str(args.output / name), labels):
                raise IOError(f'could not save {name}')
            saved.add(name)

    node.create_subscription(Image, '/semantic/class_mask', save, qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and len(saved) < args.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'Saved {len(saved)} frames in {args.output}')
    return int(len(saved) < args.frames)


if __name__ == '__main__':
    raise SystemExit(main())
