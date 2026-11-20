#!/usr/bin/env python3
"""Exercise the fusion ROS node with timestamped depth and masks."""

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from semantic_mapping.fusion import Fusion


def main():
    rclpy.init()
    fusion = Fusion()
    node = Node('synthetic_fusion_input')
    executor = SingleThreadedExecutor()
    executor.add_node(fusion)
    executor.add_node(node)
    broadcaster = StaticTransformBroadcaster(node)
    bridge = CvBridge()
    revision = node.create_publisher(
        Header, 'map_revision', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    frames = []
    for parent, child, x in [('map', 'base_link', 0.0), ('map', 'rgb', 0.0),
                             ('rgb', 'left', 0.0), ('rgb', 'right', 0.05)]:
        frame = TransformStamped()
        frame.header.stamp = node.get_clock().now().to_msg()
        frame.header.frame_id = parent
        frame.child_frame_id = child
        frame.transform.translation.x = x
        frame.transform.rotation.w = 1.0
        frames.append(frame)
    broadcaster.sendTransform(frames)
    depth_pub = node.create_publisher(Image, 'aligned_depth', qos_profile_sensor_data)
    mask_pub = node.create_publisher(Image, 'movable_mask', qos_profile_sensor_data)
    camera_pubs = {
        name: node.create_publisher(CameraInfo, f'{name}_info', qos_profile_sensor_data)
        for name in ('depth', 'left', 'right')
    }
    observed = {}
    subscriptions = {
        'left_mask': Image, 'right_mask': Image, 'class_mask': Image,
        'semantic_grid': OccupancyGrid, 'semantic_voxels': PointCloud2,
    }
    for topic, message_type in subscriptions.items():
        node.create_subscription(message_type, topic,
                                 lambda message, name=topic: observed.__setitem__(name, message),
                                 qos_profile_sensor_data)
    width, height = 64, 48
    depth = np.full((height, width), 1000, dtype=np.uint16)
    moving = np.zeros((height, width), dtype=np.uint8)
    moving[6:36, 16:40] = 3
    rev = Header(frame_id='map')
    rev.stamp = node.get_clock().now().to_msg()
    revision.publish(rev)
    started = time.monotonic()
    try:
        while time.monotonic() - started < 12 and len(observed) != len(subscriptions):
            now = node.get_clock().now().to_msg()
            for name, publisher in camera_pubs.items():
                info = CameraInfo()
                info.header.stamp = now
                info.header.frame_id = 'rgb' if name == 'depth' else name
                info.width, info.height = width, height
                info.k = [50.0, 0.0, 32.0, 0.0, 50.0, 24.0, 0.0, 0.0, 1.0]
                publisher.publish(info)
            for value, encoding, publisher in ((depth, '16UC1', depth_pub),
                                                (moving, 'mono8', mask_pub)):
                image = bridge.cv2_to_imgmsg(value, encoding=encoding)
                image.header.stamp = now
                image.header.frame_id = 'rgb'
                publisher.publish(image)
            for _ in range(18):
                executor.spin_once(timeout_sec=0.006)
        if len(observed) != len(subscriptions):
            raise RuntimeError(f'Missing fusion outputs: {sorted(subscriptions.keys() - observed.keys())}')
        assert 99 in observed['semantic_grid'].data and 100 in observed['semantic_grid'].data
        assert set(np.unique(bridge.imgmsg_to_cv2(observed['class_mask']))) == {2, 3}
        assert np.any(bridge.imgmsg_to_cv2(observed['left_mask']))
        assert np.any(bridge.imgmsg_to_cv2(observed['right_mask']))
        assert observed['semantic_voxels'].width > 0
        print('Fusion produced stereo masks, static and movable classes, and a voxel cloud.')
    finally:
        executor.shutdown()
        fusion.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
