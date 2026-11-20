"""Timestamp-matched depth/label fusion and sensor-frame tracking masks."""

from collections import deque

import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import FLOOR, MOVABLE, STATIC, backproject, fit_floor, floor_inliers, project_exclusions, transform_points
from .voxel_map import VoxelMap


def stamp(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def camera_model(info, rectified=False):
    if rectified and info.p[0] > 0 and info.p[5] > 0:
        return info.p[0], info.p[5], info.p[2], info.p[6]
    return info.k[0], info.k[4], info.k[2], info.k[5]


class Fusion(Node):
    def __init__(self):
        super().__init__('semantic_fusion')
        self.world = self.declare_parameter('world_frame', 'map').value
        self.base = self.declare_parameter('base_frame', 'base_link').value
        self.base_height = self.declare_parameter('base_height', 0.18).value
        self.max_obstacle_height = self.declare_parameter('max_obstacle_height', 1.8).value
        self.step = self.declare_parameter('pixel_stride', 8).value
        self.max_depth = self.declare_parameter('max_depth', 5.0).value
        rate = self.declare_parameter('map_hz', 8.0).value
        self.period = 1.0 / rate if rate > 0 else 0.0
        self.slop = int(self.declare_parameter('depth_slop_sec', 0.020).value * 1e9)
        self.publish_cloud = self.declare_parameter('publish_cloud', True).value
        self.publish_static_cloud = self.declare_parameter('publish_static_cloud', False).value
        self.publish_filtered_depth = self.declare_parameter('publish_filtered_depth', False).value
        if self.step < 1 or self.max_depth < 0.2 or self.period <= 0 or self.slop < 0:
            raise ValueError('Invalid fusion rate, stride, or depth limits')
        self.map = VoxelMap(
            self.declare_parameter('voxel_size', 0.05).value,
            self.declare_parameter('map_radius', 6.0).value,
            self.declare_parameter('max_voxels', 160000).value,
            self.declare_parameter('movable_decay_sec', 1.2).value,
        )
        self.bridge = CvBridge()
        self.rng = np.random.default_rng(42)
        self.tf = Buffer(cache_time=Duration(seconds=5.0))
        self.listener = TransformListener(self.tf, self)
        self.depths = deque(maxlen=24)
        self.masks = deque(maxlen=5)
        self.pending = deque(maxlen=6)
        self.info = {}
        self.camera_cache = None
        self.epoch = 0
        self.last_fused = -1
        self.last_enqueued = -1
        self.subscribe(Image, 'aligned_depth', self.depths.append)
        self.subscribe(Image, 'movable_mask', self.masks.append)
        for name in ('depth', 'left', 'right'):
            self.subscribe(CameraInfo, f'{name}_info',
                           lambda message, key=name: self.info.__setitem__(key, message))
        revision_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.revision = self.create_subscription(Header, 'map_revision', self.reset_map, revision_qos)
        self.grid = self.create_publisher(OccupancyGrid, 'semantic_grid', 1)
        self.cloud = self.create_publisher(PointCloud2, 'semantic_voxels', 1)
        self.static_cloud = self.create_publisher(PointCloud2, 'static_cloud', 1)
        self.eval_mask = self.create_publisher(Image, 'class_mask', qos_profile_sensor_data)
        self.projected_masks = {
            name: self.create_publisher(Image, f'{name}_mask', qos_profile_sensor_data)
            for name in ('left', 'right')
        }
        self.filtered = self.create_publisher(Image, 'static_depth', qos_profile_sensor_data)
        self.filtered_info = self.create_publisher(CameraInfo, 'static_depth_info', qos_profile_sensor_data)
        self.create_timer(0.015, self.update)

    def subscribe(self, typ, topic, callback):
        return self.create_subscription(typ, topic, callback, qos_profile_sensor_data)

    def reset_map(self, message):
        self.epoch = max(0, message.stamp.sec * 1_000_000_000 + message.stamp.nanosec)
        self.last_fused = -1
        self.map.voxels.clear()

    def update(self):
        try:
            self.match_frames()
            self.fuse_ready()
        except (ValueError, TypeError, cv2.error) as error:
            self.get_logger().error(f'Semantic fusion: {error}', throttle_duration_sec=2.0)

    def match_frames(self):
        if not self.depths or not self.masks or 'depth' not in self.info:
            return
        mask_msg = self.masks[0]
        t = stamp(mask_msg)
        if stamp(self.depths[-1]) < t:
            return
        self.masks.popleft()
        depth_msg = min(self.depths, key=lambda message: abs(stamp(message)-t))
        depth_t = stamp(depth_msg)
        if abs(depth_t-t) > self.slop or depth_t <= self.last_enqueued:
            return
        info = self.info['depth']
        if (info.header.frame_id != depth_msg.header.frame_id
                or mask_msg.header.frame_id != depth_msg.header.frame_id
                or (info.width, info.height) != (depth_msg.width, depth_msg.height)):
            return
        native = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if depth_msg.encoding in ('16UC1', 'mono16'):
            depth = native.astype(np.float32) * 0.001
        elif depth_msg.encoding == '32FC1':
            depth = native
        else:
            raise ValueError(f'Unsupported depth encoding: {depth_msg.encoding}')
        mask = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
        if depth.shape != mask.shape:
            raise ValueError('RGB and aligned depth resolutions differ')
        k = camera_model(info)
        if min(k[:2]) <= 0:
            raise ValueError('Invalid aligned-depth camera intrinsics')
        rays = self.normalized_rays(info)
        moving = mask == MOVABLE
        self.publish_sensor_masks(moving, depth, depth_msg, k, rays)
        if self.publish_filtered_depth:
            self.publish_static_depth(native, moving, depth_msg, info)
        self.pending.append((depth_t, depth_msg.header, depth, moving, k, rays))
        self.last_enqueued = depth_t

    def normalized_rays(self, info):
        if not any(info.d):
            return None
        key = (info.width, info.height, tuple(info.k), tuple(info.d), info.distortion_model)
        if self.camera_cache is not None and self.camera_cache[0] == key:
            return self.camera_cache[1]
        y, x = np.mgrid[:info.height, :info.width]
        pixels = np.stack([x, y], axis=-1).reshape((-1, 1, 2)).astype(np.float64)
        matrix = np.array(info.k).reshape(3, 3)
        distortion = np.array(info.d)
        if info.distortion_model == 'equidistant':
            points = cv2.fisheye.undistortPoints(pixels, matrix, distortion)
        elif info.distortion_model in ('plumb_bob', 'rational_polynomial'):
            points = cv2.undistortPoints(pixels, matrix, distortion)
        else:
            raise ValueError(f'Unsupported RGB distortion: {info.distortion_model}')
        rays = points.reshape((info.height, info.width, 2))
        self.camera_cache = (key, rays)
        return rays

    def publish_sensor_masks(self, moving, depth, depth_msg, k, rays):
        sparse_depth = np.where(moving, depth, 0.0)
        points, _, _ = backproject(sparse_depth, k, 2, 0.20, self.max_depth, rays)
        for name, publisher in self.projected_masks.items():
            if name not in self.info:
                continue
            info = self.info[name]
            ir_k = camera_model(info, rectified=True)
            if min(ir_k[:2]) <= 0:
                continue
            try:
                tf = self.tf.lookup_transform(
                    info.header.frame_id, depth_msg.header.frame_id, Time.from_msg(depth_msg.header.stamp))
            except TransformException:
                continue
            projected = project_exclusions(transform_points(points, tf.transform), ir_k,
                                           (info.width, info.height))
            projected = cv2.dilate(projected, np.ones((9, 9), dtype=np.uint8))
            message = self.bridge.cv2_to_imgmsg(projected, encoding='mono8')
            message.header.stamp = depth_msg.header.stamp
            message.header.frame_id = info.header.frame_id
            publisher.publish(message)

    def publish_static_depth(self, native, moving, message, info):
        excluded = cv2.dilate(moving.astype(np.uint8), np.ones((7, 7), dtype=np.uint8))
        filtered = np.where(excluded, 0, native)
        output = self.bridge.cv2_to_imgmsg(filtered, encoding=message.encoding)
        output.header = message.header
        # CameraInfo is stable for the configured mode. Stamp it with this image.
        info.header.stamp = message.header.stamp
        self.filtered_info.publish(info)
        self.filtered.publish(output)

    def fuse_ready(self):
        if not self.pending or not self.epoch:
            return
        for index in range(len(self.pending)-1, -1, -1):
            t, header, depth, moving, k, rays = self.pending[index]
            if t < self.epoch or (self.last_fused >= 0 and t-self.last_fused < self.period*1e9):
                continue
            when = Time.from_msg(header.stamp)
            try:
                world_depth = self.tf.lookup_transform(self.world, header.frame_id, when)
                world_base = self.tf.lookup_transform(self.world, self.base, when)
            except TransformException:
                continue
            if not self.fuse(t, header, depth, moving, k, rays, world_depth.transform, world_base.transform):
                continue
            self.last_fused = t
            for _ in range(index+1):
                self.pending.popleft()
            return

    def fuse(self, t, header, depth, moving, k, rays, world_depth, world_base):
        sensor_points, y, x = backproject(depth, k, self.step, 0.20, self.max_depth, rays)
        if not len(sensor_points):
            return False
        points = transform_points(sensor_points, world_depth)
        sensor = world_depth.translation
        origin = np.array([sensor.x, sensor.y, sensor.z])
        base = world_base.translation
        center = np.array([base.x, base.y, base.z])
        floor_z = base.z - self.base_height
        movable = moving[y, x]
        labels = np.full(len(points), STATIC, dtype=np.uint8)
        plane = fit_floor(points[~movable], floor_z, self.rng)
        labels[~movable] = np.where(
            floor_inliers(points[~movable], floor_z, plane), FLOOR, STATIC)
        labels[movable] = MOVABLE
        seconds = t * 1e-9
        self.map.integrate(points, labels, origin, center, seconds)
        cells, (ox, oy) = self.map.project(center, floor_z, seconds, self.max_obstacle_height)
        grid = OccupancyGrid()
        grid.header.stamp = header.stamp
        grid.header.frame_id = self.world
        grid.info.resolution = self.map.resolution
        grid.info.width, grid.info.height = cells.shape[1], cells.shape[0]
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.orientation.w = 1.0
        grid.data = cells.ravel().tolist()
        self.grid.publish(grid)
        if self.publish_cloud and self.cloud.get_subscription_count():
            self.publish_voxels(grid.header, seconds)
        if self.publish_static_cloud and self.static_cloud.get_subscription_count():
            self.static_cloud.publish(point_cloud2.create_cloud_xyz32(header, sensor_points[~movable]))
        if self.eval_mask.get_subscription_count():
            xyz, y, x = backproject(depth, k, 1, 0.20, self.max_depth, rays)
            xyz = transform_points(xyz, world_depth)
            view = np.zeros(depth.shape, dtype=np.uint8)
            view[y, x] = np.where(floor_inliers(xyz, floor_z, plane), FLOOR, STATIC)
            view[moving] = MOVABLE
            mask = self.bridge.cv2_to_imgmsg(view, encoding='mono8')
            mask.header = header
            self.eval_mask.publish(mask)
        return True

    def publish_voxels(self, header, now):
        fields = [
            PointField(name=coordinate, offset=4*index, datatype=PointField.FLOAT32, count=1)
            for index, coordinate in enumerate(('x', 'y', 'z'))
        ] + [
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
            PointField(name='class_id', offset=16, datatype=PointField.UINT32, count=1),
        ]
        colors = {FLOOR: 0xFF31BB55, STATIC: 0xFF6FA6DA, MOVABLE: 0xFFFF8425}
        scale = self.map.resolution
        rows = [(*((coordinate+0.5)*scale for coordinate in key), colors[label], label)
                for key, label in self.map.classified(now)]
        self.cloud.publish(point_cloud2.create_cloud(header, fields, rows))
