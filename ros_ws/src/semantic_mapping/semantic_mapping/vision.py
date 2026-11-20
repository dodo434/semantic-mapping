"""Latest-frame instance segmentation with original acquisition timestamps."""

import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .geometry import MOVABLE

# COCO detections that should be treated as potentially movable.
DEFAULT_MOVABLE = [
    'person', 'chair', 'bottle', 'backpack', 'suitcase', 'dog', 'cat',
    'handbag', 'umbrella', 'bicycle', 'motorcycle', 'stroller', 'box', 'robot',
    'bench', 'sports ball', 'cup', 'dining table', 'potted plant',
]


class Segmentation(Node):
    def __init__(self):
        super().__init__('segmentation')
        from ultralytics import YOLO

        model = self.declare_parameter('model', '').value
        if not model:
            raise ValueError('Set model to the local YOLOv8 segmentation weights or engine')
        self.model = YOLO(model, task='segment')
        self.device = self.declare_parameter('device', '0').value
        self.size = self.declare_parameter('image_size', 416).value
        self.rate = self.declare_parameter('max_hz', 15.0).value
        if self.size <= 0 or self.rate <= 0:
            raise ValueError('image_size and max_hz must be positive')
        self.confidence = self.declare_parameter('confidence', 0.35).value
        self.movable = set(self.declare_parameter('movable_classes', DEFAULT_MOVABLE).value)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'movable_mask', qos_profile_sensor_data)
        self.sub = self.create_subscription(Image, 'color', self._receive, qos_profile_sensor_data)
        self.pending = None
        self.condition = threading.Condition()
        self.stop = False
        self.worker = threading.Thread(target=self._work, name='segmentation-inference')
        self.worker.start()

    def _receive(self, message):
        with self.condition:
            self.pending = message
            self.condition.notify()

    def _work(self):
        next_start = 0.0
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.stop or self.pending is not None)
                if self.stop:
                    return
                wait = next_start - time.monotonic()
                if wait > 0.0:
                    self.condition.wait(timeout=wait)
                    continue
                image = self.pending
                self.pending = None
            started = time.monotonic()
            next_start = started + 1.0 / self.rate
            try:
                frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
                result = self.model.predict(
                    frame, imgsz=self.size, device=self.device, conf=self.confidence,
                    retina_masks=True, verbose=False,
                )[0]
                mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                if result.masks is not None and result.boxes is not None:
                    classes = result.boxes.cls.int().cpu().tolist()
                    selected = [index for index, cls in enumerate(classes)
                                if result.names[cls] in self.movable]
                    if selected:
                        combined = result.masks.data[selected].any(dim=0).byte().cpu().numpy()
                        if combined.shape != mask.shape:
                            combined = cv2.resize(
                                combined, (mask.shape[1], mask.shape[0]),
                                interpolation=cv2.INTER_NEAREST,
                            )
                        mask[combined != 0] = MOVABLE
                result_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
                result_msg.header = image.header
                if self.context.ok():
                    self.publisher.publish(result_msg)
                elapsed = (time.monotonic() - started) * 1000
                self.get_logger().info(f'Inference and mask: {elapsed:.1f} ms', throttle_duration_sec=5.0)
            except Exception as error:
                self.get_logger().error(str(error), throttle_duration_sec=2.0)

    def destroy_node(self):
        with self.condition:
            self.stop = True
            self.condition.notify_all()
        self.worker.join()
        return super().destroy_node()
