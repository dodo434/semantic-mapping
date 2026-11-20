#!/usr/bin/env python3
"""Build a YOLOv8 segmentation engine on the target or export portable ONNX."""

import argparse
from pathlib import Path

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('weights', type=Path, help='local trusted *-seg.pt file')
    parser.add_argument('--format', choices=('engine', 'onnx'), default='engine')
    parser.add_argument('--image-size', type=int, default=416)
    parser.add_argument('--int8-data', type=Path, help='representative segmentation dataset YAML')
    args = parser.parse_args()
    if not args.weights.is_file() or args.weights.suffix != '.pt':
        parser.error('provide a local .pt checkpoint')
    if args.image_size <= 0 or args.image_size % 32:
        parser.error('image size must be a positive multiple of 32')
    if args.int8_data and (args.format != 'engine' or not args.int8_data.is_file()):
        parser.error('INT8 requires TensorRT and an existing dataset YAML')
    model = YOLO(str(args.weights), task='segment')
    common = dict(imgsz=args.image_size, batch=1, nms=False)
    if args.format == 'onnx':
        output = model.export(format='onnx', device='cpu', opset=17, simplify=False, **common)
    else:
        precision = {'int8': True, 'data': str(args.int8_data)} if args.int8_data else {'half': True}
        output = model.export(format='engine', device=0, workspace=2, **common, **precision)
    print(output)


if __name__ == '__main__':
    main()
