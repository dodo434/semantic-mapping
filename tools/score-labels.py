#!/usr/bin/env python3
"""Score paired 8-bit class-ID PNGs for floor, static structure, and movable."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def confusion(truth, predictions):
    if truth.shape != predictions.shape or truth.ndim != 2:
        raise ValueError('label images must have identical two-dimensional shapes')
    if np.any(truth > 3) or np.any(predictions > 3):
        raise ValueError('label IDs must be between zero and three')
    valid = truth != 0
    return np.bincount(4 * truth[valid] + predictions[valid], minlength=16).reshape(4, 4)


def score(matrix):
    iou = {}
    for index, name in enumerate(('floor', 'static', 'movable'), start=1):
        tp = matrix[index, index]
        union = matrix[index, :].sum() + matrix[:, index].sum() - tp
        iou[name] = float(tp / union) if union else None
    defined = [value for value in iou.values() if value is not None]
    return {'mIoU': sum(defined)/len(defined) if defined else None,
            'class_IoU': iou, 'confusion': matrix.tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('truth', type=Path)
    parser.add_argument('prediction', type=Path)
    args = parser.parse_args()
    files = sorted(args.truth.glob('*.png'))
    if not files:
        parser.error('ground truth folder has no PNGs')
    matrix = np.zeros((4, 4), dtype=np.int64)
    for filename in files:
        reference = cv2.imread(str(filename), cv2.IMREAD_UNCHANGED)
        prediction = cv2.imread(str(args.prediction / filename.name), cv2.IMREAD_UNCHANGED)
        if reference is None or prediction is None:
            parser.error(f'could not read paired image: {filename.name}')
        try:
            matrix += confusion(reference, prediction)
        except ValueError as error:
            parser.error(f'{filename.name}: {error}')
    print(json.dumps(score(matrix), indent=2))


if __name__ == '__main__':
    main()
