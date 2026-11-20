"""Geometry shared by stereo mask projection and semantic fusion."""

import numpy as np

MOVABLE = 3
STATIC = 2
FLOOR = 1


def backproject(depth, k, step, minimum, maximum, rays=None):
    """Return valid XYZ points in the depth optical frame and source indices."""
    yy, xx = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
    z = depth[yy, xx]
    good = np.isfinite(z) & (z >= minimum) & (z <= maximum)
    yy, xx, z = yy[good], xx[good], z[good]
    if rays is None:
        fx, fy, cx, cy = k
        xy = np.column_stack(((xx-cx)/fx, (yy-cy)/fy))
    else:
        xy = rays[yy, xx]
    return np.column_stack((xy*z[:, None], z)), yy, xx


def transform_points(points, transform):
    """Apply an optical-frame ROS transform without assuming axis order."""
    q = transform.rotation
    t = transform.translation
    vector = np.array([q.x, q.y, q.z])
    twice_cross = 2 * np.cross(vector, points)
    return points + q.w * twice_cross + np.cross(vector, twice_cross) + [t.x, t.y, t.z]


def project_exclusions(points, k, size):
    """Project movable points into the target camera's input pixels."""
    width, height = size
    mask = np.zeros((height, width), dtype=np.uint8)
    points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > 0.02)]
    fx, fy, cx, cy = k
    xy = np.rint(points[:, :2] / points[:, 2, None] * [fx, fy] + [cx, cy])
    valid = (xy[:, 0] >= 0) & (xy[:, 0] < width) & (xy[:, 1] >= 0) & (xy[:, 1] < height)
    x, y = xy[valid].astype(np.intp).T
    mask[y, x] = 255
    return mask


def fit_floor(points, expected_z, rng):
    """Fit an upward plane near the expected ground height."""
    candidates = points[np.abs(points[:, 2] - expected_z) <= 0.25]
    if len(candidates) < 30:
        return None
    # Score only the near-floor band and reject vertical or steep surfaces.
    best_count = 29
    best = None
    for _ in range(32):
        a, b, c = candidates[rng.choice(len(candidates), size=3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-6 or abs(normal[2]) / length < 0.94:
            continue
        normal /= length
        score = np.count_nonzero(np.abs((candidates-a) @ normal) < 0.04)
        if score > best_count:
            best, best_count = (a, normal), score
    return best


def floor_inliers(points, expected_z, plane):
    if plane is None:
        return np.zeros(len(points), dtype=bool)
    point, normal = plane
    return (np.abs((points-point) @ normal) < 0.045) & (np.abs(points[:, 2]-expected_z) < 0.25)
