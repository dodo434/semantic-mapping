from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros_ws/src/semantic_mapping'))
from semantic_mapping.geometry import (FLOOR, MOVABLE, STATIC, backproject, fit_floor,
                                       floor_inliers, project_exclusions, transform_points)
from semantic_mapping.voxel_map import VoxelMap


class MappingTests(unittest.TestCase):
    def test_floor_plane_rejects_a_vertical_wall_and_non_floor_height(self):
        x, y = np.meshgrid(np.arange(8)*0.1, np.arange(8)*0.1)
        floor = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
        wall = np.column_stack((x.ravel(), np.ones(x.size), y.ravel()+0.16))
        points = np.concatenate((floor, wall))
        plane = fit_floor(points, 0.0, np.random.default_rng(42))
        labels = floor_inliers(points, 0.0, plane)
        self.assertTrue(labels[:len(floor)].all())
        self.assertFalse(labels[len(floor):].any())

    def test_backproject_transform_and_project(self):
        depth = np.array([[0, 0, 0], [0, 2.0, np.nan], [0, 0, 0]])
        k = (10.0, 10.0, 1.0, 1.0)
        points, yy, xx = backproject(depth, k, 1, 0.2, 3.0)
        self.assertEqual((yy.tolist(), xx.tolist()), ([1], [1]))
        transform = SimpleNamespace(rotation=SimpleNamespace(x=0, y=0, z=0, w=1),
                                    translation=SimpleNamespace(x=0.2, y=0, z=0))
        shifted = transform_points(points, transform)
        np.testing.assert_allclose(shifted, [[0.2, 0, 2]])
        mask = project_exclusions(shifted, k, (3, 3))
        self.assertEqual(np.argwhere(mask).tolist(), [[1, 2]])

    def test_move_decay_preserves_static_evidence(self):
        mapping = VoxelMap()
        base = np.array([0., 0., 0.])
        origin = np.array([0., 0., 0.5])
        wall = np.array([[1.04, 0.02, 0.5]])
        mapping.integrate(wall, np.array([STATIC]), origin, base, 10.0)
        mapping.integrate(wall, np.array([MOVABLE]), origin, base, 10.1)
        key = mapping.key(wall[0])
        self.assertEqual(dict(mapping.classified(10.2))[key], MOVABLE)
        self.assertEqual(dict(mapping.classified(12.0))[key], STATIC)

    def test_ray_clears_moved_endpoint_and_grid_keeps_unknown(self):
        mapping = VoxelMap()
        base = np.array([0., 0., 0.])
        origin = np.array([0., 0., 0.5])
        obstacle = np.array([[1.03, 0., 0.5], [1.03, 0., 0.0]])
        mapping.integrate(obstacle, np.array([MOVABLE, FLOOR]), origin, base, 10.0)
        before, origin_xy = mapping.project(base, 0., 10.1)
        x = int((obstacle[0, 0]-origin_xy[0])/mapping.resolution)
        y = int((0.-origin_xy[1])/mapping.resolution)
        self.assertEqual(before[y, x], 99)
        farther = np.array([[2.03, 0., 0.5]])
        mapping.integrate(farther, np.array([STATIC]), origin, base, 10.2)
        after, _ = mapping.project(base, 0., 10.2)
        self.assertEqual(after[y, x], 0)
        self.assertEqual(after[0, 0], -1)

    def test_window_and_voxel_cap(self):
        mapping = VoxelMap(max_voxels=1)
        base = np.array([0., 0., 0.])
        origin = np.array([0., 0., 0.5])
        points = np.array([[1., 0., 0.5], [1.5, 0., 0.5]])
        mapping.integrate(points, np.array([STATIC, STATIC]), origin, base, 10.0)
        self.assertEqual(len(mapping.voxels), 1)
        mapping.prune(np.array([20., 0., 0.]), 10.1)
        self.assertEqual(len(mapping.voxels), 0)
