"""Bounded sparse occupancy and semantic evidence in a gravity-aligned frame."""

from dataclasses import dataclass

import numpy as np

from .geometry import FLOOR, MOVABLE, STATIC


@dataclass(slots=True)
class Voxel:
    floor: int = 0
    solid: int = 0
    movable: int = 0
    dynamic_until: float = 0.0
    last_seen: float = 0.0


class VoxelMap:
    def __init__(self, resolution=0.05, radius=6.0, max_voxels=160_000, dynamic_ttl=1.2):
        if resolution <= 0 or radius <= resolution or max_voxels < 1 or dynamic_ttl <= 0:
            raise ValueError('Invalid voxel map limits')
        self.resolution = resolution
        self.radius = radius
        self.max_voxels = max_voxels
        self.dynamic_ttl = dynamic_ttl
        self.voxels = {}

    def key(self, point):
        return tuple(np.floor(np.asarray(point) / self.resolution).astype(np.int32))

    def integrate(self, points, labels, origin, center, now):
        if len(points) != len(labels):
            raise ValueError('each point needs a class label')
        limit = self.radius
        keep = np.max(np.abs(points[:, :2]-center[:2]), axis=1) <= limit
        points, labels = points[keep], labels[keep]
        keys = np.floor(points/self.resolution).astype(np.int32)
        hits = {}
        for xyz, label in zip(keys, labels):
            key = tuple(xyz)
            if key not in hits or label > hits[key]:
                hits[key] = int(label)

        # A bounded set of free-space rays clears old occupied evidence.
        sample_indices = np.linspace(0, len(points)-1, min(144, len(points)), dtype=int)
        clear_keys = set()
        for index in sample_indices:
            point = points[index]
            count = int(np.linalg.norm(point-origin)/self.resolution)
            if count < 3:
                continue
            # Sample at half-voxel intervals and exclude the endpoint region.
            fractions = np.arange(1, 2*count-2) / (2*count)
            ray = origin + fractions[:, None] * (point-origin)
            for row in np.floor(ray/self.resolution).astype(np.int32):
                key = tuple(row)
                if key in self.voxels and key not in hits:
                    clear_keys.add(key)
        for key in clear_keys:
            voxel = self.voxels[key]
            voxel.solid = max(0, voxel.solid-1)
            voxel.movable = 0
            voxel.dynamic_until = 0.0

        for key, label in hits.items():
            voxel = self.voxels.setdefault(key, Voxel())
            voxel.last_seen = now
            if label == MOVABLE:
                voxel.movable = min(8, voxel.movable+3)
                voxel.dynamic_until = now + self.dynamic_ttl
            elif label == FLOOR:
                voxel.floor = min(8, voxel.floor+2)
                voxel.solid = max(0, voxel.solid-1)
            elif label == STATIC:
                voxel.solid = min(8, voxel.solid+2)
                voxel.floor = max(0, voxel.floor-1)

        self.prune(center, now)

    def prune(self, center, now):
        edge = int(np.ceil(self.radius/self.resolution))
        cx, cy, _ = self.key(center)
        for key, voxel in list(self.voxels.items()):
            if voxel.dynamic_until <= now:
                voxel.movable = 0
            if (abs(key[0]-cx) > edge or abs(key[1]-cy) > edge
                    or not (voxel.floor or voxel.solid or voxel.movable)):
                del self.voxels[key]
        overflow = len(self.voxels) - self.max_voxels
        if overflow > 0:
            for key in sorted(self.voxels, key=lambda k: self.voxels[k].last_seen)[:overflow]:
                del self.voxels[key]

    def classified(self, now):
        for key, voxel in self.voxels.items():
            if voxel.movable and voxel.dynamic_until > now:
                yield key, MOVABLE
            elif voxel.solid >= 2:
                yield key, STATIC
            elif voxel.floor >= 2:
                yield key, FLOOR

    def project(self, center, floor_z, now, max_height=1.8):
        """Return OccupancyGrid costs and integer origin. Unknown stays unknown."""
        half = int(np.ceil(self.radius / self.resolution))
        cx, cy, _ = self.key(center)
        ox, oy = cx-half, cy-half
        grid = np.full((2*half+1, 2*half+1), -1, dtype=np.int8)
        for (x, y, z), label in self.classified(now):
            ix, iy = x-ox, y-oy
            if not (0 <= ix < grid.shape[1] and 0 <= iy < grid.shape[0]):
                continue
            height = (z+0.5)*self.resolution - floor_z
            if label != FLOOR and 0.04 <= height <= max_height:
                grid[iy, ix] = max(grid[iy, ix], 100 if label == STATIC else 99)
            elif label == FLOOR and abs(height) < 0.25 and grid[iy, ix] < 0:
                grid[iy, ix] = 0
        return grid, (ox*self.resolution, oy*self.resolution)
