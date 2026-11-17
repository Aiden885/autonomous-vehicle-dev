"""Spatial index for static obstacle boxes."""
from collections import defaultdict
import math


class StaticObstacleIndex:
    """Broad-phase XY lookup; huge aggregate boxes stay in a separate list."""

    def __init__(self, obstacles, tile_size=32.0):
        self.tile_size = tile_size
        self.cells = defaultdict(list)
        self.aggregates = []
        for obstacle in obstacles:
            left, bottom, right, top = obstacle["_xy_bounds"]
            xs = range(math.floor(left/tile_size), math.floor(right/tile_size)+1)
            ys = range(math.floor(bottom/tile_size), math.floor(top/tile_size)+1)
            if len(xs)*len(ys) > 64:
                self.aggregates.append(obstacle)
            else:
                for ix in xs:
                    for iy in ys:
                        self.cells[(ix, iy)].append(obstacle)

    def query(self, bounds):
        left, bottom, right, top = bounds
        result = {obstacle["id"]: obstacle for obstacle in self.aggregates}
        for ix in range(math.floor(left/self.tile_size), math.floor(right/self.tile_size)+1):
            for iy in range(math.floor(bottom/self.tile_size), math.floor(top/self.tile_size)+1):
                result.update((obstacle["id"], obstacle) for obstacle in self.cells.get((ix, iy), ()))
        return list(result.values())
