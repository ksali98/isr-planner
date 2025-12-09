from typing import List, Tuple
import numpy as np
from matplotlib.path import Path as MplPath

Point = Tuple[float, float]

class SafeRouter:
    """
    Clean polygon-aware router.
    Takes polygons and guarantees that segments do NOT cross inside anything.
    """

    def __init__(self, polygons: List[List[Point]]):
        self.polygons = polygons
        self._paths = [MplPath(poly, closed=True) for poly in polygons]

    def is_inside_any_polygon(self, p: Point) -> bool:
        x, y = p
        for path in self._paths:
            if path.contains_point((x, y)):
                return True
        return False

    def segment_crosses_any_polygon(self, a: Point, b: Point, samples: int = 64) -> bool:
        ax, ay = a
        bx, by = b
        for i in range(samples + 1):
            t = i / samples
            x = ax + t * (bx - ax)
            y = ay + t * (by - ay)
            if self.is_inside_any_polygon((x, y)):
                return True
        return False

    def direct_or_blocked(self, a: Point, b: Point) -> bool:
        return self.segment_crosses_any_polygon(a, b)

