from typing import List, Tuple, Union
import numpy as np
                      
CONE_RADIUS = 0.15

class BoundingBox:
    """ Bounding box class """

    class Point:
        """ Point class """

        def __init__(self, x: float, y: float, z: float):
            self.x = x
            self.y = y
            self.z = z

        def __repr__(self):
            return f'Point(x={self.x}, y={self.y}, z={self.z})'

        def __str__(self):
            return f'Point(x={self.x}, y={self.y}, z={self.z})'

    class Bounds:
        """ Bounds class """

        def __init__(self, x: float, y: float, z: float):
            self.x_upper = x + CONE_RADIUS
            self.x_lower = x - CONE_RADIUS
            self.y_upper = y + CONE_RADIUS
            self.y_lower = y - CONE_RADIUS

        def __repr__(self):
            return f'Bounds(x_upper={self.x_upper}, x_lower={self.x_lower}, y_upper={self.y_upper}, y_lower={self.y_lower})'
        
        def __str__(self):
            return f'Bounds(x_upper={self.x_upper}, x_lower={self.x_lower}, y_upper={self.y_upper}, y_lower={self.y_lower})'

    def __init__(self, position: Tuple[float, float, float], points: List[List[float]] = []):
        self.position = self.Point(*position)
        self.bounds = self.Bounds(*position)
        self.points = []
        self.xs = []
        self.ys = []
        self.zs = []

        for point in points:
            p = self.Point(point[0], point[1], point[2])
            self.points.append(p)
            self.xs.append(p.x)
            self.ys.append(p.y)
            self.zs.append(p.z)

    def tuple_points(self):
        return [(p.x, p.y, p.z) for p in self.points]

    def center(self):
        """Centers the bounding box to origen"""
        if len(self.points) > 0:
            mean_x = np.mean(self.xs)
            mean_y = np.mean(self.ys)

            self.points = [(x, y, z) for x, y, z in zip(self.xs-mean_x, self.ys-mean_y, self.zs)]

    def sample(self, n: int):
        """Samples n points from the bounding box"""
        if len(self.points) > 0:
            self.points = np.array(self.points)[np.random.choice(len(self.points), n, replace=(len(self.points) < n))]

    def __contains__(self, contained: Union[Tuple[float, float, float], 'BoundingBox']) -> bool:
        if isinstance(contained, tuple) and len(contained) == 3:
            point = self.Point(*contained)
            return (self.bounds.x_lower <= point.x <= self.bounds.x_upper and
                self.bounds.y_lower <= point.y <= self.bounds.y_upper)
        elif isinstance(contained, BoundingBox):
            return (self.bounds.x_lower <= contained.bounds.x_lower <= self.bounds.x_upper or
                self.bounds.x_lower <= contained.bounds.x_upper <= self.bounds.x_upper or
                self.bounds.y_lower <= contained.bounds.y_lower <= self.bounds.y_upper or
                self.bounds.y_lower <= contained.bounds.y_upper <= self.bounds.y_upper)

    def __repr__(self):
        return f'BoundingBox(position={self.position}, n_points={len(self.points)})'

    def __str__(self):
        return f'BoundingBox(position={self.position}, n_points={len(self.points)})'

    def __bool__(self):
        return len(self.points) > 0
