import math
from .vertex import Vertex


class Vector2d:
    def __init__(self, coords=[0.0, 0.0]):
        self.x = coords[0]
        self.y = coords[1]

    def init_with_2_vertex(self, v0, v1):
        # pointing from v0 to v1
        assert(isinstance(v0, Vertex))
        assert(isinstance(v1, Vertex))
        self.x = v1.x - v0.x
        self.y = v1.y - v0.y

    def get_length(self):
        return math.sqrt(self.x**2 + self.y**2)

    def get_unit(self):
        norm = self.get_length()
        return Vector2d([self.x / norm, self.y / norm])

    def get_normal_unit(self):
        unit = self.get_unit()
        return Vector2d([-unit.y, unit.x])

    def get_dot(self, vector2d):
        assert(isinstance(vector2d, Vector2d))
        return self.x * vector2d.x + self.y * vector2d.y

    def get_orientation(self):
        return math.atan2(self.y, self.x)

    def get_cross(self, vector2d):
        assert(isinstance(vector2d, Vector2d))
        return self.x * vector2d.y - self.y * vector2d.x
