import math
import numpy as np


class Transform:
    """This class represents an 2D rotation, scaling, and translation."""

    def __init__(self):
        self.set_rotation(0.0)
        self.set_translation(0.0, 0.0)
        self.set_scale(1.0)

    def set_rotation(self, rotation):
        """Calculate rotation matrix"""
        self.rotation = rotation
        cr = math.cos(self.rotation)
        sr = math.sin(self.rotation)
        self.rot_mat = np.array([[cr, -sr], [sr, cr]])

    def set_translation(self, tx, ty):
        self.translation = (tx, ty)
        self.t_vec = np.array([[self.translation[0]], [self.translation[1]]])

    def set_scale(self, scale):
        self.scale = scale

    def transform_point(self, p):
        vec = np.array([[p[0]], [p[1]]])
        transformed = (self.rot_mat @ vec) * self.scale + self.t_vec
        return (np.asscalar(transformed[0]), np.asscalar(transformed[1]))
