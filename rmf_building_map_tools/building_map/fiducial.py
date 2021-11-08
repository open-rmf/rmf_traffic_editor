import math


class Fiducial:
    def __init__(self, yaml_node, coordinate_system):
        self.x = yaml_node[0]
        self.y = yaml_node[1] * coordinate_system.y_flip_scalar()
        self.name = yaml_node[2]

    def to_yaml(self, coordinate_system):
        return [
            self.x,
            self.y * coordinate_system.y_flip_scalar(),
            self.name
        ]

    def distance(self, f):
        """ Calculate distance to another fiducial """
        dx = f.x - self.x
        dy = f.y - self.y
        return math.sqrt(dx*dx + dy*dy)

    def bearing(self, f):
        """
        Calculate the bearing angle to another fiducial.

        If the fiducial 'f' is directly east (+x) of the current fiducial,
        This function will return 0. If 'f' is directly north (+y), this
        function will return pi/2.
        """
        dx = f.x - self.x
        dy = f.y - self.y
        return math.atan2(dy, dx)
