import math


class Fiducial:
    def __init__(self, yaml_node):
        self.x = yaml_node[0]
        self.y = -yaml_node[1]
        self.name = yaml_node[2]

    def distance(self, f):
        """ Calculate distance to another fiducial """
        dx = f.x - self.x
        dy = f.y - self.y
        return math.sqrt(dx*dx + dy*dy)
