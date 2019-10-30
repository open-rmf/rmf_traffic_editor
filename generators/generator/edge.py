import math
from .param_value import ParamValue


class Edge:
    def __init__(self, yaml_node, vertices):
        self.start_idx = int(yaml_node[0])
        self.end_idx = int(yaml_node[1])
        self.params = {}
        if len(yaml_node[2]) > 0:
            for param_name, param_yaml in yaml_node[2].items():
                self.params[param_name] = ParamValue(param_yaml)

        self.calc_statistics(vertices)

    def calc_statistics(self, vertices):
        x1 = vertices[self.start_idx].x
        y1 = vertices[self.start_idx].y
        x2 = vertices[self.end_idx].x
        y2 = vertices[self.end_idx].y
        dx = x1 - x2
        dy = y1 - y2
        self.length = math.sqrt(dx*dx + dy*dy)
        self.x = (x1 + x2) / 2.0
        self.y = (y1 + y2) / 2.0
        self.yaw = math.atan2(dy, dx)

    def is_bidirectional(self):
        if 'bidirectional' not in self.params:
            return False
        p = self.params['bidirectional']
        if p.type != ParamValue.BOOL:
            raise ValueError('expected bidirectional param to be Boolean')
        return p.value

    def orientation(self):
        if 'orientation' not in self.params:
            return None
        return self.params['orientation'].value

    def reverse_orientation(self):
        if 'orientation' not in self.params:
            return None
        if self.params['orientation'].value == 'forward':
            return 'backward'
        elif self.params['orientation'].value == 'backward':
            return 'forward'
        else:
            return ''
