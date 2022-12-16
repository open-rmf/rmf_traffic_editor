from .param_value import ParamValue


class Vertex:
    def __init__(self, yaml_node, coordinate_system):
        self.x = float(yaml_node[0])
        self.y = float(yaml_node[1]) * coordinate_system.y_flip_scalar()
        self.z = float(yaml_node[2])  # currently always 0
        self.name = yaml_node[3]

        self.params = {}
        if len(yaml_node) > 4 and len(yaml_node[4]) > 0:
            for param_name, param_yaml in yaml_node[4].items():
                self.params[param_name] = ParamValue(param_yaml)

    def xy(self):
        return (self.x, self.y)

    def to_yaml(self, coordinate_system):
        y = [
            self.x,
            self.y * coordinate_system.y_flip_scalar(),
            self.z,
            self.name,
            {}
        ]
        for param_name, param_value in self.params.items():
            y[4][param_name] = param_value.to_yaml()
        return y
