import math
import os
import shutil

from xml.etree.ElementTree import SubElement

from .param_value import ParamValue


class Hole:
    def __init__(self, yaml_node):
        self.vertex_indices = []
        for v_idx in yaml_node['vertices']:
            self.vertex_indices.append(v_idx)

        self.params = {}
        if 'parameters' in yaml_node and yaml_node['parameters']:
            for param_name, param_yaml in yaml_node['parameters'].items():
                self.params[param_name] = ParamValue(param_yaml)

    def __str__(self):
        return f'hole ({len(self.vertex_indices)} vertices)'

    def __repr__(self):
        return self.__str__()
