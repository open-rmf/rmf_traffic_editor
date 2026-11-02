import numpy as np


class Zone:
    def __init__(self, yaml_node, name, transform, coordinate_system):
        self.name = name
        print(f'parsing zone {name}')

        self.level = str(yaml_node['level'])
        self.depth = float(yaml_node['depth'])
        self.width = float(yaml_node['width'])
        self.yaw = (float(yaml_node['yaw']) + transform.rotation)

        # Calculating the center of the zone
        self.raw_pos = (
            float(yaml_node['x']),
            float(yaml_node['y'] * coordinate_system.y_flip_scalar()),
        )
        self.x, self.y = transform.transform_point(self.raw_pos)
        self.type = 'Default'

        self.internal_vertices = {}
        if 'internal_vertices' in yaml_node:
            self.internal_vertices = self.parse_internal_vertices(yaml_node['internal_vertices'])

        self.external_vertices = {}
        if 'external_vertices' in yaml_node:
            self.external_vertices = self.parse_external_vertices(
                yaml_node['external_vertices'])

    def contains_point(self, x, y):
        # Test whether a point in global coordinates lies inside the zone
        # rectangle.
        dx = x - self.x
        dy = y - self.y
        s = np.sin(self.yaw)
        c = np.cos(self.yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        return abs(local_x) <= self.width / 2 and abs(local_y) <= self.depth / 2

    def parse_internal_vertices(self, yaml_node):
        vertices = {}
        for vertex_name, vertex_yaml in yaml_node.items():
            x = (
                vertex_yaml['x'] * np.cos(self.yaw)
                + vertex_yaml['y'] * np.sin(self.yaw)
                + self.x
            )
            y = (
                vertex_yaml['x'] * np.sin(self.yaw)
                - vertex_yaml['y'] * np.cos(self.yaw)
                + self.y
            )
            vertices[vertex_name] = {
                'name': vertex_name,
                'location': [float(x), float(y)],
                'priority': int(vertex_yaml['priority']),
                'group': vertex_yaml['group'],
            }
        return vertices

    def parse_external_vertices(self, yaml_node):
        vertices = {}
        for vertex_name, vertex_yaml in yaml_node.items():
            vertices[vertex_name] = {
                'name': vertex_name,
                'is_entry_point': bool(vertex_yaml['is_entry_point']),
                'is_exit_point': bool(vertex_yaml['is_exit_point']),
            }
        return vertices
