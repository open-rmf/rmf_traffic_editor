from xml.etree.ElementTree import SubElement


class Model:
    def __init__(self, name, yaml_node):
        self.name = name
        self.model_name = yaml_node['model_name']
        self.x = yaml_node['x']
        self.y = -yaml_node['y']
        self.z = 0.0
        if 'z' in yaml_node:
            self.z = yaml_node['z']
        else:
            print('parsed a deprecated .building.yaml, model should have a z'
                  ' field, setting elevation to 0.0 for now')

        # temporary hack: whitelist of robot models which must be non-static
        non_static_model_names = [
          'Sesto',
          'MiR100',
          'Magni'
        ]
        if self.model_name in non_static_model_names:
            self.static = False
        else:
            if 'static' in yaml_node:
                self.static = yaml_node['static']
            else:
                self.static = True

        self.yaw = yaml_node['yaw']

    def generate(self, world_ele, transform, elevation):
        include_ele = SubElement(world_ele, 'include')
        name_ele = SubElement(include_ele, 'name')
        name_ele.text = self.name
        uri_ele = SubElement(include_ele, 'uri')
        uri_ele.text = f'model://{self.model_name}'
        pose_ele = SubElement(include_ele, 'pose')
        x, y = transform.transform_point((self.x, self.y))
        z = self.z + elevation
        yaw = self.yaw + transform.rotation
        pose_ele.text = f'{x} {y} {z} 0 0 {yaw}'

        static_ele = SubElement(include_ele, 'static')
        static_ele.text = str(self.static)
