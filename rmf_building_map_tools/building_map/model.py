from xml.etree.ElementTree import SubElement


class Model:
    def __init__(self, name, yaml_node, coordinate_system):
        self.name = name
        self.model_name = yaml_node['model_name']
        self.x = yaml_node['x']
        self.y = yaml_node['y'] * coordinate_system.y_flip_scalar()
        self.z = 0.0
        if 'z' in yaml_node:
            self.z = yaml_node['z']
        else:
            print('parsed a deprecated .building.yaml, model should have a [z]'
                  ' field, setting [elevation] to 0.0 for now')

        self.dispensable = False
        if 'dispensable' in yaml_node:
            self.dispensable = yaml_node['dispensable']
        else:
            print('parsed a deprecated .building.yaml, model should have a '
                  ' [dispensable] field, setting [dispensable] to false for '
                  'now')

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

        # BH: Temporary holder to keep to_yaml working for now
        self.original_x = yaml_node['x']
        self.original_y = yaml_node['y']

    def to_yaml(self, coordinate_system):
        y = {}
        y['x'] = self.x
        y['y'] = self.y * coordinate_system.y_flip_scalar()
        y['z'] = self.z
        y['model_name'] = self.model_name
        y['name'] = self.name
        y['yaw'] = self.yaw
        y['static'] = self.static
        y['dispensable'] = self.dispensable
        return y

    def generate(self, world_ele, elevation, transform):
        model_name = self.name
        if self.dispensable:
            model_name += '_dispensable'

        include_ele = SubElement(world_ele, 'include')
        name_ele = SubElement(include_ele, 'name')
        name_ele.text = model_name
        uri_ele = SubElement(include_ele, 'uri')
        uri_ele.text = f'model://{self.model_name}'
        pose_ele = SubElement(include_ele, 'pose')

        x_t, y_t = transform.transform_point([self.x, self.y])
        x_t = x_t - transform.x
        y_t = y_t - transform.y

        z = self.z + elevation
        yaw = self.yaw + transform.rotation
        pose_ele.text = f'{x_t} {y_t} {z} 0 0 {yaw}'

        static_ele = SubElement(include_ele, 'static')
        static_ele.text = str(self.static)
