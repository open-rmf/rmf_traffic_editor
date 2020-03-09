from xml.etree.ElementTree import SubElement


class Model:
    def __init__(self, yaml_node):
        self.name = yaml_node['name']
        self.model_name = yaml_node['model_name']
        self.x = yaml_node['x']
        self.y = -yaml_node['y']
        self.z = 0.0
        if 'z' in yaml_node:
            self.z = yaml_node['z']
        self.yaw = yaml_node['yaw']

    def generate(self, world_ele, model_cnt, scale):
        include_ele = SubElement(world_ele, 'include')
        name_ele = SubElement(include_ele, 'name')
        name_ele.text = f'{self.model_name}_{model_cnt}'
        uri_ele = SubElement(include_ele, 'uri')
        uri_ele.text = f'model://{self.model_name}'
        pose_ele = SubElement(include_ele, 'pose')
        x = self.x * scale
        y = self.y * scale
        z = self.z * scale
        pose_ele.text = f'{x} {y} {z} 0 0 {self.yaw + 1.5707}'

        # hack... for now, everything other than robots is static (?)
        non_static_model_names = [
          'Sesto',
          'MiR100',
          'Magni'
        ]
        if self.name not in non_static_model_names:
            static_ele = SubElement(include_ele, 'static')
            static_ele.text = 'true'
