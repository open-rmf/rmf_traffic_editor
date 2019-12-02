from xml.etree.ElementTree import SubElement


class Model:
    def __init__(self, yaml_node, scale):
        self.name = yaml_node['name']
        self.model_name = yaml_node['model_name']
        self.x = yaml_node['x'] * scale
        self.y = -yaml_node['y'] * scale
        self.z = 0.0  # todo
        self.yaw = yaml_node['yaw']

    def generate(self, world_ele, model_cnt):
        include_ele = SubElement(world_ele, 'include')
        name_ele = SubElement(include_ele, 'name')
        name_ele.text = f'{self.model_name}_{model_cnt}'
        uri_ele = SubElement(include_ele, 'uri')
        uri_ele.text = f'model://{self.model_name}'
        pose_ele = SubElement(include_ele, 'pose')
        pose_ele.text = f'{self.x} {self.y} {self.z} 0 0 {self.yaw + 1.5707}'

        # hack... for now, everything other than robots is static (?)
        non_static_model_names = [
          'Sesto',
          'MiR100'
        ]
        if not self.name in non_static_model_names:
            static_ele = SubElement(include_ele, 'static')
            static_ele.text = 'true'


