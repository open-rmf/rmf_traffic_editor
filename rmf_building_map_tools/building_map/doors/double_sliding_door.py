from xml.etree.ElementTree import Element, SubElement
from .door import Door


class DoubleSlidingDoor(Door):
    def __init__(self, door_edge, level_elevation):
        super().__init__(door_edge, level_elevation)
        self.right_left_ratio = 1.0
        if 'right_left_ratio' in door_edge.params:
            self.right_left_ratio = door_edge.params['right_left_ratio'].value

    def generate(self, world_ele, options):
        right_segment_length = \
            (self.right_left_ratio / (1 + self.right_left_ratio)) * self.length
        left_segment_length = self.length - right_segment_length

        self.generate_sliding_section(
            'right',
            right_segment_length - 0.01,
            self.length / 2 - right_segment_length / 2,
            (0.0, right_segment_length),
            options)

        self.generate_sliding_section(
            'left',
            left_segment_length - 0.01,
            -self.length / 2 + left_segment_length / 2,
            (-left_segment_length, 0.0),
            options)

        if not self.plugin == 'none':
            plugin_ele = SubElement(self.model_ele, 'plugin')
            plugin_ele.set('name', 'door')
            plugin_ele.set('filename', 'libdoor.so')
            plugin_params = {
                'v_max_door': '0.2',
                'a_max_door': '0.2',
                'a_nom_door': '0.08',
                'dx_min_door': '0.001',
                'f_max_door': '100.0'
            }
            for param_name, param_value in plugin_params.items():
                ele = SubElement(plugin_ele, param_name)
                ele.text = param_value

            door_ele = SubElement(plugin_ele, 'door')
            door_ele.set('name', self.name)
            door_ele.set('type', 'DoubleSlidingDoor')
            door_ele.set('left_joint_name', 'left_joint')
            door_ele.set('right_joint_name', 'right_joint')

        world_ele.append(self.model_ele)
