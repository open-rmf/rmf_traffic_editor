from xml.etree.ElementTree import Element, SubElement
from .door import Door


class SlidingDoor(Door):
    def __init__(self, door_edge, level_elevation):
        super().__init__(door_edge, level_elevation)

    def generate(self, world_ele, options):
        self.generate_sliding_section(
            'right',
            self.length - 0.01,
            0,
            (0.0, self.length),
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
            door_ele.set('type', 'SlidingDoor')
            door_ele.set('left_joint_name', 'empty_joint')
            door_ele.set('right_joint_name', 'right_joint')

        world_ele.append(self.model_ele)
