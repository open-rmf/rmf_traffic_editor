from xml.etree.ElementTree import Element, SubElement
from .door import Door

class DoubleSwingDoor(Door):
    def __init__(self, door_edge):
        super().__init__(door_edge)
        print(f'DoubleSwingDoor({self.name})')

    def generate(self, world_ele):
        print('DoubleSwingDoor.generate()')

        self.generate_swing_section(
            'left',
            self.length / 2 - 0.01,
            -self.length / 4,
            (-1.6, 0),
            (-self.length / 4, 0, 0))

        self.generate_swing_section(
            'right',
            self.length / 2 - 0.01,
            self.length / 4,
            (0, 1.6),
            (self.length / 4, 0, 0))

        plugin_ele = SubElement(self.model_ele, 'plugin')
        plugin_ele.set('name', f'plugin_{self.name}')
        plugin_ele.set('filename', 'libdoor.so')
        plugin_params = {
          'v_max_door': '0.5',
          'a_max_door': '0.2',
          'a_nom_door': '0.15',
          'dx_min_door': '0.01',
          'f_max_door': '100.0'
        }
        for param_name, param_value in plugin_params.items():
            ele = SubElement(plugin_ele, param_name)
            ele.text = param_value

        door_ele = SubElement(plugin_ele, 'door')
        door_ele.set('name', self.name)
        door_ele.set('type', 'DoubleSwingDoor')
        door_ele.set('left_joint_name', 'left_joint')
        door_ele.set('right_joint_name', 'right_joint')

        world_ele.append(self.model_ele)
