from xml.etree.ElementTree import Element, SubElement
from .door import Door


class DoubleSwingDoor(Door):
    def __init__(self, door_edge, level_elevation):
        super().__init__(door_edge, level_elevation)
        motion_degrees = door_edge.params['motion_degrees'].value
        self.motion_radians = 3.14 * motion_degrees / 180.0
        self.motion_direction = door_edge.params['motion_direction'].value

    def generate(self, world_ele, options):
        if self.motion_direction > 0:
            x_flip_sign = 1.0
        else:
            x_flip_sign = -1.0

        self.generate_swing_section(
            'right',
            self.length / 2 - 0.01,
            x_flip_sign * -self.length / 4,
            (0, self.motion_radians),
            (x_flip_sign * -self.length / 4, 0, 0),
            options)

        self.generate_swing_section(
            'left',
            self.length / 2 - 0.01,
            x_flip_sign * self.length / 4,
            (-self.motion_radians, 0),
            (x_flip_sign * self.length / 4, 0, 0),
            options)

        if not self.plugin == 'none':
            plugin_ele = SubElement(self.model_ele, 'plugin')
            plugin_ele.set('name', 'door')
            plugin_ele.set('filename', 'libdoor.so')
            plugin_params = {
                'v_max_door': '0.5',
                'a_max_door': '0.3',
                'a_nom_door': '0.15',
                'dx_min_door': '0.01',
                'f_max_door': '500.0'
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
