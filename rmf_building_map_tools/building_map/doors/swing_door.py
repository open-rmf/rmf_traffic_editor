from xml.etree.ElementTree import Element, SubElement
from .door import Door


class SwingDoor(Door):
    def __init__(self, door_edge, level_elevation):
        super().__init__(door_edge, level_elevation)
        motion_degrees = door_edge.params['motion_degrees'].value
        self.motion_radians = 3.14 * motion_degrees / 180.0
        self.motion_direction = door_edge.params['motion_direction'].value

    def generate(self, world_ele):
        # This is configured to be negative by default to reflect how it is
        # rendered on rmf_traffic_editor.
        axis = 'z' if self.motion_direction < 0 else '-z'

        self.generate_swing_section(
            'right',
            self.length - 0.01,
            0,
            (0, self.motion_radians),
            (self.length / 2, 0, 0),
            axis)

        if not self.plugin == 'none':
            plugin_ele = SubElement(self.model_ele, 'plugin')
            plugin_ele.set('name', 'register_component')
            plugin_ele.set('filename', 'libregister_component.so')
            component_ele = SubElement(plugin_ele, 'component')
            component_ele.set('name', 'Door')
            plugin_params = {
                'v_max_door': '0.5',
                'a_max_door': '0.3',
                'a_nom_door': '0.15',
                'dx_min_door': '0.01',
                'f_max_door': '500.0',
                'ros_interface': 'true'
            }
            for param_name, param_value in plugin_params.items():
                ele = SubElement(component_ele, param_name)
                ele.text = param_value

            door_ele = SubElement(component_ele, 'door')
            door_ele.set('name', self.name)
            door_ele.set('type', 'SwingDoor')
            door_ele.set('left_joint_name', 'empty_joint')
            door_ele.set('right_joint_name', 'right_joint')

        world_ele.append(self.model_ele)
