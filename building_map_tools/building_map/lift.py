import yaml
import numpy as np
from .transform import Transform
from xml.etree.ElementTree import ElementTree, Element, SubElement

from .lift_utils import *

from .doors.double_sliding_door import DoubleSlidingDoor


class LiftDoor:
    def __init__(self, yaml_node, name):
        self.name = name
        self.door_type = yaml_node['door_type']
        # x & y coordinates are with respect to the centre of the cabin
        self.x = float(yaml_node['x'])
        self.y = float(yaml_node['y'])
        self.motion_axis_orientation = float(
            yaml_node['motion_axis_orientation'])
        self.width = float(yaml_node['width'])
        self.params = {'v_max_door': 0.3,
                       'a_max_door': 0.15,
                       'a_nom_door': 0.12,
                       'dx_min_door': 1e-06,
                       'f_max_door': 200.0}

    def generate_cabin_door(self, lift_model_ele, name):
        door_model_ele = SubElement(lift_model_ele, 'model')
        door_model_ele.set('name', name)
        door_pose = SubElement(door_model_ele, 'pose')
        if abs(self.x) >= abs(self.y):
            y = self.y
            if self.x > 0:
                x = self.x - 0.05
            else:
                x = self.x + 0.05
        else:
            x = self.x
            if self.y > 0:
                y = self.y - 0.05
            else:
                y = self.y + 0.05
        door_pose.text = f'{x} {y} 0 0 0 {self.motion_axis_orientation+1.571}'

        self.generate_door_link_and_joint(door_model_ele, parent='platform')

        self.generate_door_plugin(door_model_ele, name)

    def generate_shaft_door(self, world_ele, x, y, z, yaw, name):
        model_ele = SubElement(world_ele, 'model')
        model_ele.set('name', name)
        door_pose = SubElement(model_ele, 'pose')
        # tranformation
        x_new = x + self.x * np.cos(yaw) - self.y * np.sin(yaw)
        y_new = y + self.x * np.sin(yaw) + self.y * np.cos(yaw)
        yaw_new = yaw + self.motion_axis_orientation + 1.571
        door_pose.text = f'{x_new} {y_new} {z} 0 0 {yaw_new}'

        self.generate_door_link_and_joint(model_ele)

        ramp_size = [0.06, self.width, 0.05]
        ramp_pose = Element('pose')
        ramp_pose.text = '0 0 -0.025 0 0 0'
        model_ele.append(
            generate_box_link('ramp', ramp_size, ramp_pose, bitmask='0x02'))
        model_ele.append(
            generate_joint('ramp_joint', 'fixed', 'world', 'ramp'))

        self.generate_door_plugin(model_ele, name)

    def generate_door_link_and_joint(self, model_ele, parent='world'):
        door_size = [0.03, self.width / 2, 2.5]
        right_door_pose = Element('pose')
        right_door_pose.text = f'0 {self.width / 4} 1.25 0 0 0'

        model_ele.append(generate_box_link('right_door',
                                           door_size,
                                           right_door_pose,
                                           bitmask='0x02'))

        model_ele.append(generate_joint(f'right_joint',
                                        'prismatic',
                                        parent,
                                        'right_door',
                                        joint_axis='y',
                                        lower_limit=0,
                                        upper_limit=self.width / 2))

        left_door_pose = Element('pose')
        left_door_pose.text = f'0 {-self.width / 4} 1.25 0 0 0'

        model_ele.append(generate_box_link('left_door',
                                           door_size,
                                           left_door_pose,
                                           bitmask='0x02'))

        model_ele.append(generate_joint(f'left_joint',
                                        'prismatic',
                                        parent,
                                        'left_door',
                                        joint_axis='y',
                                        lower_limit=-self.width / 2,
                                        upper_limit=0))

    def generate_door_plugin(self, model_ele, name):
        plugin_ele = SubElement(model_ele, 'plugin')
        plugin_ele.set('name', f'{name}_plugin')
        plugin_ele.set('filename', 'libdoor.so')
        for param_name, param_value in self.params.items():
            ele = SubElement(plugin_ele, param_name)
            ele.text = f'{param_value}'
        door_ele = SubElement(plugin_ele, 'door')
        door_ele.set('left_joint_name', 'left_joint')
        door_ele.set('name', f'{name}')
        door_ele.set('right_joint_name', 'right_joint')
        door_ele.set('type', 'DoubleSlidingDoor')


class Lift:
    def __init__(self, yaml_node, name, transform, levels):
        self.name = name
        print(f'parsing lift {name}')

        self.reference_floor_name = yaml_node['reference_floor_name']
        self.depth = float(yaml_node['depth'])
        self.width = float(yaml_node['width'])
        self.yaw = float(yaml_node['yaw'])
        raw_pos = (float(yaml_node['x']), -float(yaml_node['y']))
        self.x, self.y = transform.transform_point(raw_pos)
        self.cabin_depth = self.depth - 0.1
        self.cabin_width = self.width - 0.1

        # default params
        self.cabin_mass = 800
        self.params = {
            'v_max_cabin': 2.0,
            'a_max_cabin': 1.2,
            'a_nom_cabin': 1.0,
            'dx_min_cabin': 0.001,
            'f_max_cabin': 25323.0}

        self.level_elevation = {}
        self.level_doors = {}
        self.door_names = []
        self.level_names = []
        if 'level_doors' in yaml_node:
            for level_name, door_names in yaml_node['level_doors'].items():
                self.level_doors[level_name] = door_names
                self.level_elevation[level_name] = levels[level_name].elevation
                self.door_names.append(door_names)
                self.level_names.append(level_name)

        self.doors = []
        if 'doors' in yaml_node:
            self.doors = self.parse_lift_doors(yaml_node['doors'])

    def parse_lift_doors(self, yaml_node):
        doors = []
        for lift_door_name, lift_door_yaml in yaml_node.items():
            # check if door is aligned to the edges
            x_diff = abs(float(lift_door_yaml['x']) - self.width/2)
            y_diff = abs(float(lift_door_yaml['y']) - self.depth/2)
            yaw_diff = float(lift_door_yaml['motion_axis_orientation']) % 1.571
            assert (x_diff <= 0.05 or y_diff <= 0.5)
            assert (yaw_diff <= 0.05 or yaw_diff >= 1.52)

            doors.append(LiftDoor(lift_door_yaml, lift_door_name))
        return doors

    def generate_shaft_doors(self, world_ele):
        for level_name, door_names in self.level_doors.items():
            for door in self.doors:
                if door.name in door_names:
                    name = f'ShaftDoor_{self.name}_{level_name}_{door.name}'
                    elevation = self.level_elevation[level_name]
                    door.generate_shaft_door(
                        world_ele, self.x, self.y, elevation, self.yaw, name)

    def generate_cabin(self, world_ele):
        # materials missing for now
        lift_model_name = f'{self.name}'
        lift_model_ele = SubElement(world_ele, 'model')
        lift_model_ele.set('name', lift_model_name)

        # main cabin link for actuation
        platform_name = 'platform'
        platform = SubElement(lift_model_ele, 'link')
        platform.set('name', platform_name)
        inertial = SubElement(platform, 'inertial')
        mass = SubElement(inertial, 'mass')
        mass.text = f'{self.cabin_mass}'

        # visuals and collisions for floor and walls of cabin
        floor_dims = [self.cabin_depth, self.cabin_width, 0.05]
        floor_name = 'floor'
        floor_pose = Element('pose')
        floor_pose.text = '0 0 -0.025 0 0 0'
        platform.append(
            generate_visual(
                floor_name,
                floor_pose,
                floor_dims))

        platform.append(
            generate_collision(
                floor_name,
                floor_pose,
                floor_dims))

        # TODO: Wall generation
        '''back_wall_name = 'back_wall'
        back_wall_dims = [0.05, self.cabin_width, 2.5]
        back_wall_pose = Element('pose')
        back_wall_pose.text = f'{-self.cabin_depth / 2 + 0.025} 0 1.25 0 0 0'
        platform.append(
            generate_visual(
                back_wall_name,
                back_wall_pose,
                back_wall_dims))

        platform.append(
            generate_collision(
                back_wall_name,
                back_wall_pose,
                back_wall_dims))

        side_wall_dims = [self.cabin_depth - 0.1, 0.05, 2.5]
        left_wall_name = 'left_wall'
        left_wall_pose = Element('pose')
        left_wall_pose.text = f'0 {self.cabin_width / 2 - 0.025} 1.25 0 0 0'
        platform.append(
            generate_visual(
                left_wall_name,
                left_wall_pose,
                side_wall_dims))

        platform.append(
            generate_collision(
                left_wall_name,
                left_wall_pose,
                side_wall_dims))

        right_wall_name = 'right_wall'
        right_wall_pose = Element('pose')
        right_wall_pose.text = f'0 {-self.cabin_width / 2 + 0.025} 1.25 0 0 0'
        platform.append(
            generate_visual(
                right_wall_name,
                right_wall_pose,
                side_wall_dims))

        platform.append(
            generate_collision(
                right_wall_name,
                right_wall_pose,
                side_wall_dims))

        front_wall_dims = [0.05, self.cabin_width / 4, 2.5]
        front_left_wall_name = 'front_left_wall'
        front_left_wall_pose = Element('pose')
        front_left_wall_pose.text = \
            f'{self.cabin_depth/2 - 0.025} {self.cabin_width*3/8} 1.25 0 0 0'
        platform.append(
            generate_visual(
                front_left_wall_name,
                front_left_wall_pose,
                front_wall_dims))

        platform.append(
            generate_collision(
                front_left_wall_name,
                front_left_wall_pose,
                front_wall_dims,
                bitmask='0x01'))

        front_right_wall_name = 'front_right_wall'
        front_right_wall_pose = Element('pose')
        front_right_wall_pose.text = \
            f'{self.cabin_depth/2 - 0.025} {-self.cabin_width*3/8} 1.25 0 0 0'
        platform.append(
            generate_visual(
                front_right_wall_name,
                front_right_wall_pose,
                front_wall_dims))

        platform.append(
            generate_collision(
                front_right_wall_name,
                front_right_wall_pose,
                front_wall_dims,
                bitmask='0x01'))'''

        # lift cabin actuation joint
        lift_model_ele.append(
            generate_joint(
                'cabin_joint',
                'prismatic',
                'world',
                'platform',
                joint_axis='z'))

        # cabin doors
        for lift_door in self.doors:
            lift_door.generate_cabin_door(
                lift_model_ele, f'CabinDoor_{self.name}_{lift_door.name}')

        # lift cabin plugin
        plugin_ele = SubElement(lift_model_ele, 'plugin')
        plugin_ele.set('name', f'{self.name}_plugin')
        plugin_ele.set('filename', 'liblift.so')

        lift_name_ele = SubElement(plugin_ele, 'lift_name')
        lift_name_ele.text = f'{self.name}'

        for level_name, door_names in self.level_doors.items():
            floor_ele = SubElement(plugin_ele, 'floor')
            floor_ele.set('name', f'{level_name}')
            floor_ele.set('elevation', f'{self.level_elevation[level_name]}')
            for door in self.doors:
                if door.name in door_names:
                    door_pair_ele = SubElement(floor_ele, 'door_pair')
                    door_pair_ele.set(
                        'cabin_door',
                        f'CabinDoor_{self.name}_{door.name}')
                    door_pair_ele.set(
                        'shaft_door',
                        f'ShaftDoor_{self.name}_{level_name}_{door.name}')

        default_floor_ele = SubElement(plugin_ele, 'default_floor')
        default_floor_ele.text = f'{self.reference_floor_name}'
        for param_name, param_value in self.params.items():
            ele = SubElement(plugin_ele, param_name)
            ele.text = f'{param_value}'

        cabin_joint_ele = SubElement(plugin_ele, 'cabin_joint_name')
        cabin_joint_ele.text = 'cabin_joint'

        # pose
        model_pose = SubElement(lift_model_ele, 'pose')
        model_pose.text = f'{self.x} {self.y} 0 0 0 {self.yaw}'
