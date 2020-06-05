import yaml
import numpy as np
from .transform import Transform
from xml.etree.ElementTree import ElementTree, Element, SubElement

from .lift_utils import *

from .doors.double_sliding_door import DoubleSlidingDoor


class LiftDoor:
    def __init__(self, yaml_node, name, lift_dims):
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

        x_diff = abs(abs(self.x) - lift_dims[0] / 2)
        y_diff = abs(abs(self.y) - lift_dims[1] / 2)
        if x_diff <= y_diff:
            if self.x > 0:
                self.side = ('right', self.y-self.width/2, self.y+self.width/2)
                self.shaft_door_pose = (lift_dims[0]/2, self.y)
                self.cabin_door_pose = (lift_dims[0]/2 - 0.075, self.y)
            else:
                self.side = ('left', self.y-self.width/2, self.y+self.width/2)
                self.shaft_door_pose = (-lift_dims[0]/2, self.y)
                self.cabin_door_pose = (-lift_dims[0]/2 + 0.075, self.y)
        else:
            if self.y > 0:
                self.side = ('front', self.x-self.width/2, self.x+self.width/2)
                self.shaft_door_pose = (self.x, lift_dims[1]/2)
                self.cabin_door_pose = (self.x, lift_dims[1]/2 - 0.075)
            else:
                self.side = ('back', self.x-self.width/2, self.x+self.width/2)
                self.shaft_door_pose = (self.x, -lift_dims[1]/2)
                self.cabin_door_pose = (self.x, -lift_dims[1]/2 + 0.075)

    def generate_cabin_door(self, lift_model_ele, name):
        door_model_ele = SubElement(lift_model_ele, 'model')
        door_model_ele.set('name', name)
        door_pose = SubElement(door_model_ele, 'pose')
        (x, y) = self.cabin_door_pose
        door_pose.text = f'{x} {y} 0 0 0 {self.motion_axis_orientation+1.5708}'

        self.generate_door_link_and_joint(door_model_ele, parent='platform')

        self.generate_door_plugin(door_model_ele, name)

    def generate_shaft_door(self, world_ele, x, y, z, yaw, name):
        model_ele = SubElement(world_ele, 'model')
        model_ele.set('name', name)
        door_pose = SubElement(model_ele, 'pose')
        # tranformation
        (door_x, door_y) = self.shaft_door_pose
        x_new = x + door_x * np.cos(yaw) - door_y * np.sin(yaw)
        y_new = y + door_x * np.sin(yaw) + door_y * np.cos(yaw)
        yaw_new = yaw + self.motion_axis_orientation + 1.5708
        door_pose.text = f'{x_new} {y_new} {z} 0 0 {yaw_new}'

        self.generate_door_link_and_joint(model_ele)

        ramp_size = [0.08, self.width, 0.05]
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

        # for wall generation
        self.vertices = \
            {'front': [-self.cabin_width/2, self.cabin_width/2],
             'back': [-self.cabin_width/2, self.cabin_width/2],
             'left': [-self.cabin_depth/2 + 0.05, self.cabin_depth/2 - 0.05],
             'right': [-self.cabin_depth/2 + 0.05, self.cabin_depth/2 - 0.05]}

        for door in self.doors:
            side, left, right = door.side
            self.vertices[side] += [left, right]
            self.vertices[side].sort()

    def parse_lift_doors(self, yaml_node):
        doors = []
        for lift_door_name, lift_door_yaml in yaml_node.items():
            # check if door is aligned to the edges
            # x_diff = abs(float(lift_door_yaml['x']) - self.width/2)
            # y_diff = abs(float(lift_door_yaml['y']) - self.depth/2)
            # yaw_diff = float(lift_door_yaml['motion_axis_orientation'])%1.571
            # assert (x_diff <= 0.05 or y_diff <= 0.5)
            # assert (yaw_diff <= 0.05 or yaw_diff >= 1.52)

            doors.append(LiftDoor(
                lift_door_yaml, lift_door_name, (self.width, self.depth)))
        return doors

    def generate_shaft_doors(self, world_ele):
        for level_name, door_names in self.level_doors.items():
            for door in self.doors:
                if door.name in door_names:
                    name = f'ShaftDoor_{self.name}_{level_name}_{door.name}'
                    elevation = self.level_elevation[level_name]
                    door.generate_shaft_door(
                        world_ele, self.x, self.y, elevation, self.yaw, name)

    def generate_wall(self, side, pair, name, platform):
        dims = [pair[1]-pair[0], 0.05, 2.5]
        mid = (pair[0] + pair[1]) / 2
        if side == 'front':
            x, y, yaw = mid, self.cabin_depth/2 - 0.025, 0
        elif side == 'back':
            x, y, yaw = mid, -self.cabin_depth/2 + 0.025, 0
        elif side == 'left':
            x, y, yaw = -self.cabin_width/2 + 0.025, mid, 1.5708
        elif side == 'right':
            x, y, yaw = self.cabin_width/2 - 0.025, mid, 1.5708
        else:
            return

        pose = Element('pose')
        pose.text = f'{x} {y} 1.25 0 0 {yaw}'
        platform.append(generate_visual(name, pose, dims))
        platform.append(generate_collision(name, pose, dims))

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
        floor_dims = [self.cabin_width, self.cabin_depth, 0.05]
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

        # Wall generation
        for side, vertices in self.vertices.items():
            assert len(vertices) % 2 == 0
            for i in range(0, len(vertices), 2):
                pair = vertices[i: i+2]
                name = f'{side}wall{i//2+1}'
                self.generate_wall(side, pair, name, platform)

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
