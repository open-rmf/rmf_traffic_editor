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

    def generate_cabin_door(self, model_ele):
        door_size = [0.03, self.width / 2, 2.5]
        left_door_pose = Element('pose')
        left_door_pose.text = f'{self.y-0.025} {self.width/4} 1.25 0 0 0'

        model_ele.append(
            generate_box_link(
                self.name+'_left_door',
                door_size, left_door_pose,
                bitmask='0x02'))

        model_ele.append(
            generate_joint(
                self.name+'_left_door_joint',
                'prismatic',
                'platform',
                self.name+'_left_door',
                joint_axis='y',
                lower_limit='0',
                upper_limit=f'{self.width / 2}'))

        right_door_pose = Element('pose')
        right_door_pose.text = f'{self.y-0.025} {-self.width/4} 1.25 0 0 0'

        model_ele.append(
            generate_box_link(
                self.name+'_right_door',
                door_size, right_door_pose,
                bitmask='0x02'))

        model_ele.append(
            generate_joint(
                self.name+'_right_door_joint',
                'prismatic',
                'platform',
                self.name+'_right_door',
                joint_axis='y',
                lower_limit=f'{-self.width / 2}',
                upper_limit='0'))

    def generate_shaft_door(self, world_ele, x, y, z, yaw, suffix):
        model_ele = SubElement(world_ele, 'model')
        model_ele.set('name', f'LiftDoor{suffix}')
        door_pose = SubElement(model_ele, 'pose')
        # tranformation
        x_new = x + self.x * np.cos(yaw) - self.y * np.sin(yaw)
        y_new = y + self.x * np.sin(yaw) + self.y * np.cos(yaw)
        yaw_new = yaw + self.motion_axis_orientation + 1.571
        door_pose.text = f'{x_new} {y_new} {z} 0 0 {yaw_new}'

        door_size = [0.03, self.width / 2, 2.5]
        left_door_pose = Element('pose')
        left_door_pose.text = f'0 {self.width / 4} 1.25 0 0 0'

        model_ele.append(
            generate_box_link(
                'left_door',
                door_size, left_door_pose,
                bitmask='0x02'))

        model_ele.append(
            generate_joint(
                f'left_door_joint{suffix}',
                'prismatic',
                'world',
                'left_door',
                joint_axis='y',
                lower_limit='0',
                upper_limit=f'{self.width / 2}'))

        right_door_pose = Element('pose')
        right_door_pose.text = f'0 {-self.width / 4} 1.25 0 0 0'

        model_ele.append(
            generate_box_link(
                'right_door',
                door_size, right_door_pose,
                bitmask='0x02'))

        model_ele.append(
            generate_joint(
                f'right_door_joint{suffix}',
                'prismatic',
                'world',
                'right_door',
                joint_axis='y',
                lower_limit=f'{-self.width / 2}',
                upper_limit='0'))

        ramp_size = [0.06, self.width, 0.05]
        ramp_pose = Element('pose')
        ramp_pose.text = '0 0 -0.025 0 0 0'

        model_ele.append(
            generate_box_link(
                f'LiftDoor{suffix}_ramp',
                ramp_size, ramp_pose,
                bitmask='0x02'))

        model_ele.append(
            generate_joint(
                f'LiftDoor{suffix}_ramp_joint',
                'fixed',
                'world',
                f'LiftDoor{suffix}_ramp'))


class Lift:
    def __init__(self, yaml_node, name, transform):
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
            'f_max_cabin': 25323.0,
            'v_max_door': 0.3,
            'a_max_door': 0.15,
            'a_nom_door': 0.12,
            'dx_min_door': 1e-06,
            'f_max_door': 200.0
        }

        self.level_names = []
        self.door_names = []
        self.level_doors = {}
        if 'level_doors' in yaml_node:
            for level_name, door_names in yaml_node['level_doors'].items():
                self.level_doors[level_name] = door_names
                self.level_names.append(level_name)
                self.door_names.append(door_names)

        self.doors = []
        if 'doors' in yaml_node:
            self.doors = self.parse_lift_doors(yaml_node['doors'])

    def parse_lift_doors(self, yaml_node):
        doors = []
        for lift_door_name, lift_door_yaml in yaml_node.items():
            doors.append(LiftDoor(lift_door_yaml, lift_door_name))
        return doors

    def generate_shaft_doors(self, world_ele):
        for level_name, level_tup in self.level_doors.items():
            for door in self.doors:
                if door.name in level_tup[1]:
                    suffix = f'_{self.name}_{level_name}_{door.name}'
                    elevation = level_tup[0]
                    door.generate_shaft_door(
                        world_ele, self.x, self.y, elevation, self.yaw, suffix)

    def generate_cabin(self, world_ele):
        # materials missing for now
        lift_model_name = f'{self.name}'
        lift_model = SubElement(world_ele, 'model')
        lift_model.set('name', lift_model_name)

        # main cabin link for actuation
        platform_name = 'platform'
        platform = SubElement(lift_model, 'link')
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

        back_wall_name = 'back_wall'
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

        # TODO(Kevin): dynamic front wall dimensions matching door size
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
                bitmask='0x01'))

        # lift cabin actuation joint
        lift_model.append(
            generate_joint(
                'cabin_joint',
                'prismatic',
                'world',
                'platform',
                joint_axis='z'))

        # cabin door links and joints
        # HARDCODED FOR NOW!
        door_size = [0.03, self.cabin_width / 4, 2.5]
        left_door_pose = Element('pose')
        left_door_pose.text = \
            f'{self.cabin_depth/2 - 0.025} {self.cabin_width/8} 1.25 0 0 0'
        lift_model.append(
            generate_box_link(
                'left_door',
                door_size,
                left_door_pose,
                bitmask='0x02'))

        lift_model.append(
            generate_joint(
                'left_door_joint',
                'prismatic',
                'platform',
                'left_door',
                joint_axis='y',
                lower_limit='0',
                upper_limit=f'{self.cabin_width/4}'))

        right_door_pose = Element('pose')
        right_door_pose.text = \
            f'{self.cabin_depth/2 - 0.025} {-self.cabin_width/8} 1.25 0 0 0'
        lift_model.append(
            generate_box_link(
                'right_door',
                door_size,
                right_door_pose,
                bitmask='0x02'))

        lift_model.append(
            generate_joint(
                'right_door_joint',
                'prismatic',
                'platform',
                'right_door',
                joint_axis='y',
                lower_limit=f'{-self.cabin_width/4}',
                upper_limit='0'))

        # TODO: Automatically generate cabin door according to drawing
        # for lift_door in self.doors:
        #     lift_door.generate(lift_cabin_model)

        # lift cabin plugin
        plugin_ele = SubElement(lift_model, 'plugin')
        plugin_ele.set('name', f'{self.name}_plugin')
        plugin_ele.set('filename', 'liblift.so')

        lift_name_ele = SubElement(plugin_ele, 'lift_name')
        lift_name_ele.text = f'{self.name}'

        for level_name, level_tup in self.level_doors.items():
            floor_ele = SubElement(plugin_ele, 'floor')
            floor_ele.set('name', level_name)
            floor_ele.set('elevation', f'{level_tup[0]}')
            for door in self.doors:
                if door.name in level_tup[1]:
                    suf = f'_{self.name}_{level_name}_{door.name}'
                    door_ele = SubElement(floor_ele, 'door')
                    door_ele.set('left_joint_name', f'left_door_joint{suf}')
                    door_ele.set('name', f'LiftDoor{suf}')
                    door_ele.set('right_joint_name', f'right_door_joint{suf}')
                    door_ele.set('type', 'DoubleSlidingDoor')

        default_floor_ele = SubElement(plugin_ele, 'default_floor')
        default_floor_ele.text = f'{self.reference_floor_name}'
        for param_name, param_value in self.params.items():
            ele = SubElement(plugin_ele, param_name)
            ele.text = f'{param_value}'

        cabin_ele = SubElement(plugin_ele, 'cabin')
        cabin_ele.set('name', self.name)

        cabin_joint_ele = SubElement(cabin_ele, 'cabin_joint')
        cabin_joint_ele.set('name', 'cabin_joint')

        cabin_door_ele = SubElement(cabin_ele, 'door')
        cabin_door_ele.set('left_joint_name', f'left_door_joint')
        cabin_door_ele.set('name', f'cabin_door')
        cabin_door_ele.set('right_joint_name', f'right_door_joint')
        cabin_door_ele.set('type', 'DoubleSlidingDoor')

        # pose
        model_pose = SubElement(lift_model, 'pose')
        model_pose.text = f'{self.x} {self.y} 0 0 0 {self.yaw}'
