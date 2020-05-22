import yaml
import numpy as np
from .transform import Transform
from xml.etree.ElementTree import ElementTree, Element, SubElement

from .sdf_utils import *

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

    def generate(self, model_ele):
        door_size = [0.03, self.width / 4, 2.5]
        left_door_pose = Element('pose')
        left_door_pose.text = '{} {} {} 0 0 0'.format(self.y - 0.025, self.width / 8, 1.25)
        model_ele.append(generate_box_link(self.name+'_left_door', door_size, left_door_pose, bitmask='0x02'))
        
        model_ele.append(generate_joint(self.name+'_left_door_joint', 'prismatic', 'platform', self.name+'_left_door',
                       joint_axis='y', lower_limit='0', upper_limit='{}'.format(self.width/4)))

        right_door_pose = Element('pose')
        right_door_pose.text = '{} {} {} 0 0 0'.format(self.y - 0.025, -self.width / 8, 1.25)
        model_ele.append(generate_box_link(self.name+'_right_door', door_size, right_door_pose, bitmask='0x02'))
        
        model_ele.append(generate_joint(self.name+'_right_door_joint', 'prismatic', 'platform', self.name+'_right_door',
                       joint_axis='y', lower_limit='{}'.format(-self.width/4), upper_limit='0'))


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

        # default params
        self.cabin_mass = 800
        self.cabin_max_load = 1500
        self.cabin_a_max = 1.2    # max acceleration
        self.cabin_f_max = (self.cabin_mass + self.cabin_max_load) * (self.cabin_a_max + 9.81)    # max force

        self.level_names = []
        self.door_names = []
        if 'level_doors' in yaml_node:
            for level_name, door_name in yaml_node['level_doors'].items():
                self.level_names.append(level_name)
                self.door_names.append(door_name)

        self.doors = []
        if 'doors' in yaml_node:
            self.doors = self.parse_lift_doors(yaml_node['doors'])

    def parse_lift_doors(self, yaml_node):
        doors = []
        for lift_door_name, lift_door_yaml in yaml_node.items():
            doors.append(LiftDoor(lift_door_yaml, lift_door_name))
        return doors

    def generate_holes(self, world_ele):
        pass

    def generate_shaft_doors(self, world_ele, options):
        # get the x, y, and yaw of the doors base on the lift position and lift_door positions
        # get the z position of the doors from the
        pass

    def generate_cabin(self, world_ele):
        # materials missing for now
        lift_cabin_model_name = 'LiftCabin_{}'.format(self.name)
        lift_cabin_model = SubElement(world_ele, 'model')
        lift_cabin_model.set('name', lift_cabin_model_name)

        # main cabin link for actuation
        platform_name = 'platform'
        platform = SubElement(lift_cabin_model, 'link')
        platform.set('name', platform_name)
        inertial = SubElement(platform, 'inertial')
        mass = SubElement(inertial, 'mass')
        mass.text = '{}'.format(self.cabin_mass)

        # visuals and collisions for floor and walls of cabin
        floor_dims = [self.depth, self.width, 0.05]
        floor_name = 'floor'
        floor_pose = Element('pose')
        floor_pose.text = '{} {} {} 0 0 0'.format(0, 0, -0.025)
        platform.append(generate_visual(floor_name, floor_pose, floor_dims))
        platform.append(generate_collision(floor_name, floor_pose, floor_dims))

        back_wall_name = 'back_wall'
        back_wall_dims = [0.05, self.width, 2.5]
        back_wall_pose = Element('pose')
        back_wall_pose.text = '{} {} {} 0 0 0'.format(-self.depth / 2 + 0.025, 0, 1.25)
        platform.append(generate_visual(back_wall_name, back_wall_pose, back_wall_dims))
        platform.append(generate_collision(back_wall_name, back_wall_pose, back_wall_dims))

        side_wall_dims = [self.depth - 0.1, 0.05, 2.5]
        left_wall_name = 'left_wall'
        left_wall_pose = Element('pose')
        left_wall_pose.text = '{} {} {} 0 0 0'.format(0, self.width / 2 - 0.025, 1.25)
        platform.append(generate_visual(left_wall_name, left_wall_pose, side_wall_dims))
        platform.append(generate_collision(left_wall_name, left_wall_pose, side_wall_dims))

        right_wall_name = 'right_wall'
        right_wall_pose = Element('pose')
        right_wall_pose.text = '{} {} {} 0 0 0'.format(0, -self.width / 2 + 0.025, 1.25)
        platform.append(generate_visual(right_wall_name, right_wall_pose, side_wall_dims))
        platform.append(generate_collision(right_wall_name, right_wall_pose, side_wall_dims))

        # !!! change front wall dimensions to match lift door dimensions
        front_wall_dims = [0.05, self.width / 4, 2.5]
        front_left_wall_name = 'front_left_wall'
        front_left_wall_pose = Element('pose')
        front_left_wall_pose.text = '{} {} {} 0 0 0'.format(self.depth / 2 - 0.025, self.width * 3 / 8, 1.25)
        platform.append(generate_visual(front_left_wall_name, front_left_wall_pose, front_wall_dims))
        platform.append(generate_collision(front_left_wall_name, front_left_wall_pose, front_wall_dims, bitmask='0x01'))

        front_right_wall_name = 'front_right_wall'
        front_right_wall_pose = Element('pose')
        front_right_wall_pose.text = '{} {} {} 0 0 0'.format(self.depth / 2 - 0.025, -self.width * 3 / 8, 1.25)
        platform.append(generate_visual(front_right_wall_name, front_right_wall_pose, front_wall_dims))
        platform.append(generate_collision(front_right_wall_name, front_right_wall_pose, front_wall_dims, bitmask='0x01'))

        # lift cabin actuation joint
        lift_cabin_model.append(generate_joint('cabin_joint', 'prismatic', 'world', 'platform'))

        # lift door links and joints
        
        # for lift_door in self.doors:
        #     lift_door.generate(lift_cabin_model)

        door_size = [0.03, self.width / 4, 2.5]
        left_door_pose = Element('pose')
        left_door_pose.text = '{} {} {} 0 0 0'.format(self.depth / 2 - 0.025, self.width / 8, 1.25)
        lift_cabin_model.append(generate_box_link('left_door', door_size, left_door_pose, bitmask='0x02'))
        
        lift_cabin_model.append(generate_joint('left_door_joint', 'prismatic', 'platform', 'left_door',
                       joint_axis='y', lower_limit='0', upper_limit='{}'.format(self.width/4)))

        right_door_pose = Element('pose')
        right_door_pose.text = '{} {} {} 0 0 0'.format(self.depth / 2 - 0.025, -self.width / 8, 1.25)
        lift_cabin_model.append(generate_box_link('right_door', door_size, right_door_pose, bitmask='0x02'))
        
        lift_cabin_model.append(generate_joint('right_door_joint', 'prismatic', 'platform', 'right_door',
                       joint_axis='y', lower_limit='{}'.format(-self.width/4), upper_limit='0'))

        # TODO: Plugin

        model_pose = SubElement(lift_cabin_model, 'pose')
        model_pose.text = '{} {} 0 0 0 {}'.format(self.x, self.y, self.yaw)

