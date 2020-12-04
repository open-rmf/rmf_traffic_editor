import yaml
import numpy as np
from xml.etree.ElementTree import ElementTree, Element, SubElement

from .transform import Transform
from .utils import lift_material, visual, collision, box_link, joint


class LiftDoor:
    def __init__(self, yaml_node, name, lift_size, gap, plugin=True):
        self.name = name
        self.door_type = yaml_node['door_type']
        # x & y coordinates are with respect to the centre of the cabin
        self.x = float(yaml_node['x'])
        self.y = float(yaml_node['y'])
        self.motion_axis_orientation = float(
            yaml_node['motion_axis_orientation'])
        self.width = float(yaml_node['width'])
        self.height = lift_size[2]
        self.thickness = 0.03
        self.gap = gap    # gap between cabin_door and shaft_door
        self.plugin = plugin
        self.params = {'v_max_door': 0.3,
                       'a_max_door': 0.2,
                       'a_nom_door': 0.1,
                       'dx_min_door': 1e-4,
                       'f_max_door': 35.0}

        x_diff = abs(abs(self.x) - lift_size[0] / 2)
        y_diff = abs(abs(self.y) - lift_size[1] / 2)
        offset = 0.025    # cabin door position offset
        if x_diff <= y_diff:
            if self.x > 0:
                self.side = ('right', self.y-self.width/2, self.y+self.width/2)
                self.shaft_door_pose = (lift_size[0]/2 + self.gap, self.y)
                self.cabin_door_pose = (lift_size[0]/2 - offset, self.y)
            else:
                self.side = ('left', self.y-self.width/2, self.y+self.width/2)
                self.shaft_door_pose = (-lift_size[0]/2 - self.gap, self.y)
                self.cabin_door_pose = (-lift_size[0]/2 + offset, self.y)
        else:
            if self.y > 0:
                self.side = ('front', self.x-self.width/2, self.x+self.width/2)
                self.shaft_door_pose = (self.x, lift_size[1]/2 + self.gap)
                self.cabin_door_pose = (self.x, lift_size[1]/2 - offset)
            else:
                self.side = ('back', self.x-self.width/2, self.x+self.width/2)
                self.shaft_door_pose = (self.x, -lift_size[1]/2 - self.gap)
                self.cabin_door_pose = (self.x, -lift_size[1]/2 + offset)

    def generate_cabin_door(self, lift_model_ele, name):
        door_model_ele = SubElement(lift_model_ele, 'model')
        door_model_ele.set('name', name)
        door_pose = SubElement(door_model_ele, 'pose')
        (x, y) = self.cabin_door_pose
        door_pose.text = \
            f'{x} {y} 0 0 0 {self.motion_axis_orientation}'

        self.generate_door_link_and_joint(door_model_ele, parent='platform')

        if self.plugin:
            self.generate_door_plugin(door_model_ele, name)

    def generate_shaft_door(self, world_ele, x, y, z, yaw, name):
        model_ele = SubElement(world_ele, 'model')
        model_ele.set('name', name)
        door_pose = SubElement(model_ele, 'pose')
        # tranformation
        (door_x, door_y) = self.shaft_door_pose
        x_new = x + door_x * np.cos(yaw) - door_y * np.sin(yaw)
        y_new = y + door_x * np.sin(yaw) + door_y * np.cos(yaw)
        yaw_new = yaw + self.motion_axis_orientation
        door_pose.text = f'{x_new} {y_new} {z} 0 0 {yaw_new}'

        self.generate_door_link_and_joint(model_ele)

        floor_thickness = 0.05
        ramp_depth = self.gap * 2
        ramp_size = [self.width, ramp_depth, floor_thickness]
        ramp_pose = Element('pose')
        ramp_pose.text = f'0 0 {-floor_thickness / 2} 0 0 0'
        model_ele.append(box_link('ramp',
                                  ramp_size,
                                  ramp_pose,
                                  material=lift_material(),
                                  bitmask='0x02'))
        model_ele.append(joint('ramp_joint', 'fixed', 'world', 'ramp'))

        if self.plugin:
            self.generate_door_plugin(model_ele, name)

    def generate_door_link_and_joint(self, model_ele, parent='world'):
        door_size = [self.width / 2, self.thickness, self.height]
        right_door_pose = Element('pose')
        right_door_pose.text = f'{self.width / 4} 0 {self.height / 2} 0 0 0'

        model_ele.append(box_link('right_door',
                                  door_size,
                                  right_door_pose,
                                  material=lift_material(),
                                  bitmask='0x02'))

        model_ele.append(joint('right_joint',
                               'prismatic',
                               parent,
                               'right_door',
                               joint_axis='x',
                               lower_limit=0,
                               upper_limit=self.width / 2))

        left_door_pose = Element('pose')
        left_door_pose.text = f'{-self.width / 4} 0 {self.height / 2} 0 0 0'

        model_ele.append(box_link('left_door',
                                  door_size,
                                  left_door_pose,
                                  material=lift_material(),
                                  bitmask='0x02'))

        model_ele.append(joint('left_joint',
                               'prismatic',
                               parent,
                               'left_door',
                               joint_axis='x',
                               lower_limit=-self.width / 2,
                               upper_limit=0))

    def generate_door_plugin(self, model_ele, name):
        plugin_ele = SubElement(model_ele, 'plugin')
        plugin_ele.set('name', 'door')
        plugin_ele.set('filename', 'libdoor.so')
        for param_name, param_value in self.params.items():
            ele = SubElement(plugin_ele, param_name)
            ele.text = f'{param_value}'
        door_ele = SubElement(plugin_ele, 'door')
        door_ele.set('left_joint_name', 'left_joint')
        door_ele.set('name', f'{name}')
        door_ele.set('right_joint_name', 'right_joint')
        door_ele.set('type', 'DoubleSlidingDoor')

    # TODO: remove this function once nesting model is supported in ignition.
    def generate_cabin_door_ign(self, lift_model_ele, name):
        # This is for cabin door generation for ignition gazebo as it doesn't
        # support nested models yet. Once ignition gazebo supports nested
        # models, this should be removed.
        (x, y) = self.cabin_door_pose
        yaw = self.motion_axis_orientation
        right_x = x + np.cos(yaw) * self.width/4
        left_x = x - np.cos(yaw) * self.width/4
        right_y = y + np.sin(yaw) * self.width/4
        left_y = y - np.sin(yaw) * self.width/4

        door_size = [self.width / 2, self.thickness, self.height]
        right_door_pose = Element('pose')
        right_door_pose.text = \
            f'{right_x} {right_y} {self.height / 2} 0 0 {yaw}'

        lift_model_ele.append(box_link(f'{name}_right_door',
                                       door_size,
                                       right_door_pose,
                                       material=lift_material(),
                                       bitmask='0x02'))

        lift_model_ele.append(joint(f'{name}_right_joint',
                                    'prismatic',
                                    'platform',
                                    f'{name}_right_door',
                                    joint_axis='x',
                                    lower_limit=0,
                                    upper_limit=self.width / 2))

        left_door_pose = Element('pose')
        left_door_pose.text = f'{left_x} {left_y} {self.height / 2} 0 0 {yaw}'

        lift_model_ele.append(box_link(f'{name}_left_door',
                                       door_size,
                                       left_door_pose,
                                       material=lift_material(),
                                       bitmask='0x02'))

        lift_model_ele.append(joint(f'{name}_left_joint',
                                    'prismatic',
                                    'platform',
                                    f'{name}_left_door',
                                    joint_axis='x',
                                    lower_limit=-self.width / 2,
                                    upper_limit=0))

        if self.plugin:
            plugin_ele = SubElement(lift_model_ele, 'plugin')
            plugin_ele.set('name', 'door')
            plugin_ele.set('filename', 'libdoor.so')
            for param_name, param_value in self.params.items():
                ele = SubElement(plugin_ele, param_name)
                ele.text = f'{param_value}'
            door_ele = SubElement(plugin_ele, 'door')
            door_ele.set('left_joint_name', f'{name}_left_joint')
            door_ele.set('name', f'{name}')
            door_ele.set('right_joint_name', f'{name}_right_joint')
            door_ele.set('type', 'DoubleSlidingDoor')


class Lift:
    def __init__(self, yaml_node, name, transform, levels):
        self.name = name
        print(f'parsing lift {name}')

        self.depth = float(yaml_node['depth'])
        self.width = float(yaml_node['width'])
        self.yaw = float(yaml_node['yaw'])

        if 'initial_floor_name' in yaml_node:
            self.initial_floor_name = str(yaml_node['initial_floor_name'])
        else:
            self.initial_floor_name = ''

        if 'highest_floor' in yaml_node:
            self.highest_floor = str(yaml_node['highest_floor'])
            if self.highest_floor:
                self.highest_elevation = levels[self.highest_floor].elevation
            else:
                self.highest_elevation = float('inf')
        else:
            self.highest_elevation = float('inf')

        if 'lowest_floor' in yaml_node:
            self.lowest_floor = str(yaml_node['lowest_floor'])
            if self.lowest_floor:
                self.lowest_elevation = levels[self.lowest_floor].elevation
            else:
                self.lowest_elevation = -float('inf')
        else:
            self.lowest_elevation = -float('inf')

        self.plugins = True
        if 'plugins' in yaml_node:
            self.plugins = bool(yaml_node['plugins'])

        raw_pos = (float(yaml_node['x']), -float(yaml_node['y']))
        self.x, self.y = transform.transform_point(raw_pos)
        self.cabin_height = 2.5
        self.wall_thickness = 0.05
        self.floor_thickness = 0.05
        self.gap = 0.05    # gap between lift shaft and lift cabin on each side
        self.shaft_depth = self.depth + 2 * self.gap
        self.shaft_width = self.width + 2 * self.gap

        # default params
        self.cabin_mass = 1200
        self.params = {
            'v_max_cabin': 2.0,
            'a_max_cabin': 1.2,
            'a_nom_cabin': 1.0,
            'dx_min_cabin': 0.001,
            'f_max_cabin': 25323.0}

        self.level_elevation = {}
        self.level_doors = {}
        self.level_names = []
        if 'level_doors' in yaml_node:
            for level_name, door_names in yaml_node['level_doors'].items():
                self.level_doors[level_name] = door_names
                self.level_elevation[level_name] = levels[level_name].elevation
                self.level_names.append(level_name)

        if (not self.initial_floor_name) and self.level_names:
            self.initial_floor_name = self.level_names[0]

        self.doors = []
        if 'doors' in yaml_node:
            self.doors = self.parse_lift_doors(yaml_node['doors'])

        # for wall generation
        # self.end_points stores 1-dimensional positions of endpoints of walls
        # on each side of the cabin in sorted arrays
        self.end_points = {'front': [-self.width/2, self.width/2],
                           'back': [-self.width/2, self.width/2],
                           'left': [-self.depth/2 + self.wall_thickness,
                                    self.depth/2 - self.wall_thickness],
                           'right': [-self.depth/2 + self.wall_thickness,
                                     self.depth/2 - self.wall_thickness]}

        for door in self.doors:
            side, left, right = door.side
            self.end_points[side] += [left, right]
            self.end_points[side].sort()

    def parse_lift_doors(self, yaml_node):
        doors = []
        for lift_door_name, lift_door_yaml in yaml_node.items():
            doors.append(LiftDoor(lift_door_yaml,
                                  lift_door_name,
                                  (self.width, self.depth, self.cabin_height),
                                  self.gap,
                                  self.plugins))
        return doors

    def get_lift_vertices(self):
        # parse lift shaft cavity corner vertices
        vertices = []
        d = self.depth / 2 + self.gap
        w = self.width / 2 + self.gap
        vertices.append((self.x - d * np.sin(self.yaw) - w * np.cos(self.yaw),
                         self.y + d * np.cos(self.yaw) - w * np.sin(self.yaw)))
        vertices.append((self.x + d * np.sin(self.yaw) - w * np.cos(self.yaw),
                         self.y - d * np.cos(self.yaw) - w * np.sin(self.yaw)))
        vertices.append((self.x + d * np.sin(self.yaw) + w * np.cos(self.yaw),
                         self.y - d * np.cos(self.yaw) + w * np.sin(self.yaw)))
        vertices.append((self.x - d * np.sin(self.yaw) + w * np.cos(self.yaw),
                         self.y + d * np.cos(self.yaw) + w * np.sin(self.yaw)))
        return vertices

    def generate_shaft_doors(self, world_ele):
        for level_name, door_names in self.level_doors.items():
            for door in self.doors:
                if door.name in door_names:
                    name = f'ShaftDoor_{self.name}_{level_name}_{door.name}'
                    elevation = self.level_elevation[level_name]
                    door.generate_shaft_door(
                        world_ele, self.x, self.y, elevation, self.yaw, name)

    def generate_wall(self, side, pair, name, platform):
        dims = [pair[1]-pair[0], self.wall_thickness, self.cabin_height]
        mid = (pair[0] + pair[1]) / 2
        if side == 'front':
            x, y, yaw = mid, self.depth/2 - self.wall_thickness / 2, 0
        elif side == 'back':
            x, y, yaw = mid, -self.depth/2 + self.wall_thickness / 2, 0
        elif side == 'left':
            x, y, yaw = -self.width/2 + self.wall_thickness / 2, mid, np.pi/2
        elif side == 'right':
            x, y, yaw = self.width/2 - self.wall_thickness / 2, mid, np.pi/2
        else:
            return

        pose = Element('pose')
        pose.text = f'{x} {y} {self.cabin_height / 2} 0 0 {yaw}'
        platform.append(visual(name, pose, dims, lift_material()))
        platform.append(collision(name, pose, dims, '0x01'))

    def generate_cabin(self, world_ele, options):
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
        inertial = SubElement(inertial, 'inertia')
        SubElement(inertial, 'ixx').text = \
            str(self.cabin_mass/12.0*(self.width**2 + self.floor_thickness**2))
        SubElement(inertial, 'iyy').text = \
            str(self.cabin_mass/12.0*(self.depth**2 + self.floor_thickness**2))
        SubElement(inertial, 'izz').text = \
            str(self.cabin_mass/12.0*(self.width**2 + self.depth**2))

        # visuals and collisions for floor and walls of cabin
        floor_dims = [self.width, self.depth, self.floor_thickness]
        floor_name = 'floor'
        floor_pose = Element('pose')
        floor_pose.text = f'0 0 {-self.floor_thickness / 2} 0 0 0'
        platform.append(visual(floor_name,
                               floor_pose,
                               floor_dims,
                               lift_material()))

        platform.append(collision(floor_name, floor_pose, floor_dims, '0x01'))

        # Wall generation
        # get each pair of end_points on each side, generate a section of wall
        # between the pair of points
        for side, end_points in self.end_points.items():
            assert len(end_points) % 2 == 0
            for i in range(0, len(end_points), 2):
                pair = end_points[i: i+2]
                name = f'{side}wall{i//2+1}'
                self.generate_wall(side, pair, name, platform)

        # lift cabin actuation joint
        lift_model_ele.append(joint('cabin_joint',
                                    'prismatic',
                                    'world',
                                    'platform',
                                    joint_axis='z'))

        # cabin doors
        # TODO: remove the if statement here once nesting model is supported
        # in ignition.
        if 'ignition' in options:
            for lift_door in self.doors:
                lift_door.generate_cabin_door_ign(
                    lift_model_ele, f'CabinDoor_{self.name}_{lift_door.name}')
        else:
            for lift_door in self.doors:
                lift_door.generate_cabin_door(
                    lift_model_ele, f'CabinDoor_{self.name}_{lift_door.name}')

        # lift cabin plugin
        if self.plugins:
            plugin_ele = SubElement(lift_model_ele, 'plugin')
            plugin_ele.set('name', 'lift')
            plugin_ele.set('filename', 'liblift.so')

            lift_name_ele = SubElement(plugin_ele, 'lift_name')
            lift_name_ele.text = f'{self.name}'

            for level_name, door_names in self.level_doors.items():
                floor_ele = SubElement(plugin_ele, 'floor')
                floor_ele.set('name', f'{level_name}')
                floor_ele.set(
                    'elevation', f'{self.level_elevation[level_name]}')
                for door in self.doors:
                    if door.name in door_names:
                        door_pair_ele = SubElement(floor_ele, 'door_pair')
                        door_pair_ele.set(
                            'cabin_door',
                            f'CabinDoor_{self.name}_{door.name}')
                        door_pair_ele.set(
                            'shaft_door',
                            f'ShaftDoor_{self.name}_{level_name}_{door.name}')

            initial_floor_ele = SubElement(plugin_ele, 'initial_floor')
            initial_floor_ele.text = f'{self.initial_floor_name}'
            for param_name, param_value in self.params.items():
                ele = SubElement(plugin_ele, param_name)
                ele.text = f'{param_value}'

            cabin_joint_ele = SubElement(plugin_ele, 'cabin_joint_name')
            cabin_joint_ele.text = 'cabin_joint'
        else:
            static_lift_ele = SubElement(lift_model_ele, 'static')
            static_lift_ele.text = 'true'

        # pose
        model_pose = SubElement(lift_model_ele, 'pose')
        model_pose.text = f'{self.x} {self.y} 0 0 0 {self.yaw}'
