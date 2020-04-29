

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


class Lift:
    def __init__(self, yaml_node, name):
        self.name = name
        print(f'parsing lift {name}')

        self.reference_floor_name = yaml_node['reference_floor_name']
        self.depth = float(yaml_node['depth'])
        self.width = float(yaml_node['width'])
        self.x = float(yaml_node['x'])
        self.y = float(yaml_node['y'])
        self.yaw = float(yaml_node['yaw'])

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
