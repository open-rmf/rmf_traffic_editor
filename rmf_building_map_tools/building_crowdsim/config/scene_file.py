import xml.etree.ElementTree as ET

from .leaf_element import LeafElement, Element


class SceneFile (Element):
    def __init__(self):
        Element.__init__(self, 'Experiment')
        self.attributes['version'] = 2.0

    def add_spatial_query(self):
        self.spatial_query = SpatialQuery()
        self.sub_elements.append(self.spatial_query)

    def add_common(self):
        self.common = Common()
        self.sub_elements.append(self.common)

    def add_obstacle_set(self, file_name, class_id):
        self.obstacle_set = ObstacleSet()
        self.obstacle_set.attributes['file_name'] = file_name
        self.obstacle_set.attributes['class'] = class_id
        self.sub_elements.append(self.obstacle_set)

    def add_agent_profile(self, profile_name):
        agent_profile = AgentProfile(profile_name)
        self.sub_elements.append(agent_profile)
        return agent_profile

    def add_agent_group(self, profile_name, state_name):
        agent_group = AgentGroup(profile_name, state_name)
        self.sub_elements.append(agent_group)
        return agent_group


class SpatialQuery (LeafElement):
    def __init__(self):
        LeafElement.__init__(self, 'SpatialQuery')
        self.attributes['type'] = 'kd-tree'
        self.attributes['test_visibility'] = 'false'


class Common (LeafElement):
    def __init__(self):
        LeafElement.__init__(self, 'Common')
        self.attributes['time_step'] = 0.1


class ObstacleSet (LeafElement):
    def __init__(self):
        LeafElement.__init__(self, 'ObstacleSet')
        self.attributes['type'] = 'nav_mesh'
        self.attributes['file_name'] = ''
        self.attributes['class'] = -1

    def is_valid(self):
        if self.attributes['class'] < 0:
            return False
        if not self.attributes['file_name']:
            return False
        return True

    def load_from_yaml(self, yaml_node):
        if 'class' not in yaml_node or\
           'file_name' not in yaml_node:
            raise ValueError("Invalid ObstacleSet Yaml!")
        self.attributes['class'] = int(yaml_node['class'])
        self.attributes['file_name'] = yaml_node['file_name']


class ProfileCommon (LeafElement):
    def __init__(self):
        LeafElement.__init__(self, 'Common')
        self.attributes['class'] = 0
        self.attributes['max_accel'] = 5
        self.attributes['max_angle_vel'] = 360
        self.attributes['max_neighbors'] = 10
        self.attributes['max_speed'] = 2
        self.attributes['neighbor_dist'] = 5
        self.attributes['pref_speed'] = 0
        self.attributes['r'] = 0.2
        # use default obstacleSet, should not be changed
        self.attributes['obstacleSet'] = 1

    def load_from_yaml(self, yaml_node):
        if yaml_node['class']:
            self.attributes['class'] = int(yaml_node['class'])
        if yaml_node['max_accel']:
            self.attributes['max_accel'] = float(yaml_node['max_accel'])
        if yaml_node['max_angle_vel']:
            self.attributes['max_angle_vel'] =\
                float(yaml_node['max_angle_vel'])
        if yaml_node['max_neighbors']:
            self.attributes['max_neighbors'] = int(yaml_node['max_neighbors'])
        if yaml_node['max_speed']:
            self.attributes['max_speed'] = float(yaml_node['max_speed'])
        if yaml_node['neighbor_dist']:
            self.attributes['neighbor_dist'] =\
                float(yaml_node['neighbor_dist'])
        if yaml_node['pref_speed']:
            self.attributes['pref_speed'] = float(yaml_node['pref_speed'])
        if yaml_node['r']:
            self.attributes['r'] = float(yaml_node['r'])
        if yaml_node['obstacle_set']:
            self.attributes['obstacleSet'] = int(yaml_node['obstacle_set'])


class ProfileORCA (LeafElement):
    def __init__(self):
        LeafElement.__init__(self, 'ORCA')
        self.attributes['tau'] = 3.0
        self.attributes['tauObst'] = 0.15

    def load_from_yaml(self, yaml_node):
        if yaml_node['ORCA_tau']:
            self.attributes['tau'] = float(yaml_node['ORCA_tau'])
        if yaml_node['ORCA_tauObst']:
            self.attributes['tauObst'] = float(yaml_node['ORCA_tauObst'])


class ProfileSelector (LeafElement):
    def __init__(self, name):
        LeafElement.__init__(self, 'ProfileSelector')
        self.attributes['type'] = 'const'
        self.attributes['name'] = name

    def is_valid(self):
        if not self.attributes['name']:
            return False
        return True


class StateSelector (LeafElement):
    def __init__(self, name):
        LeafElement.__init__(self, 'StateSelector')
        self.attributes['type'] = 'const'
        self.attributes['name'] = name

    def is_valid(self):
        if not self.attributes['name']:
            return False
        return True


class Agent (LeafElement):
    def __init__(self, x, y):
        LeafElement.__init__(self, 'Agent')
        self.attributes['p_x'] = x
        self.attributes['p_y'] = y


class AgentGenerator (Element):
    def __init__(self):
        Element.__init__(self, 'Generator')
        self.attributes['type'] = 'explicit'

    def is_valid(self):
        if len(self.sub_elements) == 0:
            return False
        return Element.is_valid(self)

    def add_agent(self, x, y):
        self.sub_elements.append(Agent(x, y))


class AgentProfile (Element):
    def __init__(self, name=''):
        Element.__init__(self, 'AgentProfile')
        self.attributes['name'] = name
        self.profile_common = ProfileCommon()
        self.profile_orca = ProfileORCA()
        self.sub_elements.append(self.profile_common)
        self.sub_elements.append(self.profile_orca)

    def load_from_yaml(self, yaml_node):
        required_items =\
            ['name', 'class', 'max_accel', 'max_angle_vel', 'max_neighbors',
             'max_speed', 'neighbor_dist', 'obstacle_set', 'pref_speed', 'r',
             'ORCA_tau', 'ORCA_tauObst']
        for key in required_items:
            if key not in yaml_node:
                raise ValueError("Invalid Agent Profile Yaml!")

        self.attributes['name'] = yaml_node['name']
        self.profile_common.load_from_yaml(yaml_node)
        self.profile_orca.load_from_yaml(yaml_node)


class AgentGroup (Element):
    def __init__(self, profile_type='', state_name=''):
        Element.__init__(self, 'AgentGroup')
        self.profile_selector = ProfileSelector(profile_type)
        self.state_selector = StateSelector(state_name)
        self.generator = AgentGenerator()
        self.sub_elements.append(self.profile_selector)
        self.sub_elements.append(self.state_selector)
        self.sub_elements.append(self.generator)

    def add_agent(self, x, y):
        self.generator.add_agent(x, y)

    def load_from_yaml(self, yaml_node):
        if 'profile_selector' not in yaml_node or\
           'state_selector' not in yaml_node:
            raise ValueError("Invalid AgentGroup Yaml!")

        self.profile_selector.attributes['name'] =\
            yaml_node['profile_selector']
        self.state_selector.attributes['name'] =\
            yaml_node['state_selector']

        if 'agents_number' not in yaml_node:
            raise ValueError("Invalid agents_number provided in AgentGroup!")
        number = int(yaml_node['agents_number'])
        px = float(yaml_node['x'])
        py = float(yaml_node['y'])
        for _ in range(number):
            self.generator.add_agent(px, py)
