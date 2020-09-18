import yaml
import sys
import os
import copy

from building_map.building import Building
from building_map.level import Level


class LevelWithHumanLanes (Level):
    def __init__(self, yaml_node, name, graph_idx=9):
        Level.__init__(self, yaml_node, name)

        # default graph_idx for human_lanes is 9
        self.current_graph_idx = graph_idx

        # add human_lanes variable
        self.human_lanes = []
        self.human_goals = {}
        if 'human_lanes' in yaml_node:
            self.human_lanes = \
                self.parse_edge_sequence(yaml_node['human_lanes'])
        else:
            print("Expected human_lanes tag for crowd simulation")
            raise ValueError("No 'human_lanes' found!")

        self.calculate_scale_using_measurements()
        self.transform_all_vertices()
        self.update_human_goals()

    def update_human_goals(self):
        self.human_goals = {}
        if len(self.transformed_vertices) == 0:
            print("Please transform all the vertices" +
                  " before updating the human goals")
        for vertex in self.transformed_vertices:
            if 'human_goal_set_name' not in vertex.params:
                continue
            if vertex.params['human_goal_set_name'].value\
               not in self.human_goals:
                self.human_goals[
                    vertex.params['human_goal_set_name'].value] = []
            self.human_goals[
                vertex.params['human_goal_set_name'].value].append(
                vertex.xy())


class BuildingYamlParse:
    def __init__(self, map_path):
        if not os.path.isfile(map_path):
            raise FileNotFoundError(f'input file {map_path} not found')
        self.building_file = map_path

        with open(self.building_file) as f:
            self.yaml_node = yaml.load(f, yaml.SafeLoader)

        # human_lanes for navmesh
        self.levels_with_human_lanes = {}
        self.levels_name = []
        for level_name, level_yaml in self.yaml_node['levels'].items():
            self.levels_with_human_lanes[level_name] = LevelWithHumanLanes(
                level_yaml, level_name)
            self.levels_name.append(level_name)

        # crowd_sim for configuration
        if 'crowd_sim' not in self.yaml_node:
            print("Expected 'crowd_sim' tag for crowd simulation")
            return

        self.crowd_sim_config = self.yaml_node['crowd_sim']

    def get_human_goals(self):
        result = {}
        for level_name in self.levels_with_human_lanes:
            cur_human_goals =\
                self.levels_with_human_lanes[level_name].human_goals
            for area in cur_human_goals:
                if area not in result:
                    result[area] = []
                result[area].extend(cur_human_goals[area])
        return result
