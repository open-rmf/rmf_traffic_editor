import sys
import os
import yaml

import xml.etree.ElementTree as ET

from .behavior_file import\
    BehaviorFile, BehaviorState, StateTransition, GoalSet
from .scene_file import\
    SceneFile, ObstacleSet, AgentProfile, AgentGroup
from .plugin_file import\
    Plugin
from .util import write_xml_file, write_xml_to_complete_file_path

from building_crowdsim.building_yaml_parse import\
    BuildingYamlParse, LevelWithHumanLanes
from building_map.level import Level


class ConfigFileGenerator:
    def __init__(self, building_yaml_parse):
        assert(isinstance(building_yaml_parse, BuildingYamlParse))
        self.levels_yaml = building_yaml_parse.yaml_node['levels']
        self.level = {}
        for level_name, level_yaml in self.levels_yaml.items():
            cur_level = self.level[level_name] = Level(level_yaml, level_name)
            cur_level.enable_crowdsim = 0
            if level_name in building_yaml_parse.crowd_sim_config:
                cur_level.calculate_scale_using_measurements()
                cur_level.crowd_sim_yaml =\
                    building_yaml_parse.crowd_sim_config[level_name]
                cur_level.crowd_sim_human_yaml =\
                    building_yaml_parse.crowd_sim_human[level_name]
                if 'enable' not in cur_level.crowd_sim_yaml:
                    raise ValueError(
                        "Missing 'enable' tag for crowdsim configuration.")
                cur_level.enable_crowdsim = int(
                    cur_level.crowd_sim_yaml['enable']) == 1
                cur_level.human_goals = building_yaml_parse.get_human_goals()
                cur_level.behavior_file = BehaviorFile()
                cur_level.scene_file = SceneFile()
        self.plugin_file = Plugin()

    def generate_behavior_file(self, level_name, output_dir=""):
        # must follow the sequence:
        # 'states', 'transitions', 'goal_sets'
        cur_level = self.level[level_name]
        if 'states' in cur_level.crowd_sim_yaml:
            for state in cur_level.crowd_sim_yaml['states']:
                cur_state = BehaviorState()
                cur_state.load_from_yaml(state)
                cur_level.behavior_file.sub_elements.append(cur_state)
        if 'transitions' in cur_level.crowd_sim_yaml:
            for transition in cur_level.crowd_sim_yaml['transitions']:
                cur_transition = StateTransition()
                cur_transition.load_from_yaml(transition)
                cur_level.behavior_file.sub_elements.append(cur_transition)
        if 'goal_sets' in cur_level.crowd_sim_yaml:
            for goal_set in cur_level.crowd_sim_yaml['goal_sets']:
                cur_goal_set = GoalSet()
                cur_goal_set.load_from_yaml(goal_set, cur_level.human_goals)
                cur_level.behavior_file.sub_elements.append(cur_goal_set)

        write_xml_file(
            cur_level.behavior_file.output_xml_element(),
            output_dir=output_dir,
            file_name='behavior_file.xml')

    def transform_point_modify_yaml(self, level_name, yaml_node):
        x = yaml_node['x']
        y = yaml_node['y']
        yaml_node['x'], yaml_node['y'] = \
            self.level[level_name].transform.transform_point((x, -y))

    def generate_scene_file(self, level_name, output_dir):
        # add default configuration
        cur_level = self.level[level_name]
        cur_level.scene_file.add_spatial_query()
        cur_level.scene_file.add_common()

        # must follow the sequence:
        # 'obstacle_set','agent_profiles','agent_groups'
        if 'obstacle_set' in cur_level.crowd_sim_yaml:
            obstacle_set = ObstacleSet()
            obstacle_set.load_from_yaml(
                cur_level.crowd_sim_yaml['obstacle_set'])
            cur_level.scene_file.sub_elements.append(obstacle_set)
        if 'agent_profiles' in cur_level.crowd_sim_yaml:
            for item in cur_level.crowd_sim_yaml['agent_profiles']:
                cur_profile = AgentProfile()
                cur_profile.load_from_yaml(item)
                cur_level.scene_file.sub_elements.append(cur_profile)
        if 'external_agent_groups' in cur_level.crowd_sim_yaml:
            for item in cur_level.crowd_sim_yaml['external_agent_groups']:
                cur_group = AgentGroup()
                cur_group.load_from_yaml(item)
                cur_level.scene_file.sub_elements.append(cur_group)

        agent_groups = {}
        for item in cur_level.crowd_sim_human_yaml:
            group_id = item['agent_group_id']
            if group_id in agent_groups:
                self.transform_point_modify_yaml(level_name, item)
                agent_groups[group_id].load_an_agent_from_yaml(item)
            else:
                cur_group = AgentGroup()
                self.transform_point_modify_yaml(level_name, item)
                cur_group.load_an_agent_from_yaml(item)
                agent_groups[group_id] = cur_group

        for group_id, agent_group in agent_groups.items():
            cur_level.scene_file.sub_elements.append(agent_group)

        write_xml_file(
            cur_level.scene_file.output_xml_element(),
            output_dir=output_dir,
            file_name='scene_file.xml')

    def generate_plugin_file(self):
        self.plugin_file.load_from_yaml(self.level)

    def insert_plugin_into_world_file(self, world_file_to_be_inserted):
        if not world_file_to_be_inserted:
            print("No world_file provided")
            return

        self.generate_plugin_file()

        tmp = ET.parse(world_file_to_be_inserted)
        file_root = tmp.getroot()
        world_root = None
        for root_child in file_root:
            if root_child.tag != "world":
                continue
            world_root = root_child
        if not world_root:
            raise ValueError(
                "Invalid world file! please check your world file: ",
                world_file_to_be_inserted)

        plugin_already_inserted = []
        for world_child in world_root:
            # remove the possible crowd_simulation plugin previously inserted
            if world_child.tag == "plugin" and\
               world_child.attrib['name'] == "crowd_simulation":
                plugin_already_inserted.append(world_child)
        for world_child in plugin_already_inserted:
            world_root.remove(world_child)

        world_root.append(self.plugin_file.output_xml_element())
        write_xml_to_complete_file_path(
            file_root,
            file_name=world_file_to_be_inserted)
        print(
            "Insert <plugin> tag into ",
            world_file_to_be_inserted)


def configfile_main(map_file, output_dir, world_file_to_be_inserted):
    if not os.path.exists(map_file):
        raise ValueError('Map path not exist!: ' + map_file)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print("Create output dir for:", output_dir)

    if not os.path.exists(world_file_to_be_inserted):
        world_file_to_be_inserted = ""

    try:
        yaml_parse = BuildingYamlParse(map_file)
    except ValueError as e:
        print('crowdsim unable to parse, not attempting to proceed')
        return

    configfile_generator = ConfigFileGenerator(yaml_parse)
    at_least_a_level_with_crowdsim = False
    for level_name, level in configfile_generator.level.items():
        if level.enable_crowdsim == 1:
            dest_dir = output_dir + level_name
            at_least_a_level_with_crowdsim = True
            if not os.path.exists(dest_dir):
                os.makedirs(dest_dir)
            configfile_generator.generate_behavior_file(level_name, dest_dir)
            configfile_generator.generate_scene_file(level_name, dest_dir)
    if at_least_a_level_with_crowdsim:
        configfile_generator.insert_plugin_into_world_file(
            world_file_to_be_inserted)
