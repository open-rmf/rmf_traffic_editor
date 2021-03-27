import os

import xml.etree.ElementTree as ET

from building_crowdsim.config.scene_file import\
    SceneFile, ObstacleSet, AgentProfile, AgentGroup
from building_crowdsim.config.behavior_file import\
    BehaviorFile, BehaviorState, StateTransition, GoalSet
from building_crowdsim.config.plugin_file import Plugin
import building_crowdsim.config.util as util
from building_crowdsim.building_yaml_parse import BuildingYamlParse


def test_scene_file():
    root = SceneFile()
    root.add_spatial_query()
    root.add_common()
    walking_profile = root.add_agent_profile('walking')
    walking_profile.attributes['pref_speed'] = 2
    walking_group = root.add_agent_group('human', 'walking')
    walking_group.add_agent(0.1, 0.2)
    walking_group.add_agent(0.2, 0.3)
    scene_file_root = root.output_xml_element()

    util.pretty_xml(scene_file_root, '\t', '\n')
    ET.dump(scene_file_root)


def test_scene_from_yaml():
    map_file = os.getcwd() + '/test/building_crowdsim/config_test.yaml'
    yaml_parse = BuildingYamlParse(map_file)
    crowd_sim_yaml = yaml_parse.crowd_sim_config

    scene_file = SceneFile()
    scene_file.add_spatial_query()
    scene_file.add_common()
    if 'obstacle_set' in crowd_sim_yaml:
        obstacle_set = ObstacleSet()
        obstacle_set.load_from_yaml(crowd_sim_yaml['obstacle_set'])
        scene_file.sub_elements.append(obstacle_set)
    if 'agent_profiles' in crowd_sim_yaml:
        for item in crowd_sim_yaml['agent_profiles']:
            cur_profile = AgentProfile()
            cur_profile.load_from_yaml(item)
            scene_file.sub_elements.append(cur_profile)
    if 'agent_groups' in crowd_sim_yaml:
        for item in crowd_sim_yaml['agent_groups']:
            cur_group = AgentGroup()
            cur_group.load_from_yaml(item)
            scene_file.sub_elements.append(cur_group)

    scene_file_root = scene_file.output_xml_element()
    util.pretty_xml(scene_file_root, '\t', '\n')
    ET.dump(scene_file_root)


def test_behavior_from_yaml():
    map_file = os.getcwd() + '/test/building_crowdsim/config_test.yaml'
    yaml_parse = BuildingYamlParse(map_file)
    crowd_sim_yaml = yaml_parse.crowd_sim_config

    behavior_file = BehaviorFile()
    if 'states' in crowd_sim_yaml:
        for state in crowd_sim_yaml['states']:
            cur_state = BehaviorState()
            cur_state.load_from_yaml(state)
            behavior_file.sub_elements.append(cur_state)
    if 'transitions' in crowd_sim_yaml:
        for transition in crowd_sim_yaml['transitions']:
            cur_transition = StateTransition()
            cur_transition.load_from_yaml(transition)
            behavior_file.sub_elements.append(cur_transition)
    if 'goal_sets' in crowd_sim_yaml:
        for goal_set in crowd_sim_yaml['goal_sets']:
            cur_goal_set = GoalSet()
            cur_goal_set.load_from_yaml(goal_set, yaml_parse.get_human_goals())
            behavior_file.sub_elements.append(cur_goal_set)

    behavior_file_root = behavior_file.output_xml_element()
    util.pretty_xml(behavior_file_root, '\t', '\n')
    ET.dump(behavior_file_root)


def test_plugin_from_yaml():
    map_file = os.getcwd() + '/test/building_crowdsim/config_test.yaml'
    yaml_parse = yaml_parse = BuildingYamlParse(map_file)
    crowd_sim_yaml = yaml_parse.crowd_sim_config

    plugin_file = Plugin()
    plugin_file.load_from_yaml(crowd_sim_yaml)
    plugin_file_root = plugin_file.output_xml_element()
    util.pretty_xml(plugin_file_root, '\t', '\n')
    ET.dump(plugin_file_root)


if __name__ == '__main__':
    print("=========================")
    print("simple_unit_test()")
    print("=========================")
    try:
        test_scene_file()
    except KeyboardInterrupt:
        pass

    print("=========================")
    print("scene_from_yaml_test()")
    print("=========================")
    try:
        test_scene_from_yaml()
    except KeyboardInterrupt:
        pass

    print("=========================")
    print("behavior_from_yaml_test()")
    print("=========================")
    try:
        test_behavior_from_yaml()
    except KeyboardInterrupt:
        pass

    print("=========================")
    print("plugin_from_yaml_test()")
    print("=========================")
    try:
        test_plugin_from_yaml()
    except KeyboardInterrupt:
        pass

    print("=========================")
    print("All Pass")
    print("=========================")
