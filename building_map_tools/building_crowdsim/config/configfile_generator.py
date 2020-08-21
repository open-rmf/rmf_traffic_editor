import sys
import os
import yaml

import xml.etree.ElementTree as ET

from .template_conf_yaml import *
from .behavior_file import *
from .scene_file import *
from .plugin_file import *
from .util import *

class BuildConfigYaml:
    def __init__(self, yaml_file):
        with open(yaml_file) as file:
            self._yaml_node = yaml.load(file, yaml.SafeLoader)

    # todo: add multiple levels integration     
    def getRawData(self):
        return self._yaml_node
    
    def getLevelData(self, level_name) :
        if not str(level_name) in self._yaml_node :
            raise ValueError("invalid 'level_name' provided")
        return self._yaml_node[str(level_name)]

def generate_behavior_file(yaml_node, output_dir):
    behavior_file = BehaviorFile()

    # first add all the goals
    goals = GoalsYAML()
    goals.loadGoal(yaml_node['goals'])

    # construct behavior file element
    for key in yaml_node:
        if key == "state" :
            for s in yaml_node[key] :
                state = StateYAML().load(s)
                behavior_file.addState(state)
        if key == "transition" :
            for t in yaml_node[key] :
                transition = TransitionYAML().load(t)
                behavior_file.addTransition(transition)
        if key == "goal_set" :
            for gs in yaml_node[key] :
                goal_set = GoalSetYAML().load(gs)

                # update goal_sets with goal
                for area in gs['set_area'] :
                    area_list = goals.getGoals(area)
                    for g in area_list :
                        # g is GoalYAML type
                        tmp = Goal()
                        tmp.setCoord(g._x, g._y)
                        tmp.setCapacity(gs['capacity'])
                        goal_set.addGoal(tmp)

                behavior_file.addGoalSet(goal_set)

    writeXmlFile(behavior_file.outputXmlElement(), output_dir = output_dir, file_name = 'behavior_file.xml')

def generate_scene_file(yaml_node, output_dir):
    scene_file = SceneFile()

    # add default configuration
    scene_file.addSpatialQuery()
    scene_file.addCommon()

    # load all the agent list first
    if not 'agent_list' in yaml_node :
        raise ValueError("No agent_list provided!")

    agent_list = AgentsListYAML()
    agent_list.load(yaml_node['agent_list'])

    for key in yaml_node:
        if key == 'obstacle_set' :
            for item in yaml_node[key] :
                tmp = ObstacleSetYAML().load(item)
                scene_file.addSubElement(tmp)

        if key == 'agent_profile' :
            for item in yaml_node[key]:
                tmp = AgentProfileYAML().load(item)
                scene_file.addSubElement(tmp)

        if key == 'agent_group' :
            for item in yaml_node[key]:
                tmp_yaml = AgentGroupYAML()
                tmp_yaml.load(item)
                tmp = tmp_yaml.loadAgents(agent_list)
                scene_file.addSubElement(tmp)

    writeXmlFile(scene_file.outputXmlElement(), output_dir = output_dir, file_name = 'scene_file.xml')


def load_plugin(yaml_node, model_env) :
    plugin = Plugin(model_env)

    external_agent_yaml = ExternalAgentYAML()

    for key in yaml_node :
        if key == 'model_type' :
            for item in yaml_node[key] :
                model_type = ModelTypeYAML().load(item)
                plugin.addModelType(model_type)
            continue

        if key == 'update_time_step' :
            plugin.setUpdateTimeStep(yaml_node[key])
            continue
        
        # for each level
        if 'agent_list' in yaml_node[key] :
            for item in yaml_node[key]['agent_list'] :
                external_agent_yaml.load(item)
            
    plugin.addExternalAgentList(external_agent_yaml.getExternalAgents())
    return plugin


def insert_plugin_into_world_file(plugin, world_file_to_be_inserted) :
    tmp = ET.parse(world_file_to_be_inserted)
    file_root = tmp.getroot()
    world_root = None
    for root_child in file_root :
        if root_child.tag != "world" :
            continue
        world_root = root_child

    if not world_root :
        raise ValueError("Invalid world file! please check your world file: ", world_file_to_be_inserted)
    
    if not plugin :
        raise ValueError("Invalid plugin!")

    world_root.append(plugin.outputXmlElement())
    writeXMLtoCompleteFilePath(file_root, file_name = world_file_to_be_inserted)


def generate_plugin_file(yaml_node, model_env, world_file_to_be_inserted) :
    plugin = load_plugin(yaml_node, model_env)
    insert_plugin_into_world_file(plugin, world_file_to_be_inserted)
    print("Insert plugin part in: ", world_file_to_be_inserted)

    
def main():
    model_env = None

    if len(sys.argv) > 4 :
        model_env = sys.argv[3]
        # insert in the original file
        world_file_to_be_inserted = sys.argv[4]

    if len(sys.argv) > 2 :
        config_yaml_path = sys.argv[1]
        output_dir = sys.argv[2]
    else: 
        sys.exit(1)
        raise ValueError("Please provide config_yaml_path and the output_dir as required.")
    
    if config_yaml_path[0] != '/' :
        config_yaml_path = os.getcwd() + '/' + config_yaml_path

    if not os.path.exists(config_yaml_path):
        raise ValueError('Map path not exist!: ' + config_yaml_path)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    yaml_parse = BuildConfigYaml(config_yaml_path)
    yaml_node = yaml_parse.getLevelData('L1')

    generate_behavior_file(yaml_node, output_dir)
    generate_scene_file(yaml_node, output_dir)

    if not model_env :
        return

    if world_file_to_be_inserted[0] != '/' :
        world_file_to_be_inserted = os.getcwd() + '/' + world_file_to_be_inserted
    if not os.path.exists(world_file_to_be_inserted) :
        raise ValueError("world file does not exists!: " + world_file_to_be_inserted)
    
    generate_plugin_file(yaml_parse.getRawData(), model_env, world_file_to_be_inserted)

if __name__ == '__main__':
    sys.exit(main())
