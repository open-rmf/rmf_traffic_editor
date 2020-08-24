import sys
import os
import yaml

import xml.etree.ElementTree as ET

from building_crowdsim.config.template_conf_yaml import *
from building_crowdsim.config.behavior_file import *
from building_crowdsim.config.scene_file import *
from building_crowdsim.config.plugin_file import *
from building_crowdsim.config.util import *

from building_crowdsim.building_yaml_parse import BuildingYamlParse, LevelWithHumanLanes

class ConfigFileGenerator:
    def __init__(self, building_yaml_parse, simulation_platform = "gazebo") :
        assert(isinstance(building_yaml_parse, BuildingYamlParse))
        self.crowd_sim_yaml = building_yaml_parse.GetCrowdSimConfig()
        self.human_goals = {}
        self.load_human_goals(building_yaml_parse.GetHumanGoals() )
        self.behavior_file = BehaviorFile()
        self.scene_file = SceneFile()
        self.plugin_file = Plugin(simulation_platform)

    def load_human_goals(self, human_goals_from_building_yaml_parse):
        self.human_goals = {}
        # human_goals_from_building_yaml_parse is a list with transformed_vertices
        for vertex in human_goals_from_building_yaml_parse :
            # safe check 
            if not 'human_goal_set_name' in vertex.params :
                continue
            set_name = vertex.params['human_goal_set_name'].value
            if not set_name in self.human_goals :
                self.human_goals[set_name] = []
            self.human_goals[set_name].append(vertex)

    def generate_behavior_file(self, output_dir = ""):
        for key in self.crowd_sim_yaml :
            if key == "states" :
                for s in self.crowd_sim_yaml[key] :
                    state = StateYAML().load(s)
                    self.behavior_file.addState(state)
                continue
            if key == "transitions" :
                for t in self.crowd_sim_yaml[key] :
                    transition = TransitionYAML().load(t)
                    self.behavior_file.addTransition(transition)
                continue
            if key == "goal_sets" :
                for gs in self.crowd_sim_yaml[key] :
                    goal_set = GoalSetYAML().load(gs)
                    # update goal_sets with goal
                    for area in gs['set_area'] :
                        if not area in self.human_goals:
                            print("set_area [", area, "] not found in goal set.")
                            continue
                        area_goal_list = self.human_goals[area]
                        for g in area_goal_list :
                            # g is building.map.vertex.Vertex type
                            tmp = Goal()
                            tmp.setCoord(g.x, g.y)
                            tmp.setCapacity(gs['capacity'])
                            goal_set.addGoal(tmp)
                    self.behavior_file.addGoalSet(goal_set)
                continue

        writeXmlFile(self.behavior_file.outputXmlElement(), output_dir = output_dir, file_name = 'behavior_file.xml')

    def generate_scene_file(self, output_dir) :
        # add default configuration
        self.scene_file.addSpatialQuery()
        self.scene_file.addCommon()

        for key in self.crowd_sim_yaml :
            if key == 'obstacle_set' :
                # only one obstacle set
                tmp = ObstacleSetYAML().load(self.crowd_sim_yaml[key])
                self.scene_file.addSubElement(tmp)
                continue
            if key == 'agent_profiles' :
                for item in self.crowd_sim_yaml[key]:
                    tmp = AgentProfileYAML().load(item)
                    self.scene_file.addSubElement(tmp)
                continue
            if key == 'agent_group' :
                for item in self.crowd_sim_yaml[key]:
                    tmp = AgentGroupYAML().load(item)
                    self.scene_file.addSubElement(tmp)
                continue
        writeXmlFile(self.scene_file.outputXmlElement(), output_dir = output_dir, file_name = 'scene_file.xml')

    def generate_plugin_file(self) :
        external_agent_yaml = ExternalAgentYAML()
        
        for key in self.crowd_sim_yaml :
            if key == 'agent_group' :
                for group_item in self.crowd_sim_yaml[key] :
                    if not 'agents_name' in group_item or not group_item['agents_name'] :
                        continue
                    external_agent_yaml.load(group_item['agents_name'])
                self.plugin_file.addExternalAgentList(external_agent_yaml.getExternalAgents())
            
            if key == 'update_time_step' :
                self.plugin_file.setUpdateTimeStep(self.crowd_sim_yaml[key])
                continue

            if key == 'model_types' :
                for type_item in self.crowd_sim_yaml[key] :
                    model_type_ = ModelTypeYAML().load(type_item)
                    self.plugin_file.addModelType(model_type_)
                continue
        # set the initialized flag
        self.plugin_file._initialized = True
    
    def insert_plugin_into_world_file(self, world_file_to_be_inserted) :
        tmp = ET.parse(world_file_to_be_inserted)
        file_root = tmp.getroot()
        world_root = None
        for root_child in file_root :
            if root_child.tag != "world" :
                continue
            world_root = root_child
        if not world_root :
            raise ValueError("Invalid world file! please check your world file: ", world_file_to_be_inserted)    
        if not self.plugin_file._initialized :
            raise ValueError("Plugin file has not been initialized! call generate_plugin_file() first")
        world_root.append(self.plugin_file.outputXmlElement())
        writeXMLtoCompleteFilePath(file_root, file_name = world_file_to_be_inserted)


    
def main():

    if len(sys.argv) < 3 :
        raise ValueError("Please provide config_yaml_path and the output_dir as required.")
        sys.exit(1)
    
    if len(sys.argv) == 3 :
        config_yaml_path = sys.argv[1]
        output_dir = sys.argv[2]

    if config_yaml_path[0] != '/' :
        config_yaml_path = os.getcwd() + '/' + config_yaml_path

    if not os.path.exists(config_yaml_path):
        raise ValueError('Map path not exist!: ' + config_yaml_path)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    yaml_parse = BuildingYamlParse(config_yaml_path)
    configfile_generator = ConfigFileGenerator(yaml_parse)
    configfile_generator.generate_behavior_file(output_dir)
    configfile_generator.generate_scene_file(output_dir)
    configfile_generator.generate_plugin_file()
    ET.dump(configfile_generator.plugin_file.outputXmlElement())

    

    # if not model_env :
    #     return

    # if world_file_to_be_inserted[0] != '/' :
    #     world_file_to_be_inserted = os.getcwd() + '/' + world_file_to_be_inserted
    # if not os.path.exists(world_file_to_be_inserted) :
    #     raise ValueError("world file does not exists!: " + world_file_to_be_inserted)
    
    # generate_plugin_file(yaml_parse.getRawData(), model_env, world_file_to_be_inserted)

if __name__ == '__main__':
    sys.exit(main())
