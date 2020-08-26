import sys
import os
import yaml

import xml.etree.ElementTree as ET

from .template_conf_yaml import *
from .behavior_file import *
from .scene_file import *
from .plugin_file import *
from .util import *

from building_crowdsim.building_yaml_parse import BuildingYamlParse, LevelWithHumanLanes

class ConfigFileGenerator:
    def __init__(self, building_yaml_parse, simulation_platform = "gazebo") :
        assert(isinstance(building_yaml_parse, BuildingYamlParse))
        self.crowd_sim_yaml = building_yaml_parse.GetCrowdSimConfig()
        if not 'enable' in self.crowd_sim_yaml :
            raise ValueError("Missing 'enable' tag for crowdsim configuration.")
        self.enable_crowdsim = ( self.crowd_sim_yaml['enable'] == 1 )
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
        # must follow the sequence states, transitions, goal_sets
        if 'states' in self.crowd_sim_yaml :
            for s in self.crowd_sim_yaml['states'] :
                state = StateYAML().load(s)
                self.behavior_file.addState(state)
        if 'transitions' in self.crowd_sim_yaml:
            for t in self.crowd_sim_yaml['transitions'] :
                transition = TransitionYAML().load(t)
                self.behavior_file.addTransition(transition)
        if 'goal_sets' :
            for gs in self.crowd_sim_yaml['goal_sets'] :
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

        writeXmlFile(self.behavior_file.outputXmlElement(), output_dir = output_dir, file_name = 'behavior_file.xml')

    def generate_scene_file(self, output_dir) :
        # add default configuration
        self.scene_file.addSpatialQuery()
        self.scene_file.addCommon()

        # must follow the sequence. 'obstacle_set', 'agent_profiles', 'agent_groups'
        if ('obstacle_set' in self.crowd_sim_yaml) :
            # only one obstacle set
            tmp = ObstacleSetYAML().load(self.crowd_sim_yaml['obstacle_set'])
            self.scene_file.addSubElement(tmp)
        if ('agent_profiles' in self.crowd_sim_yaml) :
            for item in self.crowd_sim_yaml['agent_profiles']:
                tmp = AgentProfileYAML().load(item)
                self.scene_file.addSubElement(tmp)
        if ('agent_groups' in self.crowd_sim_yaml) :
            for item in self.crowd_sim_yaml['agent_groups']:
                tmp = AgentGroupYAML().load(item)
                self.scene_file.addSubElement(tmp)  

        writeXmlFile(self.scene_file.outputXmlElement(), output_dir = output_dir, file_name = 'scene_file.xml')

    def generate_plugin_file(self) :
        external_agent_yaml = ExternalAgentYAML()
        
        for key in self.crowd_sim_yaml :
            if key == 'agent_groups' :
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
        self.generate_plugin_file()
        if not world_file_to_be_inserted:
            print("No world_file provided")
            return
        if not self.enable_crowdsim :
            print("Crowd Simulation is disabled. No plugin will be inserted.")
            return
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
        print("Insert <plugin> tag into ", world_file_to_be_inserted)


def configfile_main(map_file, output_dir, platform, world_file_to_be_inserted):

    if not os.path.exists(map_file):
        raise ValueError('Map path not exist!: ' + map_file)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print("Create output dir for:", output_dir)

    if not platform == "gazebo" and not platform == "ign" :
        print("Platform is expected to be either 'gazebo' or 'ign' for different modeltype.")
        print("'", platform, "' is provided. Using default 'gazebo'")
        platform = "gazebo"

    if not os.path.exists(world_file_to_be_inserted) :
        world_file_to_be_inserted = ""

    yaml_parse = BuildingYamlParse(map_file)
    configfile_generator = ConfigFileGenerator(yaml_parse, platform)
    configfile_generator.generate_behavior_file(output_dir)
    configfile_generator.generate_scene_file(output_dir)
    configfile_generator.insert_plugin_into_world_file(world_file_to_be_inserted)
