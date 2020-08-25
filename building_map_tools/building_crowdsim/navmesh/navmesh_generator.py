import sys
import os

from .build_navmesh import BuildNavmesh
from building_crowdsim.building_yaml_parse import BuildingYamlParse, LevelWithHumanLanes
from building_map.vertex import Vertex
from building_map.edge import Edge
from building_map.transform import Transform


class NavmeshGenerator:
    '''Generate navmesh for one level based on 'human_lanes' '''
    def __init__(self, level_with_human_lanes, level_name, default_graph_idx = 9):
        assert(isinstance(level_with_human_lanes, LevelWithHumanLanes))
        self.level = level_with_human_lanes
        self.level_name = level_name
        self.current_graph_idx = default_graph_idx
    
    def set_graph_idx(self, graph_idx):
        self.current_graph_idx = graph_idx

    def load(self):
        self.navmeshManager = BuildNavmesh()
        lane_vertices_number = self.LoadVertices()
        print("Load lane vertices of ", lane_vertices_number)
        lane_number = self.LoadHumanLanes()        
        if lane_number <= 0 :
            raise ValueError("loaded 0 human lanes. Error in loading human lanes.")
        print("Load human lanes of", lane_number)

    def generate(self):
        self.navmeshManager.Process()

    def output(self, output_file_path):
        self.navmeshManager.Output(output_file_path)

    # wrap up building_navmesh api
    def AddLaneVertex(self, vertex_xy):
        self.navmeshManager.AddLaneVertex(vertex_xy[0], vertex_xy[1])

    def AddLane(self, idx0, idx1, width):
        self.navmeshManager.AddLane(idx0, idx1, width)
    
    # add all lane vertices (actually load all the appeared vertices)
    def LoadVertices(self):
        count = 0
        for v in self.level.get_transformed_vertices():
            self.AddLaneVertex(v.xy())
            count += 1
        self.lane_vertices_number = count
        return count

    def LoadHumanLanes(self):
        count = 0
        for l in self.level.get_human_lanes():
            if l.params['graph_idx'].value != self.current_graph_idx :
                continue
            # get the width of human lanes
            width = l.width()
            if l.start_idx > self.lane_vertices_number or l.end_idx > self.lane_vertices_number :
                print("Error load lanes for lane, vertices_idx over stored vertices_number. [", l.start_idx, ",", l.end_idx, "]" )
                raise ValueError("edge is referencing invalid vertex.")
            self.AddLane(l.start_idx, l.end_idx, width)
            count += 1
        self.lanes_number = count
        return count

def navmesh_output(level_name, level_yaml_node, output_file_path):
    # TODO, add graph_id support
    navmesh_generator = NavmeshGenerator(level_yaml_node, level_name)
    navmesh_generator.load()
    navmesh_generator.generate()
    navmesh_generator.output(output_file_path)
    # provide lane vertices for goals 
    return navmesh_generator

def navmesh_main(map_file, output_dir, output_file_prefix):
    if not os.path.exists(map_file) :
        raise ValueError('Map path not exist!')
        
    if not os.path.exists(output_dir) :
        print("Creating output folder path: ", output_dir)
        os.makedirs(output_dir)

    if len(output_file_prefix) == 0 :
        output_file_prefix = 'temp'

    # parse the yaml file
    yaml_parse = BuildingYamlParse(map_file)

    for level_name in yaml_parse.GetLevelNames() :
        # navmesh output
        navmesh_output_file = output_dir + '/' + level_name + "_navmesh.nav"
        level_with_human_lanes = yaml_parse.GetLevelWithHumanLanes(level_name)
        navmesh_output(level_name, level_with_human_lanes, navmesh_output_file)

