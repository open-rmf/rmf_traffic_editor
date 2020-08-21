import yaml
import sys
import os
import copy

from building_crowdsim_navmesh.build_navmesh import BuildNavmesh
from building_map.vertex import Vertex
from building_map.edge import Edge
from building_map.transform import Transform
from building_map.level import Level
# from configfile_generator.template_conf_yaml import *
# from configfile_generator.util import *

class NavmeshGenerator (Level):
    '''Generate navmesh for one level based on 'human_lanes' '''
    def __init__(self, level_yaml_node, name, default_graph_idx = 9):
        Level.__init__(self, level_yaml_node, name)
        
        # parse human lanes
        self.human_lanes_raw = []
        if 'human_lanes' in level_yaml_node:
            self.human_lanes_raw = self.parse_edge_sequence(level_yaml_node['human_lanes'])
        else :
            raise ValueError("expected 'human lanes' to generate navmesh")
        
        self.calculate_scale_using_measurements()
        self.transform_all_vertices()
        # default graph idx for human lanes are 9, 
        # but may not necessary when human lanes are not drawning in different graph idx
        self.current_graph_idx = default_graph_idx

    def get_transformed_vertices(self):
        return self.transformed_vertices
    
    def set_graph_idx(self, graph_idx):
        self.current_graph_idx = graph_idx

    # load all the transformed vertices and human lanes to building_navmesh interface
    def Load(self):
        self.navmeshManager = BuildNavmesh()
        lane_vertices_number = self.LoadLaneVertices()
        print("Load lane vertices of ", lane_vertices_number)
        lane_number = self.LoadHumanLanes()        
        if lane_number <= 0 :
            raise ValueError("loaded 0 human lanes. Error in loading human lanes.")
        print("Load human lanes of", lane_number)

    # wrap up building_navmesh api
    def AddLaneVertex(self, vertex_xy):
        self.navmeshManager.AddLaneVertex(vertex_xy[0], vertex_xy[1])

    def AddLane(self, idx0, idx1, width):
        self.navmeshManager.AddLane(idx0, idx1, width)

    def Generate(self):
        self.navmeshManager.Process()

    def Output(self, output_file_path):
        self.navmeshManager.Output(output_file_path)
    
    # add all lane vertices (actually load all the appeared vertices)
    def LoadLaneVertices(self):
        count = 0
        for v in self.transformed_vertices:
            self.AddLaneVertex(v.xy())
            count += 1
        self.lane_vertices_number = count
        return count

    def LoadHumanLanes(self):
        count = 0
        for l in self.human_lanes_raw:
            if int(l.params['graph_idx'].value) != self.current_graph_idx :
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


class BuildingYamlParse:
    def __init__(self, map_path):
        if not os.path.isfile(map_path):
            raise FileNotFoundError(f'input file {map_path} not found')
        self._building_file = map_path
        with open(self._building_file) as f :
            _yaml_raw = yaml.load(f, yaml.SafeLoader)
            self._level_raw = _yaml_raw['levels']
        if not self._level_raw:
            print("Error loading map file: ", map_path)
            return
        self._level_number = len(self._level_raw)
        self._level_keys = list(self._level_raw.keys())

    def GeteRawData(self):
        return self._level_raw

    def GetLevelRawData(self, level_id):
        if level_id > self._level_number :
            print("Error finding level id for ", level_id, ". Total level number is ", self._level_number)
            return
        key = self._level_keys[level_id]
        return self.GetLevelRawDataFromKey(key)
        
    def GetLevelRawDataFromKey(self, key):
        if not key in self._level_keys:
            print("Invalid level name: ", key)
            return
        return self._level_raw[key]

def navmesh_output(level_name, level_yaml_node, output_file_path):
    # TODO, add graph_id support
    navmesh_generator = NavmeshGenerator(level_yaml_node, level_name)
    navmesh_generator.Load()
    navmesh_generator.Generate()
    navmesh_generator.Output(output_file_path)
    # provide lane vertices for goals 
    return navmesh_generator

def main():
    # get the building.yaml file generated from traffic-editor
    if len(sys.argv) > 1 :
        map_path = sys.argv[1]
    elif 'RMF_MAP_PATH' in os.environ:
        map_path = os.environ['RMF_MAP_PATH']
    else:
        print('map path must be provided in command line or RMF_MAP_PATH env')
        sys.exit(1)
        raise ValueError('Map path not provided')
    
    if not os.path.exists(map_path) :
        print('map path does not exist!')
        sys.exit(1)
        raise ValueError('Map path not exist!')

    # provide the output path
    if len(sys.argv) > 2:
        output_folder_path = sys.argv[2]
        if not os.path.exists(output_folder_path) :
            print("Creating output folder path: ", output_folder_path)
            os.makedirs(output_folder_path)
    else:
        output_folder_path = os.getcwd() + "navmesh_output"
        if not os.path.exists(output_folder_path) :
            os.makedirs(output_folder_path)

    # provide the navmeshfile prefix if needed
    if len(sys.argv) > 3 :
        output_file_prefix = sys.argv[3]
    else :
        output_file_prefix = 'tmp'

    # parse the yaml file
    yaml_parse = BuildingYamlParse(map_path)

    for level_name in yaml_parse._level_keys :
        # navmesh output
        navmesh_output_file = output_folder_path + '/' + output_file_prefix + '.' + level_name + ".navmesh.nav"
        level_yaml_node = yaml_parse.GeteRawData()[level_name]
        navmesh_output(level_name, level_yaml_node, navmesh_output_file)


if __name__ == "__main__":
    sys.exit(main())