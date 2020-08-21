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

    for level_name in yaml_parse.GetLevelNames() :
        # navmesh output
        navmesh_output_file = output_folder_path + '/' + output_file_prefix + '.' + level_name + ".navmesh.nav"
        level_with_human_lanes = yaml_parse.GetLevelWithHumanLanes(level_name)
        navmesh_output(level_name, level_with_human_lanes, navmesh_output_file)


if __name__ == "__main__":
    sys.exit(main())