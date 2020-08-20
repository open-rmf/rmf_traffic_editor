import yaml
import sys
import os
import copy

from building_crowdsim_navmesh.build_navmesh import BuildNavmesh
from building_map.vertex import Vertex
from building_map.edge import Edge
from building_map.transform import Transform
# from configfile_generator.template_conf_yaml import *
# from configfile_generator.util import *

class NavmeshGenerator:
    '''Generate navmesh for one level based on 'human_lanes' '''
    def __init__(self, level_yaml_node, name):
        self._name = name
        print("Parsing yaml for level: ", name)

        # z base
        self._elevation = 0.0
        if 'elevation' in level_yaml_node :
            self._elevation = float(level_yaml_node['elevation'])

        # y coordinates set to negative in Vertex
        self._vertices_raw = []
        if 'vertices' in level_yaml_node and level_yaml_node['vertices'] :
            for vertex_raw in level_yaml_node['vertices'] :
                self._vertices_raw.append(Vertex(vertex_raw))
        
        self._transformed_vertices = []

        self._transform = Transform()
        # parse measurement for scaling
        self._meas = []
        if 'measurements' in level_yaml_node :
            self._meas = self.parse_edge_sequence(level_yaml_node['measurements'])
            for meas in self._meas:
                meas.calc_statistics(self._vertices_raw)
        
        # calculate scale and transform all the vertices
        self.calculate_scale_using_measurements()
        self.transform_all_vertices()
        
        # parse human lanes
        self._human_lanes_raw = []
        if 'human_lanes' in level_yaml_node and level_yaml_node['human_lanes'] :
            self._human_lanes_raw = self.parse_edge_sequence(level_yaml_node['human_lanes'])
        else :
            raise ValueError('expected more than 1 human lanes to generate navmesh')

        # default graph idx = 0
        self._cur_graph_idx = 0

    def parse_edge_sequence(self, sequence_yaml):
        edges = []
        for edge_yaml in sequence_yaml:
            edges.append(Edge(edge_yaml))
        return edges


    def calculate_scale_using_measurements(self):
        # use the measurements to estimate scale for this level
        scale_cnt = 0
        scale_sum = 0
        for m in self._meas:
            scale_cnt += 1
            scale_sum += m.params['distance'].value / m.length
        if scale_cnt > 0:
            self._transform.set_scale(scale_sum / float(scale_cnt))
            print(f'level {self._name} scale: {self._transform.scale}')
        else:
            self._transform.set_scale(1.0)
            print('WARNING! No measurements defined. Scale is indetermined.')
            print('         Nav graph generated in pixel units, not meters!')


    def transform_all_vertices(self):
        self._transformed_vertices = []

        for vertex_raw in self._vertices_raw :
            v = copy.deepcopy(vertex_raw)
            transformed = self._transform.transform_point(v.xy())
            v.x, v.y = transformed
            v.z = self._elevation # set the z coordinate as the level z base
            self._transformed_vertices.append(v)

    def get_transformed_vertices(self):
        return self._transformed_vertices
    
    def set_graph_idx(self, graph_idx):
        self._cur_graph_idx = graph_idx

    # load all the transformed vertices and human lanes to building_navmesh interface
    def Load(self):
        self._navmeshManager = BuildNavmesh()

        lane_vertices_number = self.LoadLaneVertices()
        print("Load lane vertices of ", lane_vertices_number)
        
        lane_number = self.LoadHumanLanes()        
        if lane_number <= 0 :
            raise ValueError("loaded 0 human lanes. Error in loading human lanes.")
        print("Load human lanes of", lane_number)

    
    # wrap up building_navmesh api
    def AddLaneVertex(self, vertex_xy):
        self._navmeshManager.AddLaneVertex(vertex_xy[0], vertex_xy[1])

    def AddLane(self, idx0, idx1, width):
        self._navmeshManager.AddLane(idx0, idx1, width)

    def Generate(self):
        self._navmeshManager.Process()

    def Output(self, output_file_path):
        self._navmeshManager.Output(output_file_path)
    
    # add all lane vertices  
    def LoadLaneVertices(self):
        count = 0
        for v in self._transformed_vertices:
            self.AddLaneVertex(v.xy())
            count += 1
        self._lane_vertices_number = count
        return count

    def LoadHumanLanes(self):
        count = 0
        for l in self._human_lanes_raw:
            if int(l.params['graph_idx'].value) != self._cur_graph_idx :
                continue
            # get the width of human lanes
            width = l.width()
            
            if l.start_idx > self._lane_vertices_number or l.end_idx > self._lane_vertices_number :
                print("Error load lanes for lane, vertices_idx over stored vertices_number. [", l.start_idx, ",", l.end_idx, "]" )
                raise ValueError("edge is referencing invalid vertex.")
                        
            self.AddLane(l.start_idx, l.end_idx, width)
            count += 1

        self._lanes_number = count
        return count


def navmesh_output(level_name, level_yaml_node, output_file_path):
    # TODO, add graph_id support
    navmesh_generator = NavmeshGenerator(level_yaml_node, level_name)
    navmesh_generator.Load()
    navmesh_generator.Generate()
    navmesh_generator.Output(output_file_path)

    # provide lane vertices for goals 
    return navmesh_generator


def configfile_plugin_output(configfile_output_file):
    filehandle = open(configfile_output_file, "a")
    newline = '\n'
    indent = ' '
    indent_level = 0

    model_type_list = [ModelTypeYAML().getAttributes()]
    templatePrettyYaml(filehandle, indent_level, 'model_type' + ':')
    indent_level = indent_level + 1

    for item in model_type_list :
        # print key
        content = '- {'
        for key in item:
            content += str(key) + ": " + str(item[key]) + ', '
        content = content[0:-2]  #delete the last ', '
        content += ' }'
        templatePrettyYaml(filehandle, indent_level, content)
    
    filehandle.close()



def main():
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

    if len(sys.argv) > 2:
        output_folder_path = sys.argv[2]
        if not os.path.exists(output_folder_path) :
            print("Creating output folder path: ", output_folder_path)
            os.makedirs(output_folder_path)
    else:
        output_folder_path = os.getcwd() + "navmesh_output"
        if not os.path.exists(output_folder_path) :
            os.makedirs(output_folder_path)

    if len(sys.argv) > 3 :
        output_file_prefix = sys.argv[3]
    else :
        output_file_prefix = 'tmp'

    yaml_parse = BuildingYamlParse(map_path)

    # template configure file for menge
    configfile_output_file = output_folder_path + '/' + output_file_prefix + '.template_conf_menge.yaml'

    clear_filehandle = open(configfile_output_file, "w")

    for level_name in yaml_parse._level_keys :
        # navmesh output
        navmesh_output_file = output_folder_path + '/' + output_file_prefix + '.' + level_name + ".navmesh.nav"
        level_yaml_node = yaml_parse.GeteRawData()[level_name]
        navmesh_generator = navmesh_output(level_name, level_yaml_node, navmesh_output_file)
        # configfile menge part output
        configfile_level_output(level_name, navmesh_generator, configfile_output_file)

    # configfile plugin part output
    configfile_plugin_output(configfile_output_file)
    clear_filehandle.close()

    # log
    if configfile_output_file[0] == '/' :
        # start from root directory
        print("Generate: ", configfile_output_file)
    else :
        print("Generate: ", os.getcwd() + '/' + configfile_output_file)


if __name__ == "__main__":
    sys.exit(main())