import sys
import os

from .build_navmesh import BuildNavmesh
from building_crowdsim.building_yaml_parse import\
    BuildingYamlParse, LevelWithHumanLanes
from building_map.vertex import Vertex
from building_map.edge import Edge
from building_map.transform import Transform


class NavmeshGenerator:
    def __init__(
            self,
            level_with_human_lanes,
            level_name,
            default_graph_idx=9):
        assert(isinstance(level_with_human_lanes, LevelWithHumanLanes))
        self.level = level_with_human_lanes
        self.level_name = level_name
        self.current_graph_idx = default_graph_idx

    def load(self):
        self.navmesh_manager = BuildNavmesh()
        lane_vertices_number = self.load_vertices()
        print("Load lane vertices of ", lane_vertices_number)
        lane_number = self.load_human_lanes()
        if lane_number <= 0:
            raise ValueError(
                "loaded 0 human lanes. Error in loading human lanes.")
        print("Load human lanes of", lane_number)

    def generate(self):
        self.navmesh_manager.process()

    def output(self, output_file_path):
        self.navmesh_manager.output(output_file_path)

    def add_lane_vertex(self, vertex_xy):
        self.navmesh_manager.add_lane_vertex(vertex_xy[0], vertex_xy[1])

    def add_lane(self, idx0, idx1, width):
        self.navmesh_manager.add_lane(idx0, idx1, width)

    # add all lane vertices (actually load all the appeared vertices)
    def load_vertices(self):
        for v in self.level.transformed_vertices:
            self.add_lane_vertex(v.xy())
        self.lane_vertices_number = len(self.level.transformed_vertices)
        return self.lane_vertices_number

    def load_human_lanes(self):
        count = 0
        for lane in self.level.human_lanes:
            if lane.params['graph_idx'].value != self.current_graph_idx:
                continue
            # get the width of human lanes
            width = lane.width()
            if lane.start_idx > self.lane_vertices_number or\
               lane.end_idx > self.lane_vertices_number:
                print(
                    "Error load lanes for lane," +
                    "vertices_idx over stored vertices_number. [",
                    lane.start_idx,
                    ",",
                    lane.end_idx,
                    "]")
                raise ValueError("edge is referencing invalid vertex.")
            self.add_lane(lane.start_idx, lane.end_idx, width)
            count += 1
        self.lanes_number = count
        return count


def navmesh_output(level_name, level_yaml_node, output_file_path):
    navmesh_generator = NavmeshGenerator(level_yaml_node, level_name)
    navmesh_generator.load()
    navmesh_generator.generate()
    navmesh_generator.output(output_file_path)
    return navmesh_generator


def navmesh_main(map_file, output_dir):
    if not os.path.exists(map_file):
        raise ValueError('Map path not exist!')

    if not os.path.exists(output_dir):
        print("Creating output folder path: ", output_dir)
        os.makedirs(output_dir)

    # parse the yaml file
    yaml_parse = BuildingYamlParse(map_file)

    for level_name in yaml_parse.levels_name:
        # navmesh output
        navmesh_output_file = output_dir + '/' + level_name + "_navmesh.nav"
        level_with_human_lanes = yaml_parse.levels_with_human_lanes[level_name]
        navmesh_output(level_name, level_with_human_lanes, navmesh_output_file)
