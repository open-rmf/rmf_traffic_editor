from .object_manager import Manager
from .polygon_factory import PolygonFactory
from .connection import ConnectionManager
from .file_writer import FileWriter
from .vertex import Vertex
from .lane import Lane
from .polygon import HubPolygon, LanePolygon, PolygonManager


class BuildNavmesh:
    def __init__(self):
        # store lane and vertex from building.yaml
        self.lane_vertex_manager = Manager()
        self.lane_manager = Manager()
        self.connection_manager = ConnectionManager(
            self.lane_vertex_manager,
            self.lane_manager)
        # store construct result
        self.polygon_vertex_manager = Manager()
        self.edge_manager = Manager()
        self.obstacle_manager = Manager()
        self.polygon_manager = PolygonManager()
        # construct factory
        self.polygon_factory = PolygonFactory(
            connection_manager=self.connection_manager,
            polygon_vertex_manager=self.polygon_vertex_manager,
            edge_manager=self.edge_manager,
            obstacle_manager=self.obstacle_manager,
            polygon_manager=self.polygon_manager)

    def add_lane_vertex(self, px, py):
        self.lane_vertex_manager.add_obj(Vertex([px, py]))

    def add_lane(self, idx0, idx1, width):
        self.lane_manager.add_obj(Lane([idx0, idx1, width]))

    """
    2 steps for process() function
    1. generate all the HubPolygon
    2. generate all the LanePolygon
    """

    def process(self):
        self.connection_manager.build_connection()
        for lane_vertex in self.lane_vertex_manager.data:
            # ignore the dead end case
            if len(lane_vertex.related_lane_ids) < 2:
                continue

            polygon = HubPolygon()
            polygon.hub_vertex_id = lane_vertex.id
            polygon.related_lane_ids.extend(list(lane_vertex.related_lane_ids))
            self.polygon_manager.add_obj(polygon)
            self.polygon_factory.hub_polygon_update(polygon)

        for lane in self.lane_manager.data:
            polygon = LanePolygon()
            polygon.related_lane_id = lane.id
            self.polygon_manager.add_obj(polygon)
            self.polygon_factory.lane_polygon_update(polygon)

    def output(self, output_file):
        # Python write file
        navmesh_file = FileWriter(output_file)
        navmesh_file.set_data_manager(
            polygon_vertex_manager=self.polygon_vertex_manager,
            edge_manager=self.edge_manager,
            obstacle_manager=self.obstacle_manager,
            polygon_manager=self.polygon_manager)

        navmesh_file.open_file()
        navmesh_file.generate_navmesh()
        navmesh_file.close_file()
