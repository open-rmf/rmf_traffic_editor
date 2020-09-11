from .vertex import Vertex
from .object_manager import Manager


class Polygon:

    def __init__(self):
        self.id = -1
        # store the actual vertices, vertices are ordered in sequence!
        self.vertices = []
        self.vertex_ids = set()
        self.edge_ids = set()
        self.obstacle_ids = set()
        # gradient stands for plane equation for Ax+By+C = 0
        self.gradient = [0, 0, 0]
        self.center = Vertex([0, 0])

    def add_vertex(self, vertex):
        assert(isinstance(vertex, Vertex))
        if vertex.id in self.vertex_ids:
            return
        self.vertex_ids.add(vertex.id)
        self.vertices.append(vertex)

    def calulate_center(self):
        num = len(self.vertices)
        assert(num >= 3), "Polygon has less than 3 vertices"
        x = 0.0
        y = 0.0
        for vtx in self.vertices:
            x += vtx.x
            y += vtx.y
        self.center = Vertex([x / num, y / num])

    def get_variable(self):
        result = []
        self.calulate_center()
        result.append(self.center.get_coords())
        vertices_id = []
        for v in self.vertices:
            vertices_id.append(v.id)
        result.append(vertices_id)
        result.append(self.gradient)
        result.append(self.edge_ids)
        result.append(self.obstacle_ids)
        return result


class HubPolygon (Polygon):
    def __init__(self):
        Polygon.__init__(self)
        self.hub_vertex_id = -1
        self.related_lane_ids = []


class LanePolygon (Polygon):
    def __init__(self):
        Polygon.__init__(self)
        self.related_lane_id = -1


class PolygonManager (Manager):
    def __init__(self):
        Manager.__init__(self)
        # pair (hub_vertex, hub_polygon)
        self.polygon_map = dict()

    def update_polygon_set(self, polygon):
        assert(polygon.id >= 0)
        if(isinstance(polygon, HubPolygon)):
            self.polygon_map[polygon.hub_vertex_id] = polygon.id

    def get_hub_polygon_from_hub_vertex(self, hub_vertex_id):
        if(hub_vertex_id in self.polygon_map):
            return self.polygon_map[hub_vertex_id]
        return -1
