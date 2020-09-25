from .vector import Vector2d
from .vertex import Vertex
from .object_manager import Manager


class Lane:
    def __init__(self, params):
        self.id = -1
        # params for [lane_vertex0_id, lane_vertex1_id, width]
        self.lane_vertex_id = params[0:2]
        self.width = params[2]
        self.polygon_vertex_id = []

    def get_lane_vector(
            self,
            lane_vertex_manager,
            base_vertex_id):
        assert(base_vertex_id in self.lane_vertex_id)
        assert(isinstance(lane_vertex_manager, Manager))

        base_vertex = lane_vertex_manager.data[base_vertex_id]
        assert(len(self.lane_vertex_id) == 2)
        to_vertex_id = self.lane_vertex_id[0]
        if base_vertex_id == self.lane_vertex_id[0]:
            to_vertex_id = self.lane_vertex_id[1]
        assert(to_vertex_id != base_vertex_id)
        to_vertex = lane_vertex_manager.data[to_vertex_id]

        result = Vector2d()
        result.init_with_2_vertex(base_vertex, to_vertex)

        return result
