from .object_manager import Manager
from .vertex import Vertex
from .vector import Vector2d


class ConnectionManager:
    def __init__(self, lane_vertex_manager, lane_manager):
        assert(isinstance(lane_vertex_manager, Manager))
        assert(isinstance(lane_manager, Manager))

        self.lane_vertex_manager = lane_vertex_manager
        self.lane_manager = lane_manager

    def build_connection(self):
        assert(len(self.lane_manager.data) != 0)
        for lane_id in range(len(self.lane_manager.data)):
            lane = self.lane_manager.data[lane_id]
            v0_id = lane.lane_vertex_id[0]
            v1_id = lane.lane_vertex_id[1]
            v0 = self.lane_vertex_manager.data[v0_id]
            v1 = self.lane_vertex_manager.data[v1_id]
            v0.related_lane_ids.add(lane.id)
            v1.related_lane_ids.add(lane.id)

    # check the generated polygon vertices (vertex0 and vertex1) is on the
    # same side of lane
    def is_on_same_side_of_lane(self, lane_id, vertex0, vertex1):
        assert(isinstance(vertex0, Vertex))
        assert(isinstance(vertex1, Vertex))

        lane = self.lane_manager.data[lane_id]
        base_vertex = self.lane_vertex_manager.data[lane.lane_vertex_id[0]]
        lane_vector = self.get_lane_vector(lane.id, base_vertex.id)
        vector0 = Vector2d()
        vector0.init_with_2_vertex(base_vertex, vertex0)
        vector1 = Vector2d()
        vector1.init_with_2_vertex(base_vertex, vertex1)

        if lane_vector.get_cross(vector0) *\
           lane_vector.get_cross(vector1) > 0:
            return True
        else:
            return False

    # wrap up for lane.get_lane_vector
    def get_lane_vector(self, lane_id, base_vertex_id):
        lane = self.lane_manager.data[lane_id]
        return lane.get_lane_vector(self.lane_vertex_manager, base_vertex_id)

    # calculate the polygon vertex within the area from vector0 to vector1,
    # mind the sequence
    def cal_polygon_vertex_from_lane(self, id0, id1):
        lane0 = self.lane_manager.data[id0]
        lane1 = self.lane_manager.data[id1]
        base_vertex_id_intersect = list(
            set(lane0.lane_vertex_id) & set(lane1.lane_vertex_id))
        assert(len(base_vertex_id_intersect) == 1)
        base_vertex = self.lane_vertex_manager.data[
            base_vertex_id_intersect[0]]

        vector0 = self.get_lane_vector(lane0.id, base_vertex.id).get_unit()
        vector1 = self.get_lane_vector(lane1.id, base_vertex.id).get_unit()
        width0 = lane0.width
        width1 = lane1.width

        # 2 lanes are nearly parallel case
        if(abs(vector0.get_dot(vector1.get_normal_unit())) < 0.05):
            length = 0.5 * (width0 + width1) / 2
            result_x = length * vector0.get_normal_unit().x
            result_y = length * vector0.get_normal_unit().y
        else:
            a0 = 0.5 * width1 / abs(vector0.get_dot(vector1.get_normal_unit()))
            a1 = 0.5 * width0 / abs(vector1.get_dot(vector0.get_normal_unit()))
            result_x = a0 * vector0.x + a1 * vector1.x
            result_y = a0 * vector0.y + a1 * vector1.y
            # the polygon vertex must be within the area from vertor0 to
            # vector1
            if vector0.get_cross(vector1.get_unit()) < 0:
                result_x = -result_x
                result_y = -result_y

        return Vertex([result_x + base_vertex.x, result_y + base_vertex.y])
