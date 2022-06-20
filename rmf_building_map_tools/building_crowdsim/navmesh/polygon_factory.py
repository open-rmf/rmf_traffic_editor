from collections.abc import Iterable
import sys
import os

from .vector import Vector2d
from .vertex import Vertex
from .edge import Edge
from .obstacle import Obstacle
from .polygon import HubPolygon, LanePolygon
from .object_manager import Manager


class PolygonFactory:
    def __init__(
            self,
            connection_manager,
            polygon_vertex_manager,
            edge_manager,
            obstacle_manager,
            polygon_manager):
        self.connection_manager = connection_manager
        self.lane_vertex_manager = self.connection_manager.lane_vertex_manager
        self.lane_manager = self.connection_manager.lane_manager
        self.polygon_vertex_manager = polygon_vertex_manager
        self.edge_manager = edge_manager
        self.obstacle_manager = obstacle_manager
        self.polygon_manager = polygon_manager
        """
        self.scale_special_case is used when a HubPolygon only has 2 related
        human_lanes. Only 2 polygon vertices can be calculated with 2
        lanes intersect, however 2 polygon vertices are not enough to
        construct a HubPolygon. self.scale_special_case is a predefined
        variable to help generate other 2 polygon vertices. You can check
        self.hub_polygon_special_case_with_2_lanes for more details.
        """
        self.scale_special_case = 0.01
        """
        self.dead_end_extension is used in a dead-end LanePolygon. Unlike a
        general LanePolygon, which only has 2 obstacle edges (the 2 long
        parallel edges), a dead-end LanePolygon has 3 obstacle edges (plus
        the dead-end short edge). But the lane vertex on the dead end side
        might be a goal for humans (like the pantry situation). That way, the
        human will try to reach the target position, but the dead-end obstacle
        keeps the human away from the target. As such, in the dead-end
        LanPolygon case, the actual LanePolygon is 0.1 (dead_end_extension)
        longer than the actual human lane to avoid above situation happens.
        Check self.dead_end_lane_vertices for more details.
        """
        self.dead_end_extension = 0.1

    def hub_polygon_update(self, polygon):
        assert(isinstance(polygon, HubPolygon))
        assert(polygon.hub_vertex_id != -1)

        self.polygon_manager.update_polygon_set(polygon)

        hub_vertex = self.lane_vertex_manager.data[polygon.hub_vertex_id]
        if (len(hub_vertex.related_lane_ids) == 2):
            self.hub_polygon_special_case_with_2_lanes(polygon)
        else:
            self.hub_polygon_general_case(polygon)

    def lane_polygon_update(self, polygon):
        assert(isinstance(polygon, LanePolygon))
        # add already generated polygon vertices to this lane polygon
        lane = self.lane_manager.data[polygon.related_lane_id]
        already_vertex_ids = lane.polygon_vertex_id

        if(len(already_vertex_ids) == 0):
            raise ValueError(
                "You are getting a lane not connected with any other hub." +
                "Please check the building.yaml")

        if(len(already_vertex_ids) > 2):
            self.lane_polygon_general_case(polygon, already_vertex_ids)
        else:
            self.lane_polygon_dead_end_case(polygon, already_vertex_ids)

        assert(len(polygon.vertex_ids) == 4)
        self.link_polygon_edge(polygon)
        # only lane polygon has obstacles
        self.set_polygon_obstacle(polygon)

    def cal_unit_lane_vector_in_sequence_on_intersect_vertex(
            self, intersect_vertex_id):
        """
        This function caluates all the unit_vectors for lanes
        that intersects at the same lane_vertex.
        Besides, this function sort the unit_vectors by their gradient angle.
        Return with a list of pairs (lane_id, lane_unit_vector)
        in orientation sequence
        """
        base_vertex = self.lane_vertex_manager.data[intersect_vertex_id]
        if(len(base_vertex.related_lane_ids) == 0):
            print(
                "This lane vertex: ",
                base_vertex.id,
                "has not been assigned to one lane. Check initialization!")
            return []

        unit_vectors = []
        for lane_id in base_vertex.related_lane_ids:
            tmp_vector = self.connection_manager.get_lane_vector(
                lane_id, base_vertex.id)
            unit_vectors.append((lane_id, tmp_vector.get_unit()))

        unit_vectors.sort(key=lambda x: x[1].get_orientation())
        return unit_vectors

    def construct_vertices(self, polygon_id):
        """
        This function calculates all the polygon vertices for a HubPolygon.
        Return with the polygon vertices instance in sequence.
        """
        polygon = self.polygon_manager.data[polygon_id]
        if(isinstance(polygon, LanePolygon)):
            print(
                "Only HubPolygon constructs polygon vertices.",
                "Current processing polygon: [",
                polygon_id,
                "]")
            return []

        lane_vectors =\
            self.cal_unit_lane_vector_in_sequence_on_intersect_vertex(
                polygon.hub_vertex_id)
        construct_vertices = []

        for i in range(len(lane_vectors)):
            new_polygon_vertex = \
                self.connection_manager.cal_polygon_vertex_from_lane(
                    lane_vectors[i - 1][0],
                    lane_vectors[i][0])
            new_polygon_vertex.related_lane_ids.add(lane_vectors[i - 1][0])
            new_polygon_vertex.related_lane_ids.add(lane_vectors[i][0])
            construct_vertices.append(new_polygon_vertex)

        return construct_vertices

    def hub_polygon_special_case_with_2_lanes(self, polygon):
        """
        Special case where HubPolygon is only connected with 2 LanePolygon.
        In this case, only 2 polygon vertices will be generated from
        self.construct_vertices()
        This function added additional 2 polygon vertices for the HubPolygon
        """
        assert(isinstance(polygon, HubPolygon))
        vertices = self.construct_vertices(polygon.id)
        assert(len(vertices) == 2)
        intersect_vertex = self.lane_vertex_manager.data[polygon.hub_vertex_id]
        v0 = vertices[0]
        v1 = vertices[1]

        # lane_vector is a list in pair (lane_id, lane_unit_vector)
        lane_vectors =\
            self.cal_unit_lane_vector_in_sequence_on_intersect_vertex(
                intersect_vertex.id)
        assert(len(lane_vectors) == 2)
        vector0 = lane_vectors[0][1]
        vector1 = lane_vectors[1][1]

        # v0_pair and v0 constructs an edge related to lane0
        v0_pair = Vertex(
            [v1.x + self.scale_special_case * vector0.x,
             v1.y + self.scale_special_case * vector0.y])
        v0_pair.related_lane_ids.add(lane_vectors[0][0])
        # v1_pair and v1 constructs an edge related to lane1
        v1_pair = Vertex(
            [v0.x + self.scale_special_case * vector1.x,
             v0.y + self.scale_special_case * vector1.y])
        v1_pair.related_lane_ids.add(lane_vectors[1][0])

        # make the vertices in sequence!! This is quite important!
        vertices = [v0, v0_pair, v1, v1_pair]

        # update polygon vertices
        for v in vertices:
            self.polygon_vertex_manager.add_obj(v)
            polygon.add_vertex(v)

        # update polygon edges
        for i in range(2):
            if i == 0:
                id0 = v0.id
                id1 = v0_pair.id
                edge_cross_lane = \
                    self.lane_manager.data[lane_vectors[0][0]]
            else:
                id0 = v1.id
                id1 = v1_pair.id
                edge_cross_lane = \
                    self.lane_manager.data[lane_vectors[1][0]]
            edge_cross_lane.polygon_vertex_id.append(id0)
            edge_cross_lane.polygon_vertex_id.append(id1)
            edge = Edge()
            # the connected lane polygon is not generated yet, use the same
            # polygon id
            edge.init_edge(id0, id1, polygon.id, polygon.id)
            edge.cross_lane_id = edge_cross_lane.id
            self.edge_manager.add_obj(edge)
            polygon.edge_ids.add(edge.id)

    def hub_polygon_general_case(self, polygon):
        # update polygon vertices
        vertices = self.construct_vertices(polygon.id)
        for v in vertices:
            self.polygon_vertex_manager.add_obj(v)
            polygon.add_vertex(v)
            for related_lane_id in v.related_lane_ids:
                self.lane_manager\
                    .data[related_lane_id]\
                    .polygon_vertex_id.append(v.id)
        # update edges
        for idx in range(len(vertices)):
            common_lane = vertices[idx].related_lane_ids.intersection(
                vertices[idx - 1].related_lane_ids)
            # only 1 common_lane expected
            assert(len(common_lane) == 1)
            edge = Edge()
            edge.init_edge(vertices[idx].id,
                           vertices[idx - 1].id,
                           polygon.id,
                           polygon.id)
            edge.cross_lane_id = list(common_lane)[0]
            self.edge_manager.add_obj(edge)
            polygon.edge_ids.add(edge.id)

    def rearrange_vertices_sequence_for_lane_polygon(
            self, lane_id, polygon_vertex_ids):
        """
        This function aims to rearrange the sequence of the 4 added polygon
        vertices. 0 and 1 are added together; 2 and 3 are added together
        The idea is checking whether the v0, v2 is in the same side of the lane
        If in the same side of lane, the sequence is (0, 2, 3, 1)
        If not, the sequence is (0, 1, 2, 3)
        """
        v0 = self.polygon_vertex_manager.data[polygon_vertex_ids[0]]
        v1 = self.polygon_vertex_manager.data[polygon_vertex_ids[1]]
        v2 = self.polygon_vertex_manager.data[polygon_vertex_ids[2]]
        v3 = self.polygon_vertex_manager.data[polygon_vertex_ids[3]]
        result = [v0, v1, v2, v3]
        if self.connection_manager.is_on_same_side_of_lane(lane_id, v0, v2):
            result = [v0, v2, v3, v1]
        return result

    def lane_polygon_general_case(self, polygon, already_vertices):
        """
        LanePolygon general case does not generate new polygon vertex.
        """
        lane = self.lane_manager.data[polygon.related_lane_id]
        right_vertices_sequence =\
            self.rearrange_vertices_sequence_for_lane_polygon(
                lane.id, already_vertices)

        for v in right_vertices_sequence:
            polygon.add_vertex(v)

    def lane_polygon_dead_end_case(self, polygon, already_vertices):
        """
        LanePolygon dead end case, means this lane is only connected with
        1 hub polygon. There are only 2 already generated vertices.
        2 additional dead end polygon vertices should be generated.
        The sequence of polygon vertices will also be taken care of
        as general case.
        """
        assert(isinstance(polygon, LanePolygon))
        assert(len(already_vertices) == 2)

        lane = self.lane_manager.data[polygon.related_lane_id]
        construct_vertices = []
        dead_end_id = -1
        for dead_end_id in lane.lane_vertex_id:
            if len(
                self.lane_vertex_manager
                    .data[dead_end_id]
                    .related_lane_ids) == 1:
                break
        construct_vertices = self.dead_end_lane_vertices(
            polygon.id, dead_end_id)

        v2 = construct_vertices[0]
        v3 = construct_vertices[1]
        self.polygon_vertex_manager.add_obj(v2)
        self.polygon_vertex_manager.add_obj(v3)
        already_vertices.extend([v2.id, v3.id])

        right_vertices_sequence =\
            self.rearrange_vertices_sequence_for_lane_polygon(
                lane.id, already_vertices)

        for v in right_vertices_sequence:
            polygon.add_vertex(v)

    def dead_end_lane_vertices(self, polygon_id, dead_end_vertex_id):
        """
        Generate 2 additional polygon vertices for dead end LanePolygon
        """
        polygon = self.polygon_manager.data[polygon_id]
        # must be a lane node
        assert(isinstance(polygon, LanePolygon))
        lane = self.lane_manager.data[polygon.related_lane_id]

        lane_vector = self.connection_manager.get_lane_vector(
            lane.id, dead_end_vertex_id)
        lane_vector_unit = lane_vector.get_unit()
        lane_normal_unit0 = lane_vector.get_normal_unit()
        lane_normal_unit1 = Vector2d(
            [-lane_normal_unit0.x,
             -lane_normal_unit0.y])

        dead_end_vertex = self.lane_vertex_manager.data[dead_end_vertex_id]
        base_vertex = Vertex([
            dead_end_vertex.x - self.dead_end_extension * lane_vector_unit.x,
            dead_end_vertex.y - self.dead_end_extension * lane_vector_unit.y])

        lane_width = lane.width

        new_vertex0 = Vertex(
            [base_vertex.x + 0.5 * lane_width * lane_normal_unit0.x,
             base_vertex.y + 0.5 * lane_width * lane_normal_unit0.y])
        new_vertex1 = Vertex(
            [base_vertex.x + 0.5 * lane_width * lane_normal_unit1.x,
             base_vertex.y + 0.5 * lane_width * lane_normal_unit1.y])

        return [new_vertex0, new_vertex1]

    def link_polygon_edge(self, lane_polygon):
        """
        This function can only be called by lane polygon. This function should
        be called after all the 4 polygon vertices are added. This function
        aims to update the edge information between LanePolygon and HubPolygon
        """
        assert(isinstance(lane_polygon, LanePolygon))
        assert(len(lane_polygon.vertex_ids) == 4)
        lane = self.lane_manager.data[lane_polygon.related_lane_id]

        for v_id in lane.lane_vertex_id:
            # get the neighbor HubPolygon
            neighbor_polygon_id = \
                self.polygon_manager.get_hub_polygon_from_hub_vertex(v_id)
            # ignore dead end case
            if(neighbor_polygon_id < 0):
                continue

            connection_edge = -1
            for edge_id in\
                    self.polygon_manager\
                        .data[neighbor_polygon_id]\
                        .edge_ids:
                if(self.edge_manager.data[edge_id].cross_lane_id == lane.id):
                    connection_edge = edge_id
                    break
            assert(connection_edge >= 0)

            edge = self.edge_manager.data[connection_edge]
            edge.init_edge(
                edge.v0_id,
                edge.v1_id,
                edge.node0_id,
                lane_polygon.id)
            lane_polygon.edge_ids.add(edge.id)

    def set_polygon_obstacle(self, lane_polygon):
        """
        This function can only be called by LanePolygon, HubPolygon has no
        obstacles. Obstacle should be added after edge.
        There are 3 obstacles at most (3 is when the LanePolygon is a dead end)
        There are 4 polygon vertices for LanePolygon.
        The idea is 1 obstacle is constructed by 2 polygon vertices that
        on the same side of lane. 2 cases by checking v0-v1 pair and v0-v3 pair
        """
        assert(isinstance(lane_polygon, LanePolygon))
        assert(len(lane_polygon.vertex_ids) == 4)
        assert(len(lane_polygon.edge_ids) != 0)

        lane = self.lane_manager.data[lane_polygon.related_lane_id]
        polygon_vertices = lane_polygon.vertices
        v0 = polygon_vertices[0]
        v1 = polygon_vertices[1]
        v2 = polygon_vertices[2]
        v3 = polygon_vertices[3]

        assert(v0.id >= 0
               and v1.id >= 0
               and v2.id >= 0
               and v3.id >= 0)

        obstacle_pairs = []
        # v0-v1 pair on the same side
        if self.connection_manager.is_on_same_side_of_lane(lane.id, v0, v1):
            obstacle_pairs.append((v0.id, v1.id))
            obstacle_pairs.append((v2.id, v3.id))
        else:  # v0-v3 pair on the same side
            obstacle_pairs.append((v0.id, v3.id))
            obstacle_pairs.append((v1.id, v2.id))

        for pair in obstacle_pairs:
            obstacle = Obstacle()
            obstacle.init_obstacle(pair[0], pair[1], lane_polygon.id)
            self.obstacle_manager.add_obj(obstacle)
            lane_polygon.obstacle_ids.add(obstacle.id)

        # dead end case, one more obstacle added
        if len(lane_polygon.edge_ids) == 1:
            edge_id = list(lane_polygon.edge_ids)[0]
            edge = self.edge_manager.data[edge_id]
            on_edge_vertex_ids = [edge.v0_id, edge.v1_id]

            obstacle_pairs = list(
                lane_polygon.vertex_ids - set(on_edge_vertex_ids))

            obstacle = Obstacle()
            obstacle.init_obstacle(
                obstacle_pairs[0],
                obstacle_pairs[1],
                lane_polygon.id)
            self.obstacle_manager.add_obj(obstacle)
            lane_polygon.obstacle_ids.add(obstacle.id)
