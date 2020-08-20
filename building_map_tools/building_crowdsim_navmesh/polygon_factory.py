from collections import Iterable
import sys
import os

from .vector import Vector2d
from .vertex import Vertex, VertexManager
from .edge import Edge, EdgeManager
from .obstacle import Obstacle, ObstacleManager
from .polygon import HubPolygon, LanePolygon
from .util import *


class PolygonFactory:
    def __init__(self, polygonManager, laneVertexManager, laneManager, vertexManager, edgeManager, obstacleManager):
        self.polygonManager = polygonManager
        self.laneVertexManager = laneVertexManager
        self.laneManager = laneManager
        self.vertexManager = vertexManager
        self.edgeManager = edgeManager
        self.obstacleManager = obstacleManager
        # predefined scale factor, see self.intersectionPolygonUpdate() special case explaination
        self.scale_special_case = 0.5

    def getLaneVector(self, lane):
        lane_vertices = lane.getLaneVertices()
        vector = Vector2d()
        vector.initWith2P(self.vertexManager.getVertex(lane_vertices[0]), self.vertexManager.getVertex(lane_vertices[1]))
        return vector.getUnit()

    def calUnitVectorForIntersectionLanes(self, intersect_vertex):
        # return with elements vector (lane_instance, unite_vector_use_intersect_vertex_as_origin)
        if(intersect_vertex.getLanesSize() == 0):
            print("This lane vertex: ", intersect_vertex.getId(), "has not been assigned to one lane. Check initialization!")
        assert(intersect_vertex.getLanesSize() != 0)

        far_end_lane_vertex = []
        intersect_id = intersect_vertex.getId()
        for lane_id in intersect_vertex.getLanes() :
            lane = self.laneManager.getLane(lane_id)
            v0_id = lane.getLaneVertices()[0]
            v1_id = lane.getLaneVertices()[1]
            if(v0_id == intersect_id) :
                assert(v1_id != intersect_vertex)
                far_end_lane_vertex.append( (lane, self.laneVertexManager.getLaneVertex(v1_id) ))
            else :
                assert(v1_id == intersect_vertex.getId())
                far_end_lane_vertex.append( (lane, self.laneVertexManager.getLaneVertex(v0_id)) )

        lane_vector = []
        for pair in far_end_lane_vertex:
            # pair[0] is lane instance
            # pair[1] is the far_end_lane_vertex
            vector = Vector2d([0.0, 0.0])
            # pointing from v0 to v1
            vector.initWith2P(intersect_vertex, pair[1])
            lane_vector.append((pair[0], vector.getUnit()))
        
        return lane_vector
    
    def constructVertices(self, polygon_id):
        # construct the polygon vertices and return vertices in sequence
        polygon = self.polygonManager.getPolygon(polygon_id)
        if(isinstance(polygon, LanePolygon)) :
            print("Only polygon with intersection lane vertex constructs polygon vertices. Current processing polygon: [", polygon_id, "]")
            return

        intersect_vertex = self.laneVertexManager.getLaneVertex(polygon.getHubVertexId())
        lane_vector = self.calUnitVectorForIntersectionLanes(intersect_vertex)
        lane_vector.sort(key=lambda x: x[1].getOrientation())

        construct_vertices = []
        back_id = len(lane_vector)-1
        assert(back_id != 0)
        new_vertex = calIntersectVertexFromLaneVector(lane_vector, back_id, 0, intersect_vertex)
        new_vertex.setLane(lane_vector[back_id][0].getId())
        new_vertex.setLane(lane_vector[0][0].getId())
        construct_vertices.append(new_vertex)

        for id in range(1, len(lane_vector)):
            new_vertex = calIntersectVertexFromLaneVector(lane_vector, id-1, id, intersect_vertex)
            new_vertex.setLane(lane_vector[id-1][0].getId())
            new_vertex.setLane(lane_vector[id][0].getId())
            construct_vertices.append(new_vertex)
        
        return construct_vertices

    def deadEndLaneVertices(self, polygon_id, lane_vertex_id):
        polygon = self.polygonManager.getPolygon(polygon_id)
        # must be a lane node
        assert(isinstance(polygon, LanePolygon))
        lane = self.laneManager.getLane(polygon.getLaneId())
        
        if list(lane.getLaneVertices())[0] == lane_vertex_id :
            v0 = self.laneVertexManager.getLaneVertex(list(lane.getLaneVertices())[1])
            v1 = self.laneVertexManager.getLaneVertex(list(lane.getLaneVertices())[0])
        else :
            v0 = self.laneVertexManager.getLaneVertex(list(lane.getLaneVertices())[0])
            v1 = self.laneVertexManager.getLaneVertex(list(lane.getLaneVertices())[1])
        
        lane_vector = Vector2d([0.0, 0.0])
        lane_vector.initWith2P(v0, v1)
        lane_vector_unit = lane_vector.getUnit()
        lane_normal_unit0 = lane_vector.getNormalUnit()
        lane_normal_unit1 = Vector2d([-lane_normal_unit0.x, -lane_normal_unit0.y])

        orign_point = self.laneVertexManager.getLaneVertex(lane_vertex_id)
        lane_width = lane.getWidth()

        new_vertex0 = Vertex(
            [orign_point.x + 0.5 * lane_width * (lane_normal_unit0.x + lane_vector_unit.x), 
             orign_point.y + 0.5 * lane_width * (lane_normal_unit0.y + lane_vector_unit.y) ])
        new_vertex1 = Vertex(
            [orign_point.x + 0.5 * lane_width * (lane_normal_unit1.x + lane_vector_unit.x),
             orign_point.y + 0.5 * lane_width * (lane_normal_unit1.y + lane_vector_unit.y) ])

        return [new_vertex0, new_vertex1]

    def linkPolygonEdge(self, lane_polygon):
        # this function can only be called by lane_polygon, coz intersection polygon must be surrounded by lane polygon
        # this function must be called after the 4 vertices of lane polygon are generated and added
        assert(isinstance(lane_polygon, LanePolygon))
        assert(lane_polygon.getVertexSize() == 4)

        lane = self.laneManager.getLane(lane_polygon.getLaneId())
        lane_vertices = lane.getLaneVertices()
        for v_id in lane_vertices :
            neighbor_polygon_id = self.polygonManager.getPolygonIdFromIntersectVertexId(v_id)
            if(neighbor_polygon_id < 0) :
                # this lane_vertice is a dead end
                continue

            neighbor_polygon_edge_ids = self.polygonManager.getPolygon(neighbor_polygon_id).getEdge()
            connection_edge = -1
            for id in neighbor_polygon_edge_ids:
                if(self.edgeManager.getEdge(id).getLane() == lane.getId()) :
                    connection_edge = id
                    break
            assert(connection_edge >= 0)

            edge = self.edgeManager.getEdge(connection_edge)
            edge.setEdge(edge.v0, edge.v1, neighbor_polygon_id, lane_polygon.getId())
            lane_polygon.addEdge(edge.getId())

    def setPolygonObstacle(self, lane_polygon):
        '''
        # this function can only be called by lane_polygon, coz in intersection polygon, each edge is a connection with other polygon
        # obstacle must be added after edge
        # there are 3 obstacles at most (3 is when the lane_polygon is dead end)
        '''
        assert(isinstance(lane_polygon, LanePolygon))
        assert(lane_polygon.getVertexSize() == 4)
        assert(len(lane_polygon.getEdge()) != 0)

        lane = self.laneManager.getLane(lane_polygon.getLaneId())

        # here lane_vertex is used to detect points on the same side of lane
        lane_vertex = self.laneVertexManager.getLaneVertex(lane.getLaneVertices()[0])
        lane_vector = Vector2d()
        lane_vector.initWith2P(lane_vertex, self.laneVertexManager.getLaneVertex(lane.getLaneVertices()[1]))

        # get base two polygon vertices from the first edge
        _ = self.edgeManager.getEdge(lane_polygon.getEdge()[0])
        v0 = _.v0
        v1 = _.v1
        # other 2 vertices
        v2 = -1
        v3 = -1
        
        for v in lane_polygon.getVertex():
            # v stored in polygon.getVertex() is instance
            if v0 == v.getId() or v1 == v.getId():
                continue
            if(onSameSideOfLane(lane_vector=lane_vector, lane_vertex=lane_vertex, vertex0=self.vertexManager.getVertex(v0), vertex1=v)) :
                v2 = v.getId()
                obstacle = Obstacle()
                obstacle.setObstacle(v0, v2, lane_polygon.getId())
                self.obstacleManager.addObstacle(obstacle)
                lane_polygon.addObstacle(obstacle.getId())
                continue
            if(onSameSideOfLane(lane_vector=lane_vector, lane_vertex=lane_vertex, vertex0=self.vertexManager.getVertex(v1), vertex1=v)) :
                v3 = v.getId()
                obstacle = Obstacle()
                obstacle.setObstacle(v1, v3, lane_polygon.getId())
                self.obstacleManager.addObstacle(obstacle)
                lane_polygon.addObstacle(obstacle.getId())
                continue
            
        # if this lane polygon only have one edge, the third obstacle should be added
        if(len(lane_polygon.getEdge()) == 1):
            assert(v2 != -1 and v3 != -1)
            obstacle = Obstacle()
            obstacle.setObstacle(v2, v3, lane_polygon.getId())
            self.obstacleManager.addObstacle(obstacle)
            lane_polygon.addObstacle(obstacle.getId())

    def hubPolygonUpdate(self, polygon) :
        assert(isinstance(polygon, HubPolygon))
        self.polygonManager.addPolygon(polygon)
        self.polygonManager.updatePolygonSet(polygon)

        hub_vertex = self.laneVertexManager.getLaneVertex(polygon.getHubVertexId())
        if (hub_vertex.getLanesSize() == 2) :
            self.hubPolygonSpecialCaseWith2Lanes(polygon)
        else:
            self.hubPolygonGeneralCase(polygon)

    def hubPolygonSpecialCaseWith2Lanes(self, polygon):
        # special case with HubPolygon only intersected with 2 lanes
        assert(isinstance(polygon, HubPolygon))
        vertices = self.constructVertices(polygon.getId())
        assert(len(vertices) == 2)
        intersect_vertex = self.laneVertexManager.getLaneVertex(polygon.getHubVertexId())
        v0 = vertices[0]
        # v1 = Vertex([2 * intersect_vertex.x - v0.x, 2 * intersect_vertex.y - v0.y])
        v1 = vertices[1]
        # for id in v0.getLane():
        #     v1.setLane(id) # might not be necessary, but keep the same as v0
        
        '''
        ## Some Explaination: essentially, this polygon should not appear coz this polygon can be represent by an edge
        ## but then there will be another special case for lane node connected to lane node, then more special cases will come up. 
        ## so for this special case, the polygon will be constraint within a really thin area that almost as an edge
        '''
        # lane_vector is in pair (lane, lane_vector)
        lane_vector = self.calUnitVectorForIntersectionLanes(intersect_vertex)
        assert(len(lane_vector) == 2)
        vector0 = lane_vector[0][1]
        vector1 = lane_vector[1][1]

        # v0_pair and v0 constructs an edge related to lane0, v0_pair is generated from v1
        v0_pair = Vertex(
            [v1.x + self.scale_special_case * vector0.x,
                v1.y + self.scale_special_case * vector0.y])
        v0_pair.setLane(lane_vector[0][0].getId())

        v1_pair = Vertex(
            [v0.x + self.scale_special_case * vector1.x,
                v0.y + self.scale_special_case * vector1.y])
        v1_pair.setLane(lane_vector[1][0].getId())

        ## make the vertices in sequence!! This is quite important!
        vertices = [v0, v0_pair, v1, v1_pair]

        # update polygon vertices
        for v in vertices:
            self.vertexManager.addVertex(v)
            assert(v.getId() >= 0 and v.getId() < self.vertexManager.getSize())
            polygon.addVertex(v) # adding the vertex instance
        
        # update polygon edges
        for i in range(2):
            id0 = -1
            id1 = -1
            lane = lane_vector[i][0]
            edge = Edge()
            if i == 0:
                id0 = v0.getId()
                id1 = v0_pair.getId()
            else:
                id0 = v1.getId()
                id1 = v1_pair.getId()
            lane.addPolygonVertices(id0)
            lane.addPolygonVertices(id1)
            edge.setEdge(id0, id1, polygon.getId(), polygon.getId())
            edge.setLane(lane.getId())
            self.edgeManager.addEdge(edge)
            polygon.addEdge(edge.getId())
        
    def hubPolygonGeneralCase(self, polygon):
        # HubPolygon with >= 3 intersection lanes
        vertices = self.constructVertices(polygon.getId())
        for v in vertices :
            self.vertexManager.addVertex(v)
            assert(v.getId() >= 0 and v.getId() < self.vertexManager.getSize())
            polygon.addVertex(v)
            related_lane = list(v.getLane())
            for related_lane_id in related_lane :
                self.laneManager.getLane(related_lane_id).addPolygonVertices(v.getId())
        for idx in range(len(vertices)):
            edge = Edge()
            # first set no connection with other polygon because now the lane node has not been created
            edge.setEdge(vertices[idx].getId(), vertices[idx-1].getId(), polygon.getId(), polygon.getId())
            common_lane = vertices[idx].getLane().intersection(vertices[idx-1].getLane())
            if(len(common_lane) != 1) :
                print("These 2 polygon vertices have more than 1 common lane: [", 
                    vertices[idx].getId(), ", ", vertices[idx-1].getId(), "]")
            assert(len(common_lane) == 1)
            edge.setLane(list(common_lane)[0])
            self.edgeManager.addEdge(edge)
            polygon.addEdge(edge.getId())

    def lanePolygonDeadEndCase(self, polygon, already_vertices):
        assert(isinstance(polygon, LanePolygon))
        # add alread vertices
        for v_id in already_vertices:
            polygon.addVertex(self.vertexManager.getVertex(v_id))

        lane_v_ids = list(self.laneManager.getLane(polygon.getLaneId()).getLaneVertices())
        construct_vertices = []
        for id in lane_v_ids:
            if(len(self.laneVertexManager.getLaneVertex(id).getLanes()) > 1) :
                continue
            construct_vertices = self.deadEndLaneVertices(polygon.getId(), id)
            # There are only two vertices in construct_vertices, make polygon vertices sequence for lane polygon
            v0 = construct_vertices[0]
            v1 = construct_vertices[1]
            base_point = self.laneVertexManager.getLaneVertex(id)

            vector_base = Vector2d([0.0, 0.0])
            vector_base.initWith2P(base_point, polygon.getVertex()[0])
            vector0 = Vector2d([0.0, 0.0])
            vector0.initWith2P(base_point, v0)
            if(vector_base.getCross(vector0) < 0):
                construct_vertices = [v1, v0]
    
        for v in construct_vertices:
            self.vertexManager.addVertex(v)
            polygon.addVertex(v)

    def lanePolygonGeneralCase(self, polygon, already_vertices):
        assert(isinstance(polygon, LanePolygon))
        assert(len(already_vertices) > 2)

        lane = self.laneManager.getLane(polygon.getLaneId())
        # make vertices sequence
        # one lane has only 4 vertices, 0 and 1 are added together, 2 and 3 are added together
        v0 = self.vertexManager.getVertex(already_vertices[0])
        v1 = self.vertexManager.getVertex(already_vertices[2])
        vector = Vector2d()
        vector.initWith2P(v0, v1)
        
        lane_vertices = lane.getLaneVertices()
        lane_vector = Vector2d()
        lane_vector.initWith2P(self.laneVertexManager.getLaneVertex(lane_vertices[0]), self.laneVertexManager.getLaneVertex(lane_vertices[1]))

        if(vector.getCross(lane_vector) != 0):
            # not parallel with lane_vector, change the sequence
            already_vertices = [already_vertices[0], already_vertices[1], already_vertices[3], already_vertices[2]]

        for v_id in already_vertices:
            polygon.addVertex(self.vertexManager.getVertex(v_id))

    def lanePolygonUpdate(self, polygon):
        assert(isinstance(polygon, LanePolygon))
        self.polygonManager.addPolygon(polygon)
        # add already generated polygon vertices to this lane polygon
        lane = self.laneManager.getLane(polygon.getLaneId())
        already_vertices = lane.getPolygonVertices() # got ids

        if(len(already_vertices) > 2):
            self.lanePolygonGeneralCase(polygon, already_vertices)
        else:
            self.lanePolygonDeadEndCase(polygon, already_vertices)
        
        # lane polygon can and only can have 4 vertices
        assert(polygon.getVertexSize() == 4)
        self.linkPolygonEdge(polygon)
        # only lane polygon has obstacles, coz intersection polygon, each edge is a connection edge
        self.setPolygonObstacle(polygon)
