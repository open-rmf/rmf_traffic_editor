from .vertex import Vertex

class Polygon:
    
    def __init__(self):
        self.vertices = [] # store the actual vertices
        self.verticesIdSet = set()
        self.edges = set()
        self.obstacles = set()
        self.gradient = [0, 0, 0] # equations for plane Ax+By+C = 0

    def setID(self, id):
        self.id = id
    
    def getId(self):
        return self.id

    def addVertex(self, vertex):
        if(vertex.getId() in self.verticesIdSet):
            return
        self.verticesIdSet.add(vertex.getId())
        self.vertices.append(vertex)

    def getVertex(self):
        return list(self.vertices)

    def getVertexSize(self):
        return len(self.vertices)

    def addEdge(self, id):
        self.edges.add(id)

    def addEdges(self, edge_ids):
        for id in edge_ids:
            self.addEdge(id)
    
    def getEdge(self):
        return list(self.edges)
    
    def addObstacle(self, id):
        self.obstacles.add(id)

    def addObstacles(self, obstacle_ids):
        for id in obstacle_ids:
            self.addObstacle(id)

    def calulateCenter(self):
        num = len(self.vertices)
        if(num < 3):
            print("Is not a polygon")
        assert(num >= 3)
        X = 0.0
        Y = 0.0
        for vtx in self.vertices:
            X += vtx.x
            Y += vtx.y
        self.center = Vertex([X/num, Y/num])

    def getVar(self):
        result = []
        self.calulateCenter()
        result.append(self.center.getCoords())
        vertices_id = []
        for v in self.vertices:
            vertices_id.append(v.getId())
        result.append(vertices_id)
        result.append(self.gradient)
        result.append(self.edges)
        result.append(self.obstacles)
        return result

class HubPolygon (Polygon):
    def __init__(self):
        Polygon.__init__(self)
        self.hubVertexId = -1
        self.hubLanesId = []

    def setHubVertexId(self, id):
        self.hubVertexId = id
    
    def getHubVertexId(self) :
        return self.hubVertexId

    def addHubLane(self, lane_id):
        self.hubLanesId.append(lane_id)

    def addHubLanes(self, lanes_id):
        for id in lanes_id:
            self.addHubLane(id)

class LanePolygon (Polygon):
    def __init__(self):
        Polygon.__init__(self)

    def setLaneId(self, id):
        self.laneId = id

    def getLaneId(self) :
        return self.laneId


class PolygonManager:
    def __init__(self):
        self.polygons = []
        self.polygon_map = dict()

    def getSize(self):
        return len(self.polygons)

    def addPolygon(self, polygon):
        id = self.getSize()
        polygon.setID(id)
        self.polygons.append(polygon)

    def getPolygon(self, id):
        assert(id < self.getSize())
        return self.polygons[id]

    def updatePolygonSet(self, polygon):
        # if the polygon is an intersection node, should find the node by the intersection vertex
        if(isinstance(polygon, HubPolygon)) :
            self.polygon_map[polygon.getHubVertexId()] = polygon.getId()
        
    def getPolygonIdFromIntersectVertexId(self, intersect_vertex_id):
        if(intersect_vertex_id not in self.polygon_map):
            # did not find map key, might be a dead end vertex
            # print("Intersect_vertex_id:", intersect_vertex_id, " is not found in polygon map. Check function updatePolygonSet().")
            return -1
        
        return self.polygon_map[intersect_vertex_id]