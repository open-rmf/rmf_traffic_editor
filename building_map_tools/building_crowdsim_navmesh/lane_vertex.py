class LaneVertex:
    def __init__(self, coords):
        self.x = coords[0]
        self.y = coords[1]
        self.lanes = set()
    
    def addLane(self, id):
        self.lanes.add(id)

    def addLanes(self, ids):
        for id in ids:
            self.lanes.add(id)

    def setId(self, vertex_id):
        self.id = vertex_id

    def getId(self):
        return self.id
    
    def getLanes(self):
        return self.lanes
    
    def getLanesSize(self):
        return len(self.lanes)

class LaneVertexManager:
    def __init__(self):
        self.lane_vertices = []

    def addLaneVertex(self, lane_vertex):
        id = len(self.lane_vertices)
        lane_vertex.setId(id)
        self.lane_vertices.append(lane_vertex)

    def getSize(self):
        return len(self.lane_vertices)

    def getLaneVertex(self, id):
        assert(id < self.getSize())
        return self.lane_vertices[id]