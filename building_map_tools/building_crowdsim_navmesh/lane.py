class Lane:

    def __init__(self, params):
        # params for [lane_vertex0, lane_vertex1, width]
        self.lane_vertices = params[0:2]
        self.width = params[2]
        self.vertices = []
        self.verticesSet = set()

    def setId(self, id):
        self.id = id

    def getId(self):
        return self.id

    def getWidth(self):
        return self.width

    def addPolygonVertices(self, id):
        if(id in self.verticesSet):
            return
        self.verticesSet.add(id)
        self.vertices.append(id)

    def getPolygonVertices(self):
        return list(self.vertices)

    def getLaneVertices(self):
        if(self.lane_vertices[0] == self.lane_vertices[1]):
            print("Lane: ", self.getId(), " has two same lane vertices: ",
             self.lane_vertices[0], " and ", self.lane_vertices[1])
        assert self.lane_vertices[0] != self.lane_vertices[1]
        return list(self.lane_vertices)


class LaneManager:
    def __init__(self):
        self.lanes = []

    def addLane(self, lane):
        id = len(self.lanes)
        lane.setId(id)
        self.lanes.append(lane)

    def getSize(self):
        return len(self.lanes)

    def getLane(self, id):
        assert(id < self.getSize())
        return self.lanes[id]