class Edge:
    # edge for polygon
    
    def __init__(self):
        self.v0 = -1
        self.v1 = -1
        self.node0 = -1
        self.node1 = -1

    def setEdge(self, v0, v1, node0, node1):
        if(v0 == v1):
            print("Edge must have two different vertices!: ", v0, v1)
        assert(v0 != v1)
        self.v0 = v0
        self.v1 = v1
        self.node0 = node0
        self.node1 = node1
    
    def setId(self, id):
        self.id = id

    def getId(self):
        return self.id

    def getVar(self):
        return [self.v0, self.v1, self.node0, self.node1]
        
    def setLane(self, lane_id):
        self.lane_id = lane_id

    def getLane(self):
        return self.lane_id

class EdgeManager:
    def __init__(self):
        self.edges = []

    def getSize(self):
        return len(self.edges)

    def addEdge(self, edge):
        id = self.getSize()
        edge.setId(id)
        self.edges.append(edge)

    def getEdge(self, id):
        assert(id < self.getSize())
        return self.edges[id]