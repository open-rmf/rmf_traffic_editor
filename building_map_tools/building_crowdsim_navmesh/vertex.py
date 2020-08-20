class Vertex:
    def __init__(self, coords):
        self.x = coords[0]
        self.y = coords[1]
        self.lanes = set()

    def setId(self, id):
        self.id = id
    
    def getId(self):
        return self.id

    def getCoords(self):
        return [self.x, self.y]

    def setLane(self, id):
        self.lanes.add(id)
    
    def getLane(self):
        if(len(self.lanes) == 0):
            print("Need to check lane for vertice! No lane is attached to this vertice!")
        return self.lanes

class VertexManager:
    def __init__(self):
        self.vertices = []

    def getSize(self):
        return len(self.vertices)

    def getVertex(self, id):
        assert(id < self.getSize())
        return self.vertices[id]

    def addVertex(self, vertex):
        id = self.getSize()
        vertex.setId(id)
        self.vertices.append(vertex)