class Obstacle:

    def __init__(self):
        self.v0 = -1
        self.v1 = -1
        self.node = -1
        self.neighbor_obst = -1
    
    def setObstacle(self, v0, v1, node, neighbor_obst = -1):
        self.v0 = v0
        self.v1 = v1
        self.node = node
        self.neighbor_obst = neighbor_obst

    def setId(self, id):
        self.id = id

    def getId(self):
        return self.id

    def getVar(self):
        return [self.v0, self.v1, self.node, self.neighbor_obst]
    

class ObstacleManager:

    def __init__(self):
        self.obstacles = []

    def addObstacle(self, obstacle):
        id = len(self.obstacles)
        obstacle.setId(id)
        self.obstacles.append(obstacle)

    def getObstacle(self, id):
        return self.obstacles[id]

    def getSize(self):
        return len(self.obstacles)
