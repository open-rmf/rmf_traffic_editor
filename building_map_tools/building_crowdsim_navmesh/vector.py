import math

class Vector2d:

    def __init__(self, coords = [0.0, 0.0]):
        self.x = coords[0]
        self.y = coords[1]

    def initWith2P(self, v0, v1):
        # pointing from v0 to v1
        self.x = v1.x - v0.x
        self.y = v1.y - v0.y

    def getLength(self):
        return math.sqrt(self.x**2 + self.y**2)

    def getUnit(self):
        norm = self.getLength()
        return Vector2d([self.x/norm, self.y/norm])

    def getNormalUnit(self):
        unit = self.getUnit()
        return Vector2d([-unit.y, unit.x])

    def getDot(self, vector2d):
        return self.x * vector2d.x + self.y * vector2d.y

    def getOrientation(self):
        return math.atan2(self.y, self.x)
    
    def getCross(self, vector2d):
        return self.x * vector2d.y - self.y * vector2d.x

if __name__ == "__main__":
    vector = Vector2d([1.0, 2.0])
    print("get length: ", vector.getLength())
    print("get unit: ", vector.getUnit().x, " ", vector.getUnit().y)