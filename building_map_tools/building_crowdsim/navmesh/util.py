from collections import Iterable
import sys
import os

from .vector import Vector2d
from .vertex import Vertex, VertexManager
from .edge import Edge, EdgeManager
from .obstacle import Obstacle, ObstacleManager

def calIntersectVertexFromLaneVector(lane_vector, id0, id1, orign_point):
    # calculate the intersection vertex within the area from vector0 to vector1, mind the sequence
    # lane_vector has element (lane_instance, lane_vector)
    assert(id0 < len(lane_vector) and id1 < len(lane_vector) and id0 >= 0 and id1 >= 0)
    lane0_width = lane_vector[id0][0].getWidth()
    lane1_width = lane_vector[id1][0].getWidth()
    vector0 = lane_vector[id0][1]
    vector1 = lane_vector[id1][1]

    ## special case with 2 lanes are parallel, use the mid point as the intersection point
    # note that when 2 lanes are nearly parallel, not using the special case might cause rediculous result.
    if(abs( vector0.getUnit().getDot(vector1.getNormalUnit()) ) < 0.01) :
        length = (lane0_width + lane1_width) / 2
        result_x = length * vector0.getNormalUnit().x
        result_y = length * vector0.getNormalUnit().y
        if(vector0.getUnit().getCross(vector0.getNormalUnit()) < 0):
            result_x = -result_x
            result_y = -result_y
    else: 
        a0 = 0.5 * lane1_width / abs(vector0.getUnit().getDot(vector1.getNormalUnit()))
        a1 = 0.5 * lane0_width / abs(vector1.getUnit().getDot(vector0.getNormalUnit()))
        if(vector0.getUnit().getCross(vector1.getUnit()) < 0):
            a0 = -a0
            a1 = -a1
        result_x = a0 * vector0.getUnit().x + a1 * vector1.getUnit().x
        result_y = a0 * vector0.getUnit().y + a1 * vector1.getUnit().y
    
    result = Vertex([result_x + orign_point.x, result_y + orign_point.y])
    return result

def onSameSideOfLane(lane_vector, lane_vertex, vertex0, vertex1):
    vector0 = Vector2d()
    vector0.initWith2P(lane_vertex, vertex0)
    vector1 = Vector2d()
    vector1.initWith2P(lane_vertex, vertex1)

    # lane_vector can't be parallel with vector0 or vector1
    # assert(lane_vector.getCross(vector0) != 0)
    # assert(lane_vector.getCross(vector1) != 0)
    if(lane_vector.getCross(vector0) * lane_vector.getCross(vector1) > 0):
        return True
    else: 
        return False



class FileWriter:
    def __init__(self, filename):
        self.filename = filename

    def openFile(self, mode = "w+"):
        self.filehandle = open(self.filename, mode)

    def writeLine(self, content):
        if(self.filehandle):
            if isinstance(content, Iterable):
                for item in content:
                    item_str = str(item) + " "
                    self.filehandle.write(item_str)
            else:
                self.filehandle.write(str(content))
            self.filehandle.write("\n")
            self.filehandle.flush()

    def closeFile(self):
        self.filehandle.close()

    def setDataManager(self, vertexManager, edgeManager, obstacleManager, polygonManager):
        self.vertexManager = vertexManager
        self.edgeManager = edgeManager
        self.obstacleManager = obstacleManager
        self.polygonManager = polygonManager

    def generateNavMesh(self):
        # vertex part
        self.writeLine(self.vertexManager.getSize())
        for i in range(self.vertexManager.getSize()):
            v = self.vertexManager.getVertex(i)
            # print(v.getId(), v.getCoords())
            self.writeLine(v.getCoords())
        self.writeLine(" ")
        
        # edge part
        self.writeLine(self.edgeManager.getSize())
        for i in range(self.edgeManager.getSize()):
            e = self.edgeManager.getEdge(i)
            self.writeLine(e.getVar())
        self.writeLine(" ")

        # obstacle part
        self.writeLine(self.obstacleManager.getSize())
        for i in range(self.obstacleManager.getSize()):
            o = self.obstacleManager.getObstacle(i)
            self.writeLine(o.getVar())
        self.writeLine(" ")

        # polygon part
        # nodes name
        self.filehandle.write("walkable")
        self.filehandle.write("\n")
        self.writeLine(self.polygonManager.getSize())
        self.writeLine(" ")
        for i in range(self.polygonManager.getSize()):
            polygon = self.polygonManager.getPolygon(i)
            result = polygon.getVar()
            # print("polygon id: ", polygon.getId())
            # center
            self.writeLine(result[0])
            # vertices
            tmp = list(result[1])
            tmp.insert(0, len(result[1]))
            self.writeLine(tmp)
            # gradient
            self.writeLine(result[2])
            # edges
            if(len(result[3]) > 0) : 
                tmp = list(result[3])
                tmp.insert(0, len(result[3]))
                self.writeLine(tmp)
            else :
                self.writeLine(0)
            # obstalce
            if(len(result[4]) > 0) : 
                tmp = list(result[4])
                tmp.insert(0, len(result[4]))
                self.writeLine(tmp)
            else :
                self.writeLine(0)
            # blank
            self.writeLine(" ")

        if self.filename[0] == '/' :
            print("Generate: ", self.filename)
        else :
            print("Generate: ", os.getcwd() + '/' + self.filename)