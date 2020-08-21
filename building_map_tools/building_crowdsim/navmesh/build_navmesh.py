from .lane import Lane, LaneManager
from .lane_vertex import LaneVertex, LaneVertexManager
from .polygon import Polygon, HubPolygon, LanePolygon, PolygonManager
from .vertex import Vertex, VertexManager
from .edge import Edge, EdgeManager
from .obstacle import Obstacle, ObstacleManager
from .util import FileWriter
from .connection import ConnectionManager
from .polygon_factory import PolygonFactory


class BuildNavmesh:
    def __init__(self):
        self.laneVertexManager = LaneVertexManager()
        self.laneManager = LaneManager()
        self.polygonManager = PolygonManager()
        self.vertexManager = VertexManager()
        self.edgeManager = EdgeManager()
        self.obstacleManager = ObstacleManager()
        self.polygonFactory = PolygonFactory( \
            self.polygonManager, \
            self.laneVertexManager, \
            self.laneManager, \
            self.vertexManager, \
            self.edgeManager, \
            self.obstacleManager)

    def AddLaneVertex(self, px, py):
        self.laneVertexManager.addLaneVertex( LaneVertex( [px, py] ) )

    def AddLane(self, idx0, idx1, width):
        self.laneManager.addLane( Lane( [idx0, idx1, width] ) )

    def Process(self):

        self.connectionManager = ConnectionManager(self.laneVertexManager, self.laneManager)
        self.connectionManager.buildConnection()

        ## add intersection vertices as node, only intersection node generates polygon vertices
        for lane_vertex_id in range(self.laneVertexManager.getSize()):
            lane_vertex = self.laneVertexManager.getLaneVertex(lane_vertex_id)
            # dead-end lane 
            if(len(lane_vertex.getLanes()) < 2): 
                continue

            polygon = HubPolygon()
            polygon.setHubVertexId( lane_vertex_id )
            polygon.addHubLanes(lane_vertex.getLanes())
            self.polygonFactory.hubPolygonUpdate(polygon)

        ## add lane as node
        for lane_id in range(self.laneManager.getSize()):
            polygon = LanePolygon()
            polygon.setLaneId(lane_id)
            self.polygonFactory.lanePolygonUpdate(polygon)

    def Output(self, output_file):

        ## Python write file
        navmesh_file = FileWriter(output_file)
        navmesh_file.setDataManager( \
            vertexManager=self.vertexManager, \
            edgeManager=self.edgeManager, \
            obstacleManager=self.obstacleManager, \
            polygonManager=self.polygonManager)

        navmesh_file.openFile()
        navmesh_file.generateNavMesh()
        navmesh_file.closeFile()


def test():
    buildNavmesh = BuildNavmesh()

    buildNavmesh.AddLaneVertex(18.83, -3.85)
    buildNavmesh.AddLaneVertex(18.79, -6.92)
    buildNavmesh.AddLaneVertex(18.81, -10.92)
    buildNavmesh.AddLaneVertex(15.22, -11.02)
    buildNavmesh.AddLaneVertex(11.47, -10.92)
    buildNavmesh.AddLaneVertex(11.37, -7.07)
    buildNavmesh.AddLaneVertex(15.06, -6.96)
    buildNavmesh.AddLaneVertex(10.02, -3.85)
    buildNavmesh.AddLaneVertex(9.07, -5.48)
    buildNavmesh.AddLaneVertex(7.05, -10.86)
    buildNavmesh.AddLaneVertex(6.88, -2.03)

    buildNavmesh.AddLane(0, 1, 1.0)
    buildNavmesh.AddLane(1, 2, 1.0)
    buildNavmesh.AddLane(2, 3, 1.0)
    buildNavmesh.AddLane(1, 6, 1.0)
    buildNavmesh.AddLane(3, 6, 1.0)
    buildNavmesh.AddLane(3, 4, 1.5)
    buildNavmesh.AddLane(6, 5, 1.0)
    buildNavmesh.AddLane(4, 5, 1.0)
    buildNavmesh.AddLane(4, 9, 0.8)
    buildNavmesh.AddLane(5, 8, 2.0)
    buildNavmesh.AddLane(8, 9, 1.5)
    buildNavmesh.AddLane(7, 8, 1.5)
    buildNavmesh.AddLane(0, 7, 1.5)
    buildNavmesh.AddLane(7, 10, 1.0)

    buildNavmesh.Process()
    buildNavmesh.Output("test_navmesh_00.nav")

if __name__ == "__main__":
    test()