class ConnectionManager:
    def __init__(self, lane_vertices_manager, lane_manager):
        self.lane_vertices_manager = lane_vertices_manager
        self.lane_manager = lane_manager

    def buildConnection(self):
        if(self.lane_manager.getSize() == 0):
            print("No lane defined!")
        assert(self.lane_manager.getSize() != 0)
        for lane_id in range(self.lane_manager.getSize()):
            lane = self.lane_manager.getLane(lane_id)
            v0_id = lane.getLaneVertices()[0]
            v1_id = lane.getLaneVertices()[1]
            assert(v0_id < self.lane_vertices_manager.getSize() 
                and v1_id < self.lane_vertices_manager.getSize()
                and v0_id >= 0
                and v1_id >= 0)
            
            v0 = self.lane_vertices_manager.getLaneVertex(v0_id)
            v1 = self.lane_vertices_manager.getLaneVertex(v1_id)
            v0.addLane(lane.getId())
            v1.addLane(lane.getId())