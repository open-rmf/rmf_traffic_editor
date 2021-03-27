class Edge:
    # edge for polygon

    def __init__(self):
        self.id = -1
        # polygon vertex id
        self.v0_id = -1
        self.v1_id = -1
        # connected polygon node id
        self.node0_id = -1
        self.node1_id = -1
        # the lane id this edge cross
        self.cross_lane_id = -1

    def init_edge(self, v0_id, v1_id, node0_id, node1_id):
        assert(v0_id != v1_id)
        self.v0_id = v0_id
        self.v1_id = v1_id
        self.node0_id = node0_id
        self.node1_id = node1_id

    def get_variable(self):
        return [self.v0_id, self.v1_id, self.node0_id, self.node1_id]
