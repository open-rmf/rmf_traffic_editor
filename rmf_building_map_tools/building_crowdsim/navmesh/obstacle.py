class Obstacle:

    def __init__(self):
        self.id = -1
        self.v0_id = -1
        self.v1_id = -1
        self.node_id = -1
        self.neighbor_obst = -1

    def init_obstacle(self, v0_id, v1_id, node_id, neighbor_obst=-1):
        self.v0_id = v0_id
        self.v1_id = v1_id
        self.node_id = node_id
        self.neighbor_obst = neighbor_obst

    def get_variable(self):
        return [self.v0_id, self.v1_id, self.node_id, self.neighbor_obst]
