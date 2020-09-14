class Vertex:
    def __init__(self, coords):
        self.id = -1
        self.x = coords[0]
        self.y = coords[1]
        self.related_lane_ids = set()

    def get_coords(self):
        return [self.x, self.y]
