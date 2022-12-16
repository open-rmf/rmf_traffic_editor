class PassthroughTransform:
    """This class stores a 2D translation, but doesn't apply it."""

    def __init__(self, x, y, frame_name):
        self.x = x
        self.y = y
        self.rotation = 0
        self.frame_name = frame_name

    def transform_point(self, p):
        return p
