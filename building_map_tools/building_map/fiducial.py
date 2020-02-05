class Fiducial:
    def __init__(self, yaml_node):
        self.x = yaml_node[0]
        self.y = -yaml_node[1]
        self.name = yaml_node[2]
