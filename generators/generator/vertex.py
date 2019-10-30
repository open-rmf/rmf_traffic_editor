class Vertex:
    def __init__(self, yaml_node):
        self.x = yaml_node[0]
        self.y = -yaml_node[1]
        self.z = yaml_node[2]  # currently always 0
        self.name = yaml_node[3]
