class ParamValue:
    UNDEFINED = 0
    STRING = 1
    INT = 2
    DOUBLE = 3
    BOOL = 4

    def __init__(self, yaml_value):
        self.type = yaml_value[0]
        self.value = yaml_value[1]

    def to_yaml(self):
        return [self.type, self.value]

    def __eq__(self, other):
        return self.type == other.type and self.value == other.value
