from .door import Door

class DoubleSlidingDoor(Door):
    def __init__(self, door_edge):
        super().__init__(door_edge)
        print(f'DoubleSliding({self.name})')

    def generate(self, world_ele):
        print('DoubleSliding.generate()')
        self.generate_sliding_section(
            world_ele, self.name + '_left', self.width/2,
            -self.width/4, (-self.width/2, 0))
