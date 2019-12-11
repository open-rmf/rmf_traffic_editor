from .door import Door

class DoubleSlidingDoor(Door):
    def __init__(self, door_edge):
        super().__init__(door_edge)
        print(f'DoubleSliding({self.name})')

    def generate(self, world_ele):
        print('DoubleSliding.generate()')

        self.generate_sliding_section(
            self.name + '_left',
            self.length/2,
            -self.length/6 - 0.01,
            (-self.length/2, 0.01))

        '''
        self.generate_sliding_section(
            self.name + '_right',
            self.length/2,
            self.length/4 + 0.01,
            (0.01, self.length/2))
        '''

        world_ele.append(self.model_ele)
