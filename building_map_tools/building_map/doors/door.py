class Door:
    def __init__(self, door_edge):
        self.door_edge = door_edge
        self.name = door_edge.params['name'].value
        self.height = 2.5  # parameterize someday?
        self.thickness = 0.03  # parameterize someday?
        print(f'Door({self.name})')

    def generate_sliding_section(self, world, name, width, y_offset, bounds):
        link_ele = SubElement(world, 'link')
        link.set('name', name)
        pose_ele = SubElement(link_ele, 'pose')
        pose_ele.text = f'0 {y_offset} {self.height/2} 0 0 0'

        visual_ele = SubElement(link_ele, 'visual')
        collision_ele = SubElement(link_ele, 'collision')
