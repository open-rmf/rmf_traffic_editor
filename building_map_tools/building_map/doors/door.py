from xml.etree.ElementTree import Element, SubElement


class Door:
    def __init__(self, door_edge):
        self.name = door_edge.params['name'].value
        self.type = door_edge.params['type'].value
        self.length = door_edge.length
        self.cx = door_edge.x
        self.cy = door_edge.y
        self.yaw = door_edge.yaw
        self.height = 2.2  # parameterize someday?
        self.thickness = 0.03  # parameterize someday?
        print(f'Door({self.name})')

        self.model_ele = Element('model')
        self.model_ele.set('name', self.name)
        pose_ele = SubElement(self.model_ele, 'pose')
        pose_ele.text = f'{self.cx} {self.cy} 0 0 0 {self.yaw}'

    def generate_section(self, name, width, x_offset):
        link_ele = SubElement(self.model_ele, 'link')
        link_ele.set('name', name)
        pose_ele = SubElement(link_ele, 'pose')
        pose_ele.text = f'{x_offset} 0 {self.height/2+0.01} 0 0 0'

        visual_ele = SubElement(link_ele, 'visual')
        visual_ele.set('name', name)
        visual_ele.append(self.material())
        visual_geometry_ele = SubElement(visual_ele, 'geometry')
        visual_geometry_ele.append(
            self.box(width, self.thickness, self.height))

        collision_ele = SubElement(link_ele, 'collision')
        collision_ele.set('name', name)
        collision_ele.append(self.collide_bitmask())
        collision_geometry_ele = SubElement(collision_ele, 'geometry')
        collision_geometry_ele.append(
            self.box(width, self.thickness, self.height))

        mass = 50.0
        inertial_ele = SubElement(link_ele, 'inertial')
        mass_ele = SubElement(inertial_ele, 'mass')
        mass_ele.text = str(mass)
        inertia_ele = SubElement(inertial_ele, 'inertia')
        SubElement(inertia_ele, 'ixx').text = str(mass/12.0*(self.thickness**2 + self.height**2))
        SubElement(inertia_ele, 'iyy').text = str(mass/12.0*(width**2 + self.height**2))
        SubElement(inertia_ele, 'izz').text = str(mass/12.0*(self.thickness**2 + width**2))

        return link_ele

    def generate_sliding_section(self, name, width, x_offset, bounds):
        self.generate_section(name, width, x_offset)

        # now, the joint for this link
        joint_ele = SubElement(self.model_ele, 'joint')
        joint_ele.set('name', f'{name}_joint')
        joint_ele.set('type', 'prismatic')

        parent_ele = SubElement(joint_ele, 'parent')
        parent_ele.text = 'world'

        child_ele = SubElement(joint_ele, 'child')
        child_ele.text = name

        axis_ele = SubElement(joint_ele, 'axis')

        xyz_ele = SubElement(axis_ele, 'xyz')
        xyz_ele.text = '1 0 0'

        limit_ele = SubElement(axis_ele, 'limit')
        lower_ele = SubElement(limit_ele, 'lower')
        lower_ele.text = str(bounds[0])
        upper_ele = SubElement(limit_ele, 'upper')
        upper_ele.text = str(bounds[1])
        effort_ele = SubElement(limit_ele, 'effort')
        effort_ele.text = str(500.0)

    '''Generate a single swing section/panel of a door.

    name = name of the door section
    width = width of the door section
    x_offset = offset of the center of the door section from the center
               of the entire door (this will be non-zero for double doors)
    bounds = bounds for the range of motion of this section, in radians
    axis = pose of the joint axis, in the door *section* frame
    '''
    def generate_swing_section(self, name, width, x_offset, bounds, axis):
        self.generate_section(name, width, x_offset)

        # now, the joint for this link
        joint_ele = SubElement(self.model_ele, 'joint')
        joint_ele.set('name', f'{name}_joint')
        joint_ele.set('type', 'revolute')

        parent_ele = SubElement(joint_ele, 'parent')
        parent_ele.text = 'world'

        child_ele = SubElement(joint_ele, 'child')
        child_ele.text = name

        axis_ele = SubElement(joint_ele, 'axis')

        xyz_ele = SubElement(axis_ele, 'xyz')
        xyz_ele.text = '0 0 1'

        limit_ele = SubElement(axis_ele, 'limit')
        lower_ele = SubElement(limit_ele, 'lower')
        lower_ele.text = str(bounds[0])
        upper_ele = SubElement(limit_ele, 'upper')
        upper_ele.text = str(bounds[1])

        pose_ele = SubElement(joint_ele, 'pose')
        pose_ele.text = f'{axis[0]} {axis[1]} {axis[2]} 0 0 0'

    def collide_bitmask(self):
        surface_ele = Element('surface')
        contact_ele = SubElement(surface_ele, 'contact')
        collide_bitmask_ele = SubElement(contact_ele, 'collide_bitmask')
        collide_bitmask_ele.text = '0x02'
        return surface_ele

    def box(self, x, y, z):
        box_ele = Element('box')
        size_ele = SubElement(box_ele, 'size')
        size_ele.text = f'{x} {y} {z}'
        return box_ele

    def material(self):
        material_ele = Element('material')
        # blue-green glass as a default, so it's easy to see
        ambient_ele = SubElement(material_ele, 'ambient')
        ambient_ele.text = '{} {} {} {}'.format(120, 60, 0, 0.6)
        diffuse_ele = SubElement(material_ele, 'diffuse')
        diffuse_ele.text = '{} {} {} {}'.format(120, 60, 0, 0.6)
        return material_ele
