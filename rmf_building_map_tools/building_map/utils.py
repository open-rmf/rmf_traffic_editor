import yaml
from xml.etree.ElementTree import ElementTree, Element, SubElement


def lift_material():
    material_ele = Element('material')
    ambient_ele = SubElement(material_ele, 'ambient')
    diffuse_ele = SubElement(material_ele, 'diffuse')
    specular_ele = SubElement(material_ele, 'specular')
    emissive_ele = SubElement(material_ele, 'emissive')
    # Might need to pick a better colour?
    ambient_ele.text = '0.5 0.5 0.5 1'
    diffuse_ele.text = '0.7 0.7 0.7 1'
    specular_ele.text = '0.6 0.6 0.6 1'
    emissive_ele.text = '0.1 0.1 0.1 1'
    return material_ele


def door_material(options):
    material_ele = Element('material')
    # blue-green glass as a default, so it's easy to see
    ambient_ele = SubElement(material_ele, 'ambient')
    ambient_ele.text = '{} {} {} {}'.format(0.5, 0.25, 0, 0.6)
    diffuse_ele = SubElement(material_ele, 'diffuse')
    diffuse_ele.text = '{} {} {} {}'.format(0.5, 0.25, 0, 0.6)
    if 'ignition' in options:
        pbr_ele = SubElement(material_ele, 'pbr')
        metal_ele = SubElement(pbr_ele, 'metal')
        metalness_ele = SubElement(metal_ele, 'metalness')
        metalness_ele.text = '0.0'
    return material_ele


def box(size):
    '''size: [x, y, z]'''
    [x, y, z] = size
    box_ele = Element('box')
    size_ele = SubElement(box_ele, 'size')
    size_ele.text = f'{x} {y} {z}'
    return box_ele


def collide_bitmask(bitmask):
    surface = Element('surface')
    contact = SubElement(surface, 'contact')
    collide_bitmask = SubElement(contact, 'collide_bitmask')
    collide_bitmask.text = f'{bitmask}'
    return surface


def visual(name, pose, size, material=None):
    visual_ele = Element('visual')
    visual_ele.set('name', name)
    if pose is not None:
        visual_ele.append(pose)

    visual_geometry_ele = SubElement(visual_ele, 'geometry')
    visual_geometry_ele.append(box(size))
    if material:
        visual_ele.append(material)

    return visual_ele


def collision(name, pose, size, bitmask=None):
    collision_ele = Element('collision')
    collision_ele.set('name', name)
    if pose is not None:
        collision_ele.append(pose)

    collision_geometry_ele = SubElement(collision_ele, 'geometry')
    collision_geometry_ele.append(box(size))
    if bitmask:
        collision_ele.append(collide_bitmask(bitmask))

    return collision_ele


def box_link(
    name,
    size,
    pose,
    with_visual=True,
    with_collision=True,
    material=None,
    bitmask=None
):
    link = Element('link')
    link.set('name', name)
    link.append(pose)
    if with_visual:
        link.append(visual(f'{name}_visual', None, size, material))
    if with_collision:
        link.append(collision(f'{name}_collision', None, size, bitmask))
    return link


def joint(
    joint_name,
    joint_type,
    parent_link,
    child_link,
    joint_axis=None,
    lower_limit=None,
    upper_limit=None,
    max_effort=None,
    pose=None
):
    joint = Element('joint')
    joint.set('name', joint_name)

    supported_joint_types = ['fixed', 'prismatic', 'revolute']
    if joint_type not in supported_joint_types:
        raise RuntimeError(f'joint type {joint_type} not supported.')
    joint.set('type', joint_type)

    parent = SubElement(joint, 'parent')
    parent.text = parent_link

    child = SubElement(joint, 'child')
    child.text = child_link

    if joint_type == 'fixed':
        return joint

    axis = SubElement(joint, 'axis')
    xyz = SubElement(axis, 'xyz')
    if joint_axis == 'x':
        xyz.text = '1 0 0'
    elif joint_axis == 'y':
        xyz.text = '0 1 0'
    elif joint_axis == 'z':
        xyz.text = '0 0 1'
    elif joint_axis == '-x':
        xyz.text = '-1 0 0'
    elif joint_axis == '-y':
        xyz.text = '0 -1 0'
    elif joint_axis == '-z':
        xyz.text = '0 0 -1'
    else:
        raise RuntimeError(
            'Axis requested is undefined, only "x", "-x", "y", "-y", '
            '"z" and "-"z" available')

    if lower_limit is not None and upper_limit is not None:
        limit = SubElement(axis, 'limit')
        lower = SubElement(limit, 'lower')
        lower.text = f'{lower_limit}'
        upper = SubElement(limit, 'upper')
        upper.text = f'{upper_limit}'

        if max_effort is not None:
            effort = SubElement(limit, 'effort')
            effort.text = f'{max_effort}'

    if pose is not None:
        joint.append(pose)

    return joint
