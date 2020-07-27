import copy
import math
import os
import shutil
import numpy as np

from xml.etree.ElementTree import ElementTree, Element, SubElement
from .etree_utils import indent_etree

from ament_index_python.packages import get_package_share_directory
from .edge import Edge
from .fiducial import Fiducial
from .floor import Floor
from .hole import Hole
from .model import Model
from .transform import Transform
from .vertex import Vertex
from .doors.swing_door import SwingDoor
from .doors.sliding_door import SlidingDoor
from .doors.double_swing_door import DoubleSwingDoor
from .doors.double_sliding_door import DoubleSlidingDoor


class Level:
    def __init__(self, yaml_node, name):
        self.name = name
        print(f'parsing level {name}')

        self.drawing_name = None
        if 'drawing' in yaml_node:
            self.drawing_name = yaml_node['drawing']['filename']

        self.elevation = 0.0
        if 'elevation' in yaml_node:
            self.elevation = float(yaml_node['elevation'])

        self.fiducials = []
        if 'fiducials' in yaml_node:
            for fiducial_yaml in yaml_node['fiducials']:
                self.fiducials.append(Fiducial(fiducial_yaml))

        self.wall_height = 2.5  # meters
        self.wall_thickness = 0.1  # meters
        self.cap_thickness = 0.11  # meters
        self.cap_height = 0.02  # meters

        self.transform = Transform()

        self.vertices = []
        if 'vertices' in yaml_node and yaml_node['vertices']:
            for vertex_yaml in yaml_node['vertices']:
                self.vertices.append(Vertex(vertex_yaml))

        self.transformed_vertices = []  # will be calculated in a later pass

        self.lift_vert_lists = []  # will be calculated in a later pass

        self.meas = []
        if 'measurements' in yaml_node:
            self.meas = self.parse_edge_sequence(yaml_node['measurements'])
            for meas in self.meas:
                meas.calc_statistics(self.vertices)

        self.lanes = []
        if 'lanes' in yaml_node:
            self.lanes = self.parse_edge_sequence(yaml_node['lanes'])

        self.walls = []
        if 'walls' in yaml_node:
            self.walls = self.parse_edge_sequence(yaml_node['walls'])

        self.doors = []
        if 'doors' in yaml_node:
            self.doors = self.parse_edge_sequence(yaml_node['doors'])

        self.models = []
        model_counts = {}
        if 'models' in yaml_node:
            for model_yaml in yaml_node['models']:
                name = model_yaml["name"]
                if name not in model_counts:
                    model_counts[name] = 1
                    self.models.append(Model(name, model_yaml))
                else:
                    model_counts[name] += 1
                    self.models.append(
                            Model(f'{name}_{model_counts[name]}', model_yaml))

        self.floors = []
        if 'floors' in yaml_node:
            for floor_yaml in yaml_node['floors']:
                self.floors.append(Floor(floor_yaml))

        self.holes = []
        if 'holes' in yaml_node:
            for hole_yaml in yaml_node['holes']:
                self.holes.append(Hole(hole_yaml))

    def transform_all_vertices(self):
        self.transformed_vertices = []

        for untransformed_vertex in self.vertices:
            v = copy.deepcopy(untransformed_vertex)
            transformed = self.transform.transform_point(v.xy())
            v.x, v.y = transformed
            v.z = self.elevation
            self.transformed_vertices.append(v)

    def set_lift_vert_lists(self, lift_vert_lists):
        self.lift_vert_lists = lift_vert_lists

    def calculate_scale_using_measurements(self):
        # use the measurements to estimate scale for this level
        scale_cnt = 0
        scale_sum = 0
        for m in self.meas:
            scale_cnt += 1
            scale_sum += m.params['distance'].value / m.length
        if scale_cnt > 0:
            self.transform.set_scale(scale_sum / float(scale_cnt))
            print(f'level {self.name} scale: {self.transform.scale}')
        else:
            self.transform.set_scale(1.0)
            print('WARNING! No measurements defined. Scale is indetermined.')
            print('         Nav graph generated in pixel units, not meters!')

    def parse_edge_sequence(self, sequence_yaml):
        edges = []
        for edge_yaml in sequence_yaml:
            edges.append(Edge(edge_yaml))
        return edges

    def generate_wall_box_geometry(self, wall, parent_ele, item):
        x1, y1 = self.transformed_vertices[wall.start_idx]
        x2, y2 = self.transformed_vertices[wall.end_idx]

        dx = x1 - x2
        dy = y1 - y2
        length = math.sqrt(dx*dx + dy*dy) + self.wall_thickness
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        wall_yaw = math.atan2(dy, dx)

        if item == 'wall':
            box_thickness = self.wall_thickness
            box_height = self.wall_height
            cz = self.wall_height / 2.0
        elif item == 'cap':
            box_thickness = self.cap_thickness
            box_height = self.cap_height
            cz = self.wall_height
        else:
            raise RuntimeError('requested unknown wall item type  :(')

        geometry_ele = SubElement(parent_ele, 'geometry')

        box_ele = SubElement(geometry_ele, 'box')

        size_ele = SubElement(box_ele, 'size')
        size_ele.text = f'{length} {box_thickness} {box_height}'

        pose_ele = SubElement(parent_ele, 'pose')
        pose_ele.text = f'{cx} {cy} {cz} 0 0 {wall_yaw}'

    def generate_wall_visual_mesh(self, model_name, model_path):
        print(f'generate_wall_visual_mesh({model_name}, {model_path})')

        meshes_path = f'{model_path}/meshes'
        if not os.path.exists(meshes_path):
            os.makedirs(meshes_path)

        obj_model_rel_path = 'meshes/walls.obj'
        obj_path = os.path.join(model_path, obj_model_rel_path)
        print(f'  generating {obj_path}')
        with open(obj_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'mtllib wall.mtl\n')
            f.write(f'o walls\n')

            h = self.wall_height  # todo: allow per-segment height?

            # calculate faces for all the wall segments
            wall_cnt = 0
            wall_verts = np.array([])

            texture_lengths = [0]

            # normal #1 is the vertical normal for wall "caps"
            norms = np.array([[0, 0, 1]])

            for wall in self.walls:
                wall_cnt += 1

                wx1, wy1 = self.transformed_vertices[wall.start_idx].xy()
                wx2, wy2 = self.transformed_vertices[wall.end_idx].xy()

                wdx = wx2 - wx1
                wdy = wy2 - wy1
                wlen = math.sqrt(wdx*wdx + wdy*wdy)
                wcx = (wx1 + wx2) / 2.0
                wcy = (wy1 + wy2) / 2.0
                wyaw = math.atan2(wdy, wdx)

                # calculate the 4 corners of the wall footprint
                t2 = self.wall_thickness / 2.0

                wall_footprint_at_origin = np.array([
                    [-wlen / 2.0 - t2, t2],
                    [wlen / 2.0 + t2, t2],
                    [wlen / 2.0 + t2, -t2],
                    [-wlen / 2.0 - t2, -t2]])

                # now rotate the wall footprint
                rot = np.array([
                    [math.cos(wyaw), math.sin(wyaw)],
                    [-math.sin(wyaw), math.cos(wyaw)]])

                rot_verts = wall_footprint_at_origin.dot(rot)

                # finally, translate the wall segment vertices
                v = rot_verts + np.array([[wcx, wcy]])

                segment_norms_at_origin = np.array([
                    [0, 1],
                    [-1, 0],
                    [0, -1],
                    [1, 0]])
                segment_norms = segment_norms_at_origin.dot(rot)

                if not wall_verts.any():
                    wall_verts = v
                else:
                    wall_verts = np.vstack((wall_verts, v))

                # add z=0 to all segment norms
                norms = np.vstack((
                    norms,
                    np.hstack((segment_norms, np.zeros((4, 1))))
                ))

                # in the future we may have texture tiles of different scale,
                # but for now let's assume 1-meter x 1-meter tiles, so we don't
                # need to scale the texture coordinates currently.
                texture_lengths.append(wlen)

            for v in wall_verts:
                f.write(f'v {v[0]:.4f} {v[1]:.4f} 0.000\n')
                f.write(f'v {v[0]:.4f} {v[1]:.4f} {h:.4f}\n')

            for length in texture_lengths:
                f.write(f'vt {length:.4f} 0.000\n')
                f.write(f'vt {length:.4f} 1.000\n')

            for norm in norms:
                f.write(f'vn {norm[0]:.4f} {norm[1]:.4f} {norm[2]:.4f}\n')

            f.write('usemtl wall\n')
            f.write('s off\n')
            f.write('g walls\n')

            # finally we can wind the actual 8 face triangles
            for w in range(0, len(self.walls)):
                # first the side facing 'north' before rotation
                f.write(
                    f'f {w*8+1}/1/{w*4+2}'
                    f' {w*8+2}/2/{w*4+2}'
                    f' {w*8+3}/1/{w*4+2}\n')
                f.write(
                    f'f {w*8+4}/2/{w*4+2}'
                    f' {w*8+3}/1/{w*4+2}'
                    f' {w*8+2}/2/{w*4+2}\n')

                # now the 'east' side
                f.write(
                    f'f {w*8+3}/1/{w*4+3}'
                    f' {w*8+4}/2/{w*4+3}'
                    f' {w*8+5}/1/{w*4+3}\n')
                f.write(
                    f'f {w*8+6}/2/{w*4+3}'
                    f' {w*8+5}/1/{w*4+3}'
                    f' {w*8+4}/2/{w*4+3}\n')

                # now the 'south' side
                f.write(
                    f'f {w*8+5}/1/{w*4+4}'
                    f' {w*8+6}/2/{w*4+4}'
                    f' {w*8+7}/1/{w*4+4}\n')
                f.write(
                    f'f {w*8+8}/2/{w*4+4}'
                    f' {w*8+7}/1/{w*4+4}'
                    f' {w*8+6}/2/{w*4+4}\n')

                # now the 'west' side
                f.write(
                    f'f {w*8+7}/1/{w*4+5}'
                    f' {w*8+8}/2/{w*4+5}'
                    f' {w*8+1}/1/{w*4+5}\n')
                f.write(
                    f'f {w*8+2}/2/{w*4+5}'
                    f' {w*8+1}/1/{w*4+5}'
                    f' {w*8+8}/2/{w*4+5}\n')

                # now the top "cap" of this wall segment
                f.write(f'f {w*8+2}/1/1 {w*8+6}/1/1 {w*8+4}/1/1\n')
                f.write(f'f {w*8+2}/1/1 {w*8+8}/1/1 {w*8+6}/1/1\n')

        mtl_path = f'{meshes_path}/wall.mtl'
        print(f'  generating {mtl_path}')
        with open(mtl_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'newmtl wall\n')
            f.write('Ka 1.0 1.0 1.0\n')  # ambient
            f.write('Kd 1.0 1.0 1.0\n')  # diffuse
            f.write('Ke 0.0 0.0 0.0\n')  # emissive
            f.write('Ns 50.0\n')  # specular highlight, 0..100 (?)
            f.write('Ni 1.0\n')  # no idea what this is
            f.write('d 1.0\n')  # alpha (maybe?)
            f.write('illum 2\n')  # illumination model (enum)
            f.write(f'map_Kd wall.png\n')

        print(f'  copying wall textures into {meshes_path}')
        texture_path_source = os.path.join(
            get_package_share_directory('building_map_tools'),
            'textures/wall.png')
        texture_path_dest = f'{meshes_path}/wall.png'
        shutil.copyfile(texture_path_source, texture_path_dest)

    def generate_walls(self, model_ele, model_name, model_path):
        if not self.walls:
            return

        link_ele = SubElement(model_ele, 'link', {'name': 'walls'})
        self.generate_wall_visual_mesh(model_name, model_path)

        obj_path = f'model://{model_name}/meshes/walls.obj'

        visual_ele = SubElement(link_ele, 'visual')
        visual_ele.set('name', f'walls_{self.name}')

        v_geom_ele = SubElement(visual_ele, 'geometry')

        v_mesh_ele = SubElement(v_geom_ele, 'mesh')
        v_mesh_uri_ele = SubElement(v_mesh_ele, 'uri')
        v_mesh_uri_ele.text = obj_path

        collision_ele = SubElement(link_ele, 'collision')
        collision_ele.set('name', 'collision')

        c_geom_ele = SubElement(collision_ele, 'geometry')
        c_mesh_ele = SubElement(c_geom_ele, 'mesh')
        c_mesh_uri_ele = SubElement(c_mesh_ele, 'uri')
        c_mesh_uri_ele.text = obj_path

        c_surface_ele = SubElement(collision_ele, 'surface')
        c_contact_ele = SubElement(c_surface_ele, 'contact')
        c_bitmask_ele = SubElement(c_contact_ele, 'collide_bitmask')
        c_bitmask_ele.text = '0x01'

    def generate_sdf_models(self, world_ele):
        model_cnt = 0
        for model in self.models:
            model_cnt += 1
            model.generate(world_ele, model_cnt, self.transform)

        # sniff around in our vertices and spawn robots if requested
        for vertex_idx, vertex in enumerate(self.vertices):
            if 'spawn_robot_type' in vertex.params:
                self.generate_robot_at_vertex_idx(vertex_idx, world_ele)

    def generate_doors(self, world_ele, options):
        for door_edge in self.doors:
            door_edge.calc_statistics(self.transformed_vertices)
            self.generate_door(door_edge, world_ele, options)

    def generate_door(self, door_edge, world_ele, options):
        door_name = door_edge.params['name'].value
        door_type = door_edge.params['type'].value
        print(f'generate door name={door_name} type={door_type}')

        door = None
        if door_type == 'sliding':
            door = SlidingDoor(door_edge, self.elevation)
        elif door_type == 'hinged':
            door = SwingDoor(door_edge, self.elevation)
        elif door_type == 'double_sliding':
            door = DoubleSlidingDoor(door_edge, self.elevation)
        elif door_type == 'double_hinged':
            door = DoubleSwingDoor(door_edge, self.elevation)
        else:
            print(f'door type {door_type} not yet implemented')

        if door:
            door.generate(world_ele, options)

    def generate_robot_at_vertex_idx(self, vertex_idx, world_ele):
        vertex = self.transformed_vertices[vertex_idx]
        robot_type = vertex.params['spawn_robot_type'].value
        robot_name = vertex.params['spawn_robot_name'].value
        print(f'spawning robot name {robot_name} of type {robot_type}')

        yaw = 0
        # find the first vertex connected by a lane to this vertex
        for lane in self.lanes:
            if vertex_idx == lane.start_idx or vertex_idx == lane.end_idx:
                yaw = self.edge_heading(lane)
                if lane.orientation() == 'backward':
                    yaw += math.pi
                break

        include_ele = SubElement(world_ele, 'include')
        name_ele = SubElement(include_ele, 'name')
        name_ele.text = robot_name
        uri_ele = SubElement(include_ele, 'uri')
        uri_ele.text = f'model://{robot_type}'
        pose_ele = SubElement(include_ele, 'pose')
        pose_ele.text = f'{vertex.x} {vertex.y} {vertex.z} 0 0 {yaw}'

    def generate_floors(self, world_ele, model_name, model_path):
        i = 0
        for floor in self.floors:
            i += 1
            floor.generate(
                world_ele,
                i,
                model_name,
                model_path,
                self.transformed_vertices,
                self.holes,
                self.lift_vert_lists)

    def write_sdf(self, model_name, model_path):
        sdf_ele = Element('sdf', {'version': '1.6'})

        model_ele = SubElement(sdf_ele, 'model', {'name': model_name})

        static_ele = SubElement(model_ele, 'static')
        static_ele.text = 'true'

        self.generate_floors(model_ele, model_name, model_path)
        self.generate_walls(model_ele, model_name, model_path)

        sdf_tree = ElementTree(sdf_ele)
        indent_etree(sdf_ele)
        sdf_path = os.path.join(model_path, 'model.sdf')
        sdf_tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
        print(f'  wrote {sdf_path}')

    def generate_sdf_model(self, model_name, model_path):
        print(f'generating model of level {self.name} in {model_path}')
        config_fn = os.path.join(model_path, 'model.config')
        self.write_config(model_name, config_fn)
        print(f'  wrote {config_fn}')

        self.write_sdf(model_name, model_path)

    def write_config(self, model_name, path):
        config_ele = Element('model')

        name_ele = SubElement(config_ele, 'name')
        name_ele.text = model_name

        version_ele = SubElement(config_ele, 'version')
        version_ele.text = '1.0.0'

        sdf_ele = SubElement(config_ele, 'sdf', {'version': '1.6'})
        sdf_ele.text = 'model.sdf'

        author_ele = SubElement(config_ele, 'author')
        author_name_ele = SubElement(author_ele, 'name')
        author_name_ele.text = 'automatically generated from the Great Editor'
        author_email_ele = SubElement(author_ele, 'email')
        author_email_ele.text = 'info@openrobotics.org'

        description_ele = SubElement(config_ele, 'description')
        description_ele.text = f'level {model_name} (automatically generated)'

        config_tree = ElementTree(config_ele)
        indent_etree(config_ele)
        config_tree.write(path, encoding='utf-8', xml_declaration=True)

    def segments_intersect(self, v1, v2, v3, v4):
        x1 = v1.x
        y1 = v1.y
        x2 = v2.x
        y2 = v2.y
        x3 = v3.x
        y3 = v3.y
        x4 = v4.x
        y4 = v4.y
        # line segments are (x1,y1),(x2,y2) and (x3,y3),(x4,y4)
        det = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        if abs(det) < 0.01:
            # print('  determinant is {}. precision is no bueno.'.format(det))
            # print('    ({},{}),({},{}) and ({},{}),({},{})'.format(
            #     x1, y1, x2, y2, x3, y3, x4, y4))
            return False
        t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4)) / det
        u = -((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3)) / det
        # print('  t = {}  u = {}'.format(round(t,3), round(u,3)))
        if u < 0 or t < 0 or u > 1 or t > 1:
            return False
        print('hooray, we found an intersection: t={}, u={}'.format(
            round(t, 3), round(u, 3)))
        print('  ({},{}),({},{}) and ({},{}),({},{})'.format(
            x1, y1, x2, y2, x3, y3, x4, y4))
        return True

    def generate_nav_graph(self, graph_idx, always_unidirectional=True):
        """ Generate a graph without unnecessary (non-lane) vertices """
        # first remap the vertices. Store both directions; we'll need them
        next_idx = 0
        vidx_to_mapped_idx = {}
        mapped_idx_to_vidx = {}
        for l in self.lanes:
            if l.params['graph_idx'].value != graph_idx:
                continue
            if l.start_idx not in vidx_to_mapped_idx:
                vidx_to_mapped_idx[l.start_idx] = next_idx
                mapped_idx_to_vidx[next_idx] = l.start_idx
                next_idx += 1
            if l.end_idx not in vidx_to_mapped_idx:
                vidx_to_mapped_idx[l.end_idx] = next_idx
                mapped_idx_to_vidx[next_idx] = l.end_idx
                next_idx += 1
        # print(vidx_to_mapped_idx)
        # print(mapped_idx_to_vidx)

        # now output the mapped vertices (in order)
        nav_data = {}
        nav_data['vertices'] = []
        for i in range(0, next_idx):
            v = self.transformed_vertices[mapped_idx_to_vidx[i]]
            p = {'name': v.name}
            for param_name, param_value in v.params.items():
                p[param_name] = param_value.value
            nav_data['vertices'].append([v.x, v.y, p])

        nav_data['lanes'] = []
        for l in self.lanes:
            if l.params['graph_idx'].value != graph_idx:
                continue
            v1 = self.vertices[l.start_idx]
            v2 = self.vertices[l.end_idx]

            start_idx = vidx_to_mapped_idx[l.start_idx]
            end_idx = vidx_to_mapped_idx[l.end_idx]

            p = {}  # params

            for door in self.doors:
                door_v1 = self.vertices[door.start_idx]
                door_v2 = self.vertices[door.end_idx]
                door_name = door.params['name'].value
                if self.segments_intersect(v1, v2, door_v1, door_v2):
                    print(f'found intersection with door {door_name}!')
                    p['door_name'] = door_name

            if l.orientation():
                p['orientation_constraint'] = l.orientation()

            if 'demo_mock_floor_name' in l.params and \
                    l.params['demo_mock_floor_name'].value:
                p['demo_mock_floor_name'] = \
                    l.params['demo_mock_floor_name'].value

            if 'demo_mock_lift_name' in l.params and \
                    l.params['demo_mock_lift_name'].value:
                p['demo_mock_lift_name'] = \
                    l.params['demo_mock_lift_name'].value

            dock_name = None
            dock_at_end = True
            if 'dock_name' in v2.params:  # lane segment will end at dock
                dock_name = v2.params['dock_name'].value
            elif 'dock_name' in v1.params:
                dock_name = v1.params['dock_name'].value
                dock_at_end = False

            if always_unidirectional and l.is_bidirectional():
                # now flip things around and make the second link
                forward_params = copy.deepcopy(p)
                backward_params = copy.deepcopy(p)

                # we need to create two unidirectional lane segments
                # todo: clean up this logic, it's overly spaghetti
                if dock_name:
                    if dock_at_end:
                        forward_params['dock_name'] = dock_name
                    else:
                        forward_params['undock_name'] = dock_name
                nav_data['lanes'].append([start_idx, end_idx, forward_params])

                if dock_name:
                    if dock_at_end:
                        backward_params['undock_name'] = dock_name
                    else:
                        backward_params['dock_name'] = dock_name

                if l.orientation():
                    backward_params['orientation_constraint'] = \
                        l.reverse_orientation()
                nav_data['lanes'].append([end_idx, start_idx, backward_params])
            else:
                # ensure the directionality parameter is set
                p['is_bidirectional'] = l.is_bidirectional()
                if dock_name:
                    p['dock_name'] = dock_name
                nav_data['lanes'].append([start_idx, end_idx, p])

        return nav_data

    def edge_heading(self, edge):
        vs_x, vs_y = self.transformed_vertices[edge.start_idx].xy()
        ve_x, ve_y = self.transformed_vertices[edge.end_idx].xy()
        dx = ve_x - vs_x
        dy = ve_y - vs_y
        return math.atan2(dy, dx)

    def center(self):
        if not self.floors:
            return (0, 0)
        bounds = self.floors[0].polygon.bounds
        return ((bounds[0] + bounds[2]) / 2.0, (bounds[1] + bounds[3]) / 2.0)
