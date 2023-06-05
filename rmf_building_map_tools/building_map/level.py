import copy
import math
import os
import shutil
import numpy as np

from xml.etree.ElementTree import ElementTree, Element, SubElement
from .etree_utils import indent_etree
import rtree

from ament_index_python.packages import get_package_share_directory
from .edge import Edge
from .edge_type import EdgeType
from .fiducial import Fiducial
from .floor import Floor
from .wall import Wall
from .hole import Hole
from .model import Model
from .param_value import ParamValue
from .vertex import Vertex
from .doors.swing_door import SwingDoor
from .doors.sliding_door import SlidingDoor
from .doors.double_swing_door import DoubleSwingDoor
from .doors.double_sliding_door import DoubleSlidingDoor

from .transform import Transform


class Level:
    def __init__(self, name):
        self.name = name
        self.drawing_name = None
        self.elevation = 0.0
        self.fiducials = []
        self.vertices = []
        self.transformed_vertices = []  # will be calculated in a later pass
        self.lift_vert_lists = {}  # will be calculated in a later pass
        self.meas = []
        self.lanes = []
        self.walls = []
        self.doors = []
        self.models = []
        self.floors = []
        self.holes = []
        self.transform = Transform()
        self.vertices_index = None

    def parse_yaml(
        self,
        yaml_node,
        coordinate_system,
        model_counts={},
        transform=None
    ):
        print(f'parsing level {self.name}')

        self.drawing_name = None
        if 'drawing' in yaml_node:
            self.drawing_name = yaml_node['drawing']['filename']

        self.elevation = 0.0
        if 'elevation' in yaml_node:
            self.elevation = float(yaml_node['elevation'])

        self.fiducials = []
        if 'fiducials' in yaml_node:
            for fiducial_yaml in yaml_node['fiducials']:
                self.fiducials.append(
                    Fiducial(fiducial_yaml, coordinate_system))

        self.vertices = []
        if 'vertices' in yaml_node and yaml_node['vertices']:
            for vertex_yaml in yaml_node['vertices']:
                self.vertices.append(Vertex(vertex_yaml, coordinate_system))

        if transform is None:
            self.transform = Transform()
        else:
            self.transform = transform

        self.transformed_vertices = []  # will be calculated in a later pass

        self.lift_vert_lists = {}  # will be calculated in a later pass

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
        if 'models' in yaml_node:
            for model_yaml in yaml_node['models']:
                name = model_yaml["name"]
                if name not in model_counts:
                    model_counts[name] = 1
                    self.models.append(
                        Model(name, model_yaml, coordinate_system))
                else:
                    model_counts[name] += 1
                    self.models.append(
                        Model(
                            f'{name}_{model_counts[name]}',
                            model_yaml,
                            coordinate_system))

        self.floors = []
        if 'floors' in yaml_node:
            for floor_yaml in yaml_node['floors']:
                self.floors.append(Floor(floor_yaml))

        self.holes = []
        if 'holes' in yaml_node:
            for hole_yaml in yaml_node['holes']:
                self.holes.append(Hole(hole_yaml))

    def to_yaml(self, coordinate_system):
        y = {}
        if self.drawing_name:
            y['drawing'] = {}
            y['drawing']['filename'] = self.drawing_name
        y['elevation'] = self.elevation

        y['fiducials'] = [f.to_yaml(coordinate_system) for f in self.fiducials]
        y['vertices'] = [v.to_yaml(coordinate_system) for v in self.vertices]
        y['measurements'] = [e.to_yaml() for e in self.meas]
        y['lanes'] = [e.to_yaml() for e in self.lanes]
        y['walls'] = [e.to_yaml() for e in self.walls]
        y['doors'] = [e.to_yaml() for e in self.doors]
        y['models'] = [m.to_yaml(coordinate_system) for m in self.models]
        y['floors'] = [f.to_yaml() for f in self.floors]
        y['holes'] = [h.to_yaml() for h in self.holes]
        return y

    def transform_all_vertices(self):
        self.transformed_vertices = []

        for untransformed_vertex in self.vertices:
            v = copy.deepcopy(untransformed_vertex)
            transformed = self.transform.transform_point(v.xy())
            v.x, v.y = transformed
            v.z = self.elevation
            self.transformed_vertices.append(v)

    def set_lift_vert_lists(self, lift_vert_lists, lifts):
        for lift_name, lift in lifts.items():
            if lift.level_doors and \
                    self.elevation >= lift.lowest_elevation and \
                    self.elevation <= lift.highest_elevation:
                self.lift_vert_lists[lift_name] = \
                    (lift_vert_lists[lift_name])

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
            edge = Edge()
            edge.parse_yaml(edge_yaml)
            edges.append(edge)
        return edges

    def nearest_vertex_index(self, p):
        return next(self.vertices_index.nearest((p[0], p[1], p[0], p[1]), 1))

    def build_spatial_index(self):
        self.vertices_index = rtree.index.Index()
        for i, v in enumerate(self.vertices):
            self.vertices_index.insert(i, (v.x, v.y, v.x, v.y))

    def add_edge_from_coords(self, edge_type, p1, p2, props):
        edge = Edge()
        edge.start_idx = self.nearest_vertex_index(p1)
        edge.end_idx = self.nearest_vertex_index(p2)

        if edge_type == EdgeType.LANE:
            edge.params['graph_idx'] = ParamValue([
                ParamValue.INT,
                props.get('graph_idx', 0)
            ])
            edge.params['is_bidirectional'] = ParamValue([
                ParamValue.BOOL,
                props.get('is_bidirectional', False)
            ])
            edge.params['speed_limit'] = ParamValue([
                ParamValue.DOUBLE,
                props.get('speed_limit', 0.0)
            ])
            self.lanes.append(edge)
        else:
            raise ValueError('unknown edge type!')

    def generate_walls(self, model_ele, model_name, model_path):
        wall_params_list = []
        # crude method to identify all unique params list in walls
        for wall in self.walls:
            # check if param exists, if not use default val
            if "texture_name" not in wall.params:
                wall.params["texture_name"] = ParamValue(
                        [ParamValue.STRING, 'default'])
            if "alpha" not in wall.params:
                wall.params["alpha"] = ParamValue([ParamValue.DOUBLE, 1.0])
            if wall.params not in wall_params_list:
                wall_params_list.append(wall.params)
        print(f'Walls Generation, wall params list: {wall_params_list}')

        wall_cnt = 0
        for wall_params in wall_params_list:
            wall_cnt += 1
            single_texture_walls = Wall(self.walls, wall_params)
            single_texture_walls.generate(
                model_ele,
                wall_cnt,
                model_name,
                model_path,
                self.transformed_vertices)

    def generate_sdf_models(self, world_ele):
        for model in self.models:
            model.generate(
                world_ele,
                self.elevation,
                self.transform)

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

        '''
        Because we may be translating the world significantly in order for
        the simulation coordinates to be near the origin, in the case of
        Georeferenced frames, we also need to translate the robot spawn
        point. Otherwise our poor robots could be spawned thousands of
        kilometers from the building, thereby falling into the abyss of -NaN
        '''
        robot_x = vertex.x - self.transform.x
        robot_y = vertex.y - self.transform.y

        include_ele = SubElement(world_ele, 'include')
        name_ele = SubElement(include_ele, 'name')
        name_ele.text = robot_name
        uri_ele = SubElement(include_ele, 'uri')
        uri_ele.text = f'model://{robot_type}'
        pose_ele = SubElement(include_ele, 'pose')
        pose_ele.text = f'{robot_x} {robot_y} {vertex.z} 0 0 {yaw}'

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
            if floor.has_ceiling():
                floor.generate_ceiling(
                    world_ele,
                    i,
                    model_name,
                    model_path,
                    self.transformed_vertices,
                    self.holes,
                    self.lift_vert_lists)

    def write_sdf(self, model_name, model_path):
        sdf_ele = Element('sdf', {'version': '1.7'})

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

    def is_in_lift(self, p, lift_vert_list):
        verts = np.array(lift_vert_list)
        # array of vectors from the point to four rectangle vertices
        a = verts - np.array(p)
        # array of vectors for the four sides of the rectangle
        b = []
        for i in range(4):
            b.append(verts[i-1] - verts[i])
        # cross products of the four pairs of vectors. If the four cross
        # products have the same sign, then the point is inside the rectangle
        cross = np.cross(a, np.array(b))
        if np.all(cross >= 0) or np.all(cross <= 0):
            return True
        else:
            return False

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
            for lift_name, lift_vert_list in self.lift_vert_lists.items():
                if self.is_in_lift([v.x, v.y], lift_vert_list):
                    p['lift'] = lift_name
                    break
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

            if 'speed_limit' in l.params and \
                    l.params['speed_limit'].value > 0.0:
                p['speed_limit'] = l.params['speed_limit'].value

            if 'demo_mock_floor_name' in l.params and \
                    l.params['demo_mock_floor_name'].value:
                p['demo_mock_floor_name'] = \
                    l.params['demo_mock_floor_name'].value

            if 'demo_mock_lift_name' in l.params and \
                    l.params['demo_mock_lift_name'].value:
                p['demo_mock_lift_name'] = \
                    l.params['demo_mock_lift_name'].value

            beginning_dock = None
            end_dock = None
            if 'dock_name' in v2.params:  # lane segment will end at dock
                end_dock = v2.params['dock_name'].value
            if 'dock_name' in v1.params:  # lane segment begins at dock
                beginning_dock = v1.params['dock_name'].value

            if 'speed_limit' in l.params:
                p['speed_limit'] = l.params['speed_limit'].value

            if always_unidirectional and l.is_bidirectional():
                # now flip things around and make the second link
                forward_params = copy.deepcopy(p)
                backward_params = copy.deepcopy(p)

                # we need to create two unidirectional lane segments
                if end_dock:
                    forward_params['dock_name'] = end_dock
                    backward_params['undock_name'] = end_dock
                if beginning_dock:
                    forward_params['undock_name'] = beginning_dock
                    backward_params['dock_name'] = beginning_dock

                nav_data['lanes'].append([start_idx, end_idx, forward_params])

                if l.orientation():
                    backward_params['orientation_constraint'] = \
                        l.reverse_orientation()
                nav_data['lanes'].append([end_idx, start_idx, backward_params])
            else:
                # ensure the directionality parameter is set
                p['is_bidirectional'] = l.is_bidirectional()
                if end_dock:
                    p['dock_name'] = end_dock
                if beginning_dock:
                    p['undock_name'] = beginning_dock
                nav_data['lanes'].append([start_idx, end_idx, p])

        return nav_data

    def edge_heading(self, edge):
        vs_x, vs_y = self.transformed_vertices[edge.start_idx].xy()
        ve_x, ve_y = self.transformed_vertices[edge.end_idx].xy()
        dx = ve_x - vs_x
        dy = ve_y - vs_y
        return math.atan2(dy, dx)

    def center(self):
        if not self.floors or self.floors[0].polygon is None:
            return (0, 0)
        bounds = self.floors[0].polygon.bounds
        return ((bounds[0] + bounds[2]) / 2.0, (bounds[1] + bounds[3]) / 2.0)

    def add_lanes_from(self, other):
        print(f'add_lanes_from()')
        print(f'  this level has {len(self.vertices)} vertices')
        print(f'  other level has {len(other.vertices)} vertices')

        fiducial_pairs = []
        for ref_fiducial in self.fiducials:
            for fiducial in other.fiducials:
                if ref_fiducial.name == fiducial.name:
                    fiducial_pairs.append((ref_fiducial, fiducial))
        t = Transform()
        t.set_from_fiducials(fiducial_pairs, 1.0)

        v_idx_offset = len(self.vertices)

        # transform the incoming vertices
        for v_in in other.vertices:
            v_out = copy.deepcopy(v_in)
            (v_out.x, v_out.y) = t.transform_point([v_in.x, v_in.y])
            self.vertices.append(v_out)

        for lane_in in other.lanes:
            lane_out = copy.deepcopy(lane_in)
            lane_out.start_idx += v_idx_offset
            lane_out.end_idx += v_idx_offset
            self.lanes.append(lane_out)

    def generate_wall_graph(self):
        """ Genereate a wall graph without unnecessary (non-wall) vertices"""

        # first remap the vertices. Store both directions; we'll need them
        next_idx = 0
        vidx_to_mapped_idx = {}
        mapped_idx_to_vidx = {}

        for w in self.walls:
            if w.start_idx not in vidx_to_mapped_idx:
                vidx_to_mapped_idx[w.start_idx] = next_idx
                mapped_idx_to_vidx[next_idx] = w.start_idx
                next_idx += 1
            if w.end_idx not in vidx_to_mapped_idx:
                vidx_to_mapped_idx[w.end_idx] = next_idx
                mapped_idx_to_vidx[next_idx] = w.end_idx
                next_idx += 1

        wall_data = {}
        wall_data['vertices'] = []

        for i in range(next_idx):
            v = self.transformed_vertices[mapped_idx_to_vidx[i]]
            p = {'name': v.name}

            for param_name, param_value in v.params.items():
                p[param_name] = param_value.value

            wall_data['vertices'].append([v.x, v.y, p])

        wall_data['walls'] = []

        for w in self.walls:
            v1 = self.vertices[w.start_idx]
            v2 = self.vertices[w.end_idx]

            start_idx = vidx_to_mapped_idx[w.start_idx]
            end_idx = vidx_to_mapped_idx[w.end_idx]

            wall_data['walls'].append([start_idx, end_idx, w.params])

        return wall_data
