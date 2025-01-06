import fiona
import gzip
import json
import math
import numpy as np
import os
import sqlite3
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from pyproj import Transformer
from pyproj.crs import CRS
from xml.etree.ElementTree import Element, ElementTree, SubElement, parse

from .coordinate_system import CoordinateSystem
from .edge_type import EdgeType
from .etree_utils import indent_etree
from .geopackage import GeoPackage
from .level import Level
from .lift import Lift
from .material_utils import copy_texture
from .param_value import ParamValue
from .passthrough_transform import PassthroughTransform
from .vertex import Vertex
from .web_mercator_transform import WebMercatorTransform
from .wgs84_transform import WGS84Transform


class Building:
    def __init__(self, data, data_format='yaml'):
        if data_format == 'yaml':
            self.parse_yaml(data)
        elif data_format == 'geojson':
            self.parse_geojson(data)
        else:
            raise ValueError(f'unknown data format: {data_format}')

    def parse_yaml(self, yaml_node):
        if 'building_name' in yaml_node:
            self.name = yaml_node['building_name']
        else:
            self.name = yaml_node['name']
        print(f'building name: {self.name}')

        self.params = {}
        if 'parameters' in yaml_node and yaml_node['parameters']:
            for param_name, param_yaml in yaml_node['parameters'].items():
                self.params[param_name] = ParamValue(param_yaml)

        if 'map_version' in yaml_node:
            self.map_version = yaml_node['map_version']
        else:
            self.map_version = None

        cs_name = yaml_node.get('coordinate_system', 'reference_image')
        print(f'coordinate system: {cs_name}')
        self.coordinate_system = CoordinateSystem[cs_name]

        self.global_transform = None

        if self.coordinate_system == CoordinateSystem.reference_image:
            pass
        elif self.coordinate_system == CoordinateSystem.web_mercator:
            if 'generate_crs' not in self.params:
                raise ValueError('generate_crs must be defined for global nav')

            crs_name = self.params['generate_crs'].value
            self.global_transform = WebMercatorTransform(crs_name)

            origin_name = 'origin'
            if 'generate_origin_vertex' in self.params:
                origin_name = self.params['generate_origin_vertex'].value

            # this is weird, but we have to pre-parse the level vertices
            # in order to find the offset vertex :|
            origin_found = False
            for level_name, level_yaml in yaml_node['levels'].items():
                for vertex in level_yaml['vertices']:
                    if vertex[3] == origin_name:
                        origin_found = True
                        break
                if origin_found:
                    # transform the origin to the target frame
                    x = float(vertex[0])
                    # TODO: revisit this y inversion...
                    y = -float(vertex[1])  # invert due to historical reasons
                    self.global_transform.set_offset(
                        self.global_transform.transform_point((x, y)))
                    break
        elif self.coordinate_system == CoordinateSystem.cartesian_meters:
            if 'offset_x' in self.params:
                offset_x = self.params['offset_x'].value
            else:
                offset_x = 0

            if 'offset_y' in self.params:
                offset_y = self.params['offset_y'].value
            else:
                offset_y = 0

            if 'generate_crs' in self.params:
                crs_name = self.params['generate_crs'].value
            else:
                crs_name = ''

            self.global_transform = \
                PassthroughTransform(offset_x, offset_y, crs_name)
        elif self.coordinate_system == CoordinateSystem.wgs84:
            if 'generate_crs' not in self.params:
                # todo: automatically add a reasonable CRS in traffic-editor
                raise ValueError('generate_crs must be defined in wgs84 maps')

            crs_name = self.params['generate_crs'].value

            if 'suggested_offset_x' in self.params:
                offset_x = self.params['suggested_offset_x'].value
            else:
                offset_x = 0

            if 'suggested_offset_y' in self.params:
                offset_y = self.params['suggested_offset_y'].value
            else:
                offset_y = 0

            self.global_transform = \
                WGS84Transform(crs_name, (offset_x, offset_y))

        self.levels = {}
        self.model_counts = {}
        for level_name, level_yaml in yaml_node['levels'].items():
            self.levels[level_name] = Level(level_name)
            self.levels[level_name].parse_yaml(
                level_yaml,
                self.coordinate_system,
                self.model_counts,
                self.global_transform)

        if 'reference_level_name' in yaml_node:
            self.reference_level_name = yaml_node['reference_level_name']
        else:
            self.reference_level_name = next(iter(self.levels))
        self.ref_level = self.levels[self.reference_level_name]  # save typing

        # we only need to calculate offsets/scales if we're in pixel space
        if self.coordinate_system == CoordinateSystem.reference_image:
            self.calculate_level_offsets_and_scales()

        self.transform_all_vertices()

        self.lifts = {}
        if 'lifts' in yaml_node:
            for lift_name, lift_yaml in yaml_node['lifts'].items():
                if 'reference_floor_name' in lift_yaml:
                    ref_level_name = lift_yaml['reference_floor_name']
                    transform = self.levels[ref_level_name].transform
                else:
                    transform = self.ref_level.transform
                self.lifts[lift_name] = \
                    Lift(lift_yaml, lift_name, transform, self.levels,
                         self.coordinate_system)

        self.set_lift_vert_lists()

    def parse_geojson(self, json_node):
        self.levels = {}
        self.lifts = {}

        self.name = json_node.get('site_name', 'no_name')
        print(f'name: {self.name}')

        if 'features' not in json_node:
            return

        self.coordinate_system = CoordinateSystem.cartesian_meters

        if 'preferred_crs' not in json_node:
            # todo: calculate based on UTM grid
            print('CRS not specified. TODO: infer one.')
            return

        crs_name = json_node.get('preferred_crs', '')
        offset_x = json_node.get('suggested_offset_x', 0)
        offset_y = json_node.get('suggested_offset_y', 0)

        self.global_transform = \
            PassthroughTransform(offset_x, offset_y, crs_name)

        # project from WGS 84 to whatever is requested by this file
        transformer = Transformer.from_crs('EPSG:4326', crs_name)

        # Spin through all items and see how many levels we have.
        # todo: encode level polygons and names in GeoJSON files.
        # For now, just compute a bounding box and expand it a bit

        for feature in json_node['features']:
            if 'feature_type' not in feature:
                continue
            if feature['feature_type'] == 'rmf_vertex':
                self.parse_geojson_vertex(feature, transformer)

        for level_name in self.levels:
            self.levels[level_name].build_spatial_index()

        # now spin through and find the lanes, and assign them to vertices
        # using the rtree that was just built
        for feature in json_node['features']:
            if 'feature_type' not in feature:
                continue
            if feature['feature_type'] == 'rmf_lane':
                self.parse_geojson_lane(feature, transformer)

        self.transform_all_vertices()
        for level_name, level in self.levels.items():
            print(f'level {level_name}:')
            print(f'  bbox: {level.bbox}')
            print(f'  {len(level.vertices)} vertices')
            print(f'  {len(level.lanes)} lanes')

    def parse_geojson_lane(self, feature, transformer):
        if 'geometry' not in feature:
            return
        geometry = feature['geometry']
        if 'type' not in geometry:
            return
        if geometry['type'] != 'LineString':
            return
        if 'coordinates' not in geometry:
            return
        start_lon = geometry['coordinates'][0][0]
        start_lat = geometry['coordinates'][0][1]
        end_lon = geometry['coordinates'][1][0]
        end_lat = geometry['coordinates'][1][1]
        start_y, start_x = transformer.transform(start_lat, start_lon)
        end_y, end_x = transformer.transform(end_lat, end_lon)

        if 'properties' in feature:
            props = feature['properties']
            level_idx = props.get('level_idx', 0)

        # todo: look up the real level name somewhere
        level_name = f'level_{level_idx}'

        level = self.levels[level_name]
        level.add_edge_from_coords(
            EdgeType.LANE,
            (start_x, start_y),
            (end_x, end_y),
            props)

    def parse_geojson_vertex(self, feature, transformer):
        if 'geometry' not in feature:
            return

        geometry = feature['geometry']
        if 'type' not in geometry:
            return
        if geometry['type'] != 'Point':
            return
        if 'coordinates' not in geometry:
            return
        lon = geometry['coordinates'][0]
        lat = geometry['coordinates'][1]
        y, x = transformer.transform(lat, lon)

        level_idx = 0
        vertex_name = ''
        if 'properties' in feature:
            props = feature['properties']
            level_idx = props.get('level_idx', 0)
            vertex_name = props.get('name', '')

        # todo: look up the real level name somewhere
        level_name = f'level_{level_idx}'

        if level_name not in self.levels:
            level = Level(level_name)
            level.bbox = [[x, y], [x, y]]
            level.transform = self.global_transform
            self.levels[level_name] = level

        level = self.levels[level_name]

        level.bbox[0][0] = min(level.bbox[0][0], x)
        level.bbox[0][1] = min(level.bbox[0][1], y)
        level.bbox[1][0] = max(level.bbox[1][0], x)
        level.bbox[1][1] = max(level.bbox[1][1], y)

        # todo: parse all remaining params from json properties
        vertex_params = {}

        level.vertices.append(
            Vertex(
                [x, y, 0, vertex_name, vertex_params],
                self.coordinate_system
            )
        )

    def __str__(self):
        s = ''
        for level_name, level in self.levels.items():
            s += f'{level_name}: ({len(level.vertices)} vertices) '
        return s

    def set_lift_vert_lists(self):
        lift_vert_lists = {}
        for lift_name, lift in self.lifts.items():
            lift_vert_lists[lift_name] = lift.get_lift_vertices()

        for level_name, level in self.levels.items():
            level.set_lift_vert_lists(lift_vert_lists, self.lifts)

    def transform_all_vertices(self):
        """ Transform all vertices on all levels to a unified system """
        for level_name, level in self.levels.items():
            level.transform_all_vertices()

    def calculate_level_offsets_and_scales(self):
        # calculate all level offsets relative to reference_level_name
        print(f'calculating levels relative to {self.reference_level_name}')

        # first calculate scale of the reference level using its measurements
        self.ref_level.calculate_scale_using_measurements()

        # now calculate every other level's scale and offsets
        for level_name, level in self.levels.items():
            if level_name == self.reference_level_name:
                continue
            self.calculate_level_offset_and_scale(level_name)

    def calculate_level_offset_and_scale(self, level_name):
        print(f'calculating level {level_name} offset and scale...')
        level = self.levels[level_name]
        # find which fiducials are in common between ref_level and level
        fiducials = []
        for ref_fiducial in self.ref_level.fiducials:
            for fiducial in level.fiducials:
                if ref_fiducial.name == fiducial.name:
                    fiducials.append((ref_fiducial, fiducial))
        print(f'  {len(fiducials)} common fiducials:')
        level.transform.set_from_fiducials(
            fiducials,
            self.ref_level.transform.scale)

    def generate_nav_graphs(self):
        """ Returns a dict of all non-empty nav graphs """
        print("generating nav data")
        nav_graphs = {}
        # at the moment, graphs are numbered 0..9
        # iterate through and generate any non-empty graphs
        for i in range(0, 9):
            g = {}
            g['building_name'] = self.name
            g['levels'] = {}
            g['lifts'] = {}
            g['doors'] = {}

            if self.coordinate_system == CoordinateSystem.web_mercator:
                g['crs_name'] = self.global_transform.crs_name
                g['offset'] = [*self.global_transform.offset]
            elif self.coordinate_system == CoordinateSystem.cartesian_meters:
                if 'generate_crs' in self.params:
                    g['crs_name'] = self.params['generate_crs'].value
                tx, ty = self.global_transform.x, self.global_transform.y
                g['offset'] = [tx, ty]
            elif self.coordinate_system == CoordinateSystem.wgs84:
                g['crs_name'] = self.params['generate_crs'].value
                tx, ty = self.global_transform.x, self.global_transform.y
                g['offset'] = [tx, ty]

            empty = True
            for level_name, level in self.levels.items():
                level_graph = level.generate_nav_graph(i)
                g['levels'][level_name] = level_graph
                if level_graph['lanes']:
                    empty = False

                for door_edge in level.doors:
                    door_edge.calc_statistics(level.transformed_vertices)
                    g['doors'][door_edge.params['name'].value] = {
                        'endpoints': [
                            [door_edge.x1, door_edge.y1],
                            [door_edge.x2, door_edge.y2]
                        ],
                        'map': level_name
                    }

            for lift_name, lift in self.lifts.items():
                g['lifts'][lift_name] = {
                    'position': [lift.x, lift.y, lift.yaw],
                    'dims': [lift.width, lift.depth]
                }
            if not empty:
                nav_graphs[f'{i}'] = g
        return nav_graphs

    def generate_sdf_world(self, template_file, skip_camera_pose):
        """ Return an etree of this Building in SDF starting from a template"""
        if template_file == "":
            template_name = 'gz_world.sdf'
            template_path = os.path.join(
                get_package_share_directory('rmf_building_map_tools'),
                f'templates/{template_name}')
        else:
            template_path = template_file
        tree = parse(template_path)
        sdf = tree.getroot()

        world = sdf.find('world')

        for level_name, level in self.levels.items():
            level.generate_sdf_models(world)  # todo: a better name
            level.generate_doors(world)

            level_include_ele = SubElement(world, 'include')
            level_model_name = f'{self.name}_{level_name}'
            name_ele = SubElement(level_include_ele, 'name')
            name_ele.text = level_model_name
            uri_ele = SubElement(level_include_ele, 'uri')
            uri_ele.text = f'model://{level_model_name}'
            pose_ele = SubElement(level_include_ele, 'pose')
            if self.coordinate_system == CoordinateSystem.wgs84:
                tx = -self.global_transform.x
                ty = -self.global_transform.y
            else:
                tx = 0
                ty = 0
            pose_ele.text = f'{tx} {ty} {level.elevation} 0 0 0'

        for lift_name, lift in self.lifts.items():
            if not lift.level_doors:
                print(f'[{lift_name}] is not serving any floor, ignoring.')
                continue
            lift.generate_shaft_doors(world)
            lift.generate_cabin(world)

        charger_waypoints_ele = SubElement(
            world,
            'rmf_charger_waypoints',
            {'name': 'charger_waypoints'})

        for level_name, level in self.levels.items():
            for vertex in level.transformed_vertices:
                if 'is_charger' in vertex.params:
                    SubElement(
                        charger_waypoints_ele,
                        'rmf_vertex',
                        {'name': vertex.name, 'x': str(vertex.x),
                         'y': str(vertex.y), 'level': level_name})

        if self.coordinate_system == CoordinateSystem.web_mercator:
            (tx, ty) = self.global_transform.x, self.global_transform.y
            offset_ele = SubElement(world, 'offset')
            offset_ele.text = f'{tx} {ty} 0 0 0 0'

            crs_ele = SubElement(world, 'crs')
            crs_ele.text = self.global_transform.frame_name

        elif self.coordinate_system == CoordinateSystem.cartesian_meters:
            tx, ty = self.global_transform.x, self.global_transform.y
            offset_ele = SubElement(world, 'offset')
            offset_ele.text = f'{tx} {ty} 0 0 0 0'

            if self.global_transform.frame_name:
                crs_ele = SubElement(world, 'crs')
                crs_ele.text = self.global_transform.frame_name

        elif self.coordinate_system == CoordinateSystem.wgs84:
            tx = self.global_transform.x
            ty = self.global_transform.y
            offset_ele = SubElement(world, 'offset')
            offset_ele.text = f'{tx} {ty} 0 0 0 0'
            crs_ele = SubElement(world, 'crs')
            crs_ele.text = self.global_transform.crs_name

        gui_ele = world.find('gui')

        if not skip_camera_pose:
            c = self.center()
            # Transforming camera to account for offsets if
            # not in reference_image mode and when a floor polygon is defined.
            if self.global_transform and c != (0, 0):
                camera_pose = f'{c[0] - self.global_transform.x}  \
                {c[1]-20 - self.global_transform.y} 10 0 0.6 1.57'
            else:
                camera_pose = f'{c[0]} {c[1]-20} 10 0 0.6 1.57'
            # add floor-toggle GUI plugin parameters
            plugin_ele = gui_ele.find('.//plugin[@filename="MinimalScene"]')
            camera_pose_ele = plugin_ele.find('camera_pose')
            camera_pose_ele.text = camera_pose

        toggle_floors_ele = SubElement(
            gui_ele,
            'plugin',
            {'name': 'toggle_floors', 'filename': 'toggle_floors'})

        for level_name, level in self.levels.items():
            floor_ele = SubElement(
                toggle_floors_ele,
                'floor',
                {
                    'name': level_name,
                    'model_name': f'{self.name}_{level_name}'})

            for model in level.models:
                if model.static:
                    model_ele = SubElement(
                        floor_ele,
                        'model',
                        {'name': model.name})

            for door in level.doors:
                model_ele = SubElement(
                    floor_ele,
                    'model',
                    {'name': door.params['name'].value})

            for lift_name, lift in self.lifts.items():
                if level_name in lift.level_doors:
                    for door in lift.doors:
                        if door.name in lift.level_doors[level_name]:
                            model_ele = SubElement(
                                floor_ele,
                                'model',
                                {'name': (f'ShaftDoor_{lift_name}_' +
                                          f'{level_name}_{door.name}')})

        return sdf

    def generate_sdf_models(self, models_path):
        for level_name, level in self.levels.items():
            model_name = f'{self.name}_{level_name}'
            model_path = os.path.join(models_path, model_name)
            if not os.path.exists(model_path):
                os.makedirs(model_path)

            level.generate_sdf_model(model_name, model_path)

    def generate_navgraph_visualizations(self, output_dir):
        for i in range(0, 9):
            for level_name, level in self.levels.items():
                graph = level.generate_nav_graph(i)
                if not graph['lanes']:
                    continue

                self.generate_navgraph_visualization(
                    output_dir,
                    f'navgraph_{i}',
                    level,
                    graph)

    def generate_navgraph_visualization(self, output_dir, name, level, graph):
        print(f'generating {name}')
        meshes_path = f'{output_dir}/{name}/meshes'
        if not os.path.exists(meshes_path):
            os.makedirs(meshes_path)
        obj_path = f'{meshes_path}/{name}.obj'

        print(f'  generating {obj_path}')
        with open(obj_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'mtllib {name}.mtl\n')
            f.write(f'o walls\n')
            h = 0.1  # just above the floor
            thickness = 1.0  # meters
            for lane in graph['lanes']:
                v1 = graph['vertices'][lane[0]]
                v2 = graph['vertices'][lane[1]]
                v1x, v1y = v1[0], v1[1]
                v2x, v2y = v2[0], v2[1]

                dx = v2x - v1x
                dy = v2y - v1y
                length = math.sqrt(dx*dx + dy*dy)
                cx = (v1x + v2x) / 2.0
                cy = (v1y + v2y) / 2.0
                yaw = math.atan2(dy, dx)

                lane_footprint_at_origin = np.array([
                    [-length/2.0 - thickness / 2.0, thickness / 2.0],
                    [length/2.0 + thickness / 2.0, thickness / 2.0],
                    [length/2.0 + thickness / 2.0, -thickness / 2.0],
                    [-length/2.0 - thickness / 2.0, -thickness / 2.0]])

                rot = np.array([
                    [math.cos(yaw), math.sin(yaw)],
                    [-math.sin(yaw), math.cos(yaw)]])

                rot_verts = lane_footprint_at_origin.dot(rot)
                verts = rot_verts + np.array([[cx, cy]])

                for v in verts:
                    f.write(f'v {v[0]:.4f} {v[1]:.4f} {h:.4f}\n')

                f.write(f'vt 0.000 1.000\n')
                f.write(f'vt {length:.4f} 1.000\n')
                f.write(f'vt {length:.4f} 0.000\n')
                f.write(f'vt 0.000 0.000\n')

            f.write(f'vn 0.000 0.000 1.000\n')
            f.write('s off\n')
            f.write('g lanes\n')

            for i in range(0, len(graph['lanes'])):
                f.write(
                    f'f {i*4+1}/{i*4+1}/1'
                    f' {i*4+3}/{i*4+3}/1'
                    f' {i*4+2}/{i*4+2}/1\n')
                f.write(
                    f'f {i*4+1}/{i*4+1}/1'
                    f' {i*4+4}/{i*4+4}/1'
                    f' {i*4+3}/{i*4+3}/1\n')

        alpha = 0.5
        mtl_path = f'{meshes_path}/{name}.mtl'
        texture_filename = 'arrows.png'
        print(f'  generating {mtl_path}')
        with open(mtl_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'newmtl {name}\n')
            f.write('Ka 1.0 1.0 1.0\n')  # ambient
            f.write('Kd 1.0 1.0 1.0\n')  # diffuse
            f.write('Ke 0.0 0.0 0.0\n')  # emissive
            f.write('Ns 50.0\n')  # specular highlight, 0..100 (?)
            f.write('Ni 1.0\n')  # no idea what this is
            f.write(f'd {alpha}\n')  # alpha
            f.write('illum 2\n')  # illumination model (enum)
            f.write(f'map_Kd {texture_filename}\n')

        copy_texture('arrows', meshes_path)

        config_ele = Element('model')
        config_name_ele = SubElement(config_ele, 'name')
        config_name_ele.text = name
        config_version_ele = SubElement(config_ele, 'version')
        config_version_ele.text = '1.0.0'
        config_sdf_ele = SubElement(config_ele, 'sdf', {'version': '1.6'})
        config_sdf_ele.text = 'model.sdf'

        config_author_ele = SubElement(config_ele, 'author')
        config_author_name_ele = SubElement(config_author_ele, 'name')
        config_author_name_ele.text = 'generated by RMF Building Map Tools'
        config_author_email_ele = SubElement(config_author_ele, 'email')
        config_author_email_ele.text = 'info@openrobotics.org'

        config_description_ele = SubElement(config_ele, 'description')
        config_description_ele.text = f'{name} (generated)'

        config_tree = ElementTree(config_ele)
        indent_etree(config_ele)
        config_path = os.path.join(output_dir, name, 'model.config')
        config_tree.write(config_path, encoding='utf-8', xml_declaration=True)
        print(f'  wrote {config_path}')

        sdf_ele = Element('sdf', {'version': '1.7'})
        sdf_model_ele = SubElement(sdf_ele, 'model', {'name': name})
        sdf_static_ele = SubElement(sdf_model_ele, 'static')
        sdf_static_ele.text = 'true'
        sdf_link_ele = SubElement(sdf_model_ele, 'link', {'name': name})
        sdf_visual_ele = SubElement(sdf_link_ele, 'visual')
        sdf_visual_ele.set('name', name)
        sdf_visual_geom_ele = SubElement(sdf_visual_ele, 'geometry')
        sdf_transparency_ele = SubElement(sdf_visual_ele, 'transparency')
        sdf_transparency_ele.text = str(1.0 - alpha)
        sdf_mesh_ele = SubElement(sdf_visual_geom_ele, 'mesh')
        sdf_mesh_uri_ele = SubElement(sdf_mesh_ele, 'uri')
        sdf_mesh_uri_ele.text = os.path.join('meshes', f'{name}.obj')

        sdf_pose_ele = SubElement(sdf_link_ele, 'pose')
        sdf_link_x = 0
        sdf_link_y = 0
        sdf_pose_ele.text = f'{sdf_link_x} {sdf_link_y} 0 0 0 0'

        sdf_tree = ElementTree(sdf_ele)
        indent_etree(sdf_ele)
        sdf_path = os.path.join(output_dir, name, 'model.sdf')
        sdf_tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
        print(f'  wrote {sdf_path}')

    def center(self):
        # todo: something smarter in the future. For now just the center
        # of the first level
        return self.levels[list(self.levels.keys())[0]].center()

    def add_lanes_from(self, other_building):
        # go through each level and try to add lanes from the other building
        print(f'add_lanes_from()')
        print(f'our levels: {list(self.levels.keys())}')
        print(f'other levels: {list(other_building.levels.keys())}')
        for level_name, level in self.levels.items():
            if level_name in other_building.levels.keys():
                print(f'level {level_name} exists in both buildings')
                level.add_lanes_from(other_building.levels[level_name])
            else:
                print(f'WARNING: {level_name} does not exist in both')

    def write_yaml_file(self, filename):
        with open(filename, 'w') as f:
            d = {}
            d['name'] = self.name
            d['reference_level_name'] = self.reference_level_name
            d['coordinate_system'] = self.coordinate_system.name

            d['levels'] = {}
            for level_name, level_data in self.levels.items():
                d['levels'][level_name] = \
                    level_data.to_yaml(self.coordinate_system)

            d['lifts'] = {}
            for lift_name, lift in self.lifts.items():
                d['lifts'][lift_name] = lift.to_yaml(
                    self.coordinate_system)

            yaml.dump(d, f)

    def generate_geopackage(self):
        print('generating geopackage...')
        if self.coordinate_system != CoordinateSystem.cartesian_meters:
            print('Not a Cartesian map; not generating GeoPackage.')
            return []

        if 'generate_crs' not in self.params:
            print('Map does not have CRS defined; not generating GeoPackage.')
            return []

        with tempfile.TemporaryDirectory() as tempdirname:
            gpkg_filename = os.path.join(tempdirname, 'temp.gpkg')
            self.generate_geopackage_file(gpkg_filename)
            with open(gpkg_filename, 'rb') as f:
                b = f.read()

        print(f'GeoPackage is {len(b)} bytes')
        return b

    def generate_geopackage_file(self, gpkg_filename):
        print(f'generating GeoPackage in {gpkg_filename}')
        edge_schema = {
            'geometry': 'LineString',
            'properties': [
                ('level_idx', 'int'),
                ('edge_type', 'int'),
                ('parameters', 'str')
            ]
        }

        point_schema = {
            'geometry': 'Point',
            'properties': [
                ('name', 'str'),
                ('level_idx', 'int'),
                ('parameters', 'str')
            ]
        }

        level_schema = {
            'geometry': 'MultiPolygon',
            'properties': [
                ('name', 'str'),
                ('elevation', 'float'),
                ('parameters', 'str')
            ]
        }

        if 'generate_crs' not in self.params:
            print(f'cannot generate GeoPackage: map does not declare a CRS')
            return

        proj_crs = CRS(self.params['generate_crs'].value)
        fio_crs = proj_crs.to_wkt()

        level_idx_table = {}
        level_idx = 0
        for level_name, level in self.levels.items():
            level_idx_table[level_name] = level_idx
            level_idx += 1

        all_vertices = []
        all_edges = []
        for level_name, level in self.levels.items():
            level_idx = level_idx_table[level_name]
            for vertex in level.vertices:
                vertex_params = {}
                for param_name, param_value in vertex.params.items():
                    vertex_params[param_name] = param_value.value
                all_vertices.append({
                    'geometry': {
                        'type': 'Point',
                        'coordinates': [vertex.x, vertex.y],
                    },
                    'properties': {
                        'name': vertex.name,
                        'level_idx': level_idx,
                        'parameters': json.dumps(vertex_params),
                    }
                })
            for lane in level.lanes:
                lane_params = {}
                for param_name, param_value in lane.params.items():
                    lane_params[param_name] = param_value.value
                v1 = level.vertices[lane.start_idx]
                v2 = level.vertices[lane.end_idx]
                all_edges.append({
                    'geometry': {
                        'type': 'LineString',
                        'coordinates': [
                            [v1.x, v1.y],
                            [v2.x, v2.y],
                        ]
                    },
                    'properties': {
                        'level_idx': level_idx,
                        'edge_type': 1,  # todo: nice enum somewhere
                        'parameters': json.dumps(lane_params)
                    }
                })
            # todo: add measurement edges

        print(f'writing {len(all_vertices)} vertices...')
        with fiona.open(gpkg_filename,
                        'w',
                        layer='vertices',
                        driver='GPKG',
                        crs=fio_crs,
                        schema=point_schema) as collection:
            collection.writerecords(all_vertices)

        print(f'writing {len(all_edges)} edges...')
        with fiona.open(gpkg_filename,
                        'w',
                        layer='edges',
                        driver='GPKG',
                        crs=fio_crs,
                        schema=edge_schema) as collection:
            collection.writerecords(all_edges)

        metadata = {
            'name': self.name,
            'coordinate_system': self.coordinate_system.name,
        }
        for param_name, param_value in self.params.items():
            metadata[param_name] = param_value.value

        with GeoPackage(gpkg_filename) as gpkg:
            gpkg.set_metadata(json.dumps(metadata))

    def generate_geojson_file(self, filename, compress=False):
        j = self.generate_geojson()
        if j is None:
            return None

        if compress:
            data_str = json.dumps(j, indent=2, sort_keys=True)
            data_gzip = gzip.compress(bytes(data_str, 'utf-8'))
            with open(filename, 'wb') as f:
                f.write(data_gzip)
        else:
            with open(filename, 'w') as f:
                json.dump(j, f, indent=2, sort_keys=True)

        print(f'wrote {filename}')

    def generate_geojson(self):
        print(f'generating GeoJSON...')

        if 'generate_crs' not in self.params:
            print(f'cannot generate GeoJSON: map does not declare a CRS')
            return {}

        source_crs = self.params['generate_crs'].value
        wgs_transformer = Transformer.from_crs(source_crs, 'EPSG:4326')

        features = []

        # todo: re-order levels by elevation
        level_idx_table = {}
        level_idx = 0
        for level_name, level in self.levels.items():
            level_idx_table[level_name] = level_idx
            level_idx += 1

        all_vertices = []
        all_edges = []
        for level_name, level in self.levels.items():
            level_idx = level_idx_table[level_name]
            for vertex in level.vertices:
                (lat, lon) = wgs_transformer.transform(vertex.y, vertex.x)
                properties = {
                    'name': vertex.name,
                    'level_idx': level_idx,
                    'rmf_type': 'rmf_vertex'
                }
                for param_name, param_value in vertex.params.items():
                    properties[param_name] = param_value.value
                features.append({
                    'type': 'Feature',
                    'feature_type': 'rmf_vertex',
                    'geometry': {
                        'type': 'Point',
                        'coordinates': [lon, lat],
                    },
                    'properties': properties
                })

            for lane in level.lanes:
                properties = {
                    'level_idx': level_idx,
                    'rmf_type': 'rmf_lane'
                }
                for param_name, param_value in lane.params.items():
                    properties[param_name] = param_value.value
                v1 = level.vertices[lane.start_idx]
                v2 = level.vertices[lane.end_idx]
                (v1_lat, v1_lon) = wgs_transformer.transform(v1.y, v1.x)
                (v2_lat, v2_lon) = wgs_transformer.transform(v2.y, v2.x)

                if v1.name:
                    properties['start_vertex_name'] = v1.name

                if v2.name:
                    properties['end_vertex_name'] = v2.name

                features.append({
                    'type': 'Feature',
                    'feature_type': 'rmf_lane',
                    'geometry': {
                        'type': 'LineString',
                        'coordinates': [
                            [v1_lon, v1_lat],
                            [v2_lon, v2_lat],
                        ]
                    },
                    'properties': properties
                })

            # todo: add measurement edges
            # todo: add wall edges
            # todo: add door edges
            # todo: add lifts

        j = {
            'site_name': self.name,
            'preferred_crs': self.params['generate_crs'].value,
            'type': 'FeatureCollection',
            'features': features,
        }

        if self.map_version is not None:
            j['map_version'] = self.map_version

        if 'suggested_offset_x' in self.params:
            j['suggested_offset_x'] = self.params['suggested_offset_x'].value
        if 'suggested_offset_y' in self.params:
            j['suggested_offset_y'] = self.params['suggested_offset_y'].value

        print(f'generated {len(features)} features...')
        return j
