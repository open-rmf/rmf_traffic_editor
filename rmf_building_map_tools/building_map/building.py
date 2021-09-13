import math
import numpy as np
import os
import yaml
from xml.etree.ElementTree import Element, SubElement, parse
from ament_index_python.packages import get_package_share_directory

from .level import Level
from .lift import Lift
from .param_value import ParamValue
from .web_mercator_transform import WebMercatorTransform


class Building:
    def __init__(self, yaml_node):
        if 'building_name' in yaml_node:
            self.name = yaml_node['building_name']
        else:
            self.name = yaml_node['name']
        print(f'building name: {self.name}')

        self.params = {}
        if 'parameters' in yaml_node and yaml_node['parameters']:
            for param_name, param_yaml in yaml_node['parameters'].items():
                self.params[param_name] = ParamValue(param_yaml)
        print('parsed parameters' + str(self.params))

        if 'coordinate_system' in yaml_node:
            self.coordinate_system = yaml_node['coordinate_system']
        else:
            self.coordinate_system = 'reference_image'
        print(f'coordinate system: {self.coordinate_system}')

        if (self.coordinate_system == 'web_mercator' and
                'generate_crs' not in self.params):
            raise ValueError('generate_crs must be defined for global nav!')

        self.global_transform = None
        if self.coordinate_system == 'web_mercator':
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
                    y = -float(vertex[1])  # invert due to historical reasons
                    self.global_transform.set_offset(
                        self.global_transform.transform_point((x, y)))
                    break

        self.levels = {}
        self.model_counts = {}
        for level_name, level_yaml in yaml_node['levels'].items():
            self.levels[level_name] = Level(
                level_yaml,
                level_name,
                self.model_counts,
                self.global_transform)

        if 'reference_level_name' in yaml_node:
            self.reference_level_name = yaml_node['reference_level_name']
        else:
            self.reference_level_name = next(iter(self.levels))
        self.ref_level = self.levels[self.reference_level_name]  # save typing

        # we only need to calculate offsets/scales if we're in pixel space
        if self.coordinate_system == 'reference_image':
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
                    Lift(lift_yaml, lift_name, transform, self.levels)

        self.set_lift_vert_lists()

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
            if self.global_transform is not None:
                g['crs_name'] = self.global_transform.crs_name
                g['offset'] = [*self.global_transform.offset]

            empty = True
            for level_name, level in self.levels.items():
                level_graph = level.generate_nav_graph(i)
                g['levels'][level_name] = level_graph
                if level_graph['lanes']:
                    empty = False
            if not empty:
                nav_graphs[f'{i}'] = g
        return nav_graphs

    def generate_sdf_world(self, options):
        """ Return an etree of this Building in SDF starting from a template"""
        print(f'generator options: {options}')
        dae_export_plugin = False
        use_baked_assets = False
        if 'gazebo' in options:
            template_name = 'gz_world.sdf'
        elif 'ignition' in options:
            template_name = 'ign_world.sdf'
            if 'dae_export' in options:
                dae_export_plugin = True
            if 'baked_assets' in options:
                use_baked_assets = True
        else:
            raise RuntimeError("expected either gazebo or ignition in options")

        template_path = os.path.join(
            get_package_share_directory('rmf_building_map_tools'),
            f'templates/{template_name}')
        tree = parse(template_path)
        sdf = tree.getroot()

        world = sdf.find('world')

        if dae_export_plugin:
            world_export_plugin_ele = SubElement(
                world,
                'plugin',
                {
                    'name': 'ignition::gazebo::systems::ColladaWorldExporter',
                    'filename': 'ignition-gazebo-collada-world-exporter-system'
                })

        for level_name, level in self.levels.items():
            # todo: a better name
            if dae_export_plugin:
                level.generate_sdf_models(world, True, False)
            elif use_baked_assets:
                level.generate_sdf_models(world, False, True)
                # use the baked asset in our world file
                baked_include_ele = SubElement(world, 'include')
                name_ele = SubElement(baked_include_ele, 'name')
                name_ele.text = level_name
                uri_ele = SubElement(baked_include_ele, 'uri')
                uri_ele.text = f'model://{level_name}'
                pose_ele = SubElement(baked_include_ele, 'pose')
                pose_ele.text = f'0 0 {level.elevation} 0 0 0'
            else:
                level.generate_sdf_models(world, True, True)

            if dae_export_plugin is False:
                level.generate_doors(world, options)

            if use_baked_assets is False:
                level_include_ele = SubElement(world, 'include')
                level_model_name = f'{self.name}_{level_name}'
                name_ele = SubElement(level_include_ele, 'name')
                name_ele.text = level_model_name
                uri_ele = SubElement(level_include_ele, 'uri')
                uri_ele.text = f'model://{level_model_name}'
                pose_ele = SubElement(level_include_ele, 'pose')
                pose_ele.text = f'0 0 {level.elevation} 0 0 0'

        if dae_export_plugin is False:
            for lift_name, lift in self.lifts.items():
                if not lift.level_doors:
                    print(f'[{lift_name}] is not serving any floor, ignoring.')
                    continue
                lift.generate_shaft_doors(world)
                lift.generate_cabin(world, options)

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

        if self.global_transform is not None:
            offset = self.global_transform.offset
            offset_ele = SubElement(world, 'offset')
            offset_ele.text = f'{offset[0]} {offset[1]} 0 0 0 0'

            crs_ele = SubElement(world, 'crs')
            crs_ele.text = self.global_transform.crs_name

        gui_ele = world.find('gui')
        c = self.center()
        camera_pose = f'{c[0]} {c[1]-20} 10 0 0.6 1.57'
        # add floor-toggle GUI plugin parameters
        if 'gazebo' in options:
            camera_pose_ele = gui_ele.find('camera').find('pose')
            camera_pose_ele.text = camera_pose

            toggle_charge_ele = SubElement(
                gui_ele,
                'plugin',
                {'name': 'toggle_charging',
                 'filename': 'libtoggle_charging.so'})

            toggle_ele = SubElement(
                gui_ele,
                'plugin',
                {'name': 'toggle_floors', 'filename': 'libtoggle_floors.so'})

            for level_name, level in self.levels.items():
                floor_ele = SubElement(
                    toggle_ele,
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

        elif 'ignition' in options:
            plugin_ele = gui_ele.find('.//plugin[@filename="GzScene3D"]')
            camera_pose_ele = plugin_ele.find('camera_pose')
            camera_pose_ele.text = camera_pose

        return sdf

    def generate_sdf_world_for_dae_export(self, export_world_name, options):
        if 'gazebo' in options:
            template_name = 'gz_world.sdf'
        elif 'ignition' in options:
            template_name = 'ign_world.sdf'
        else:
            raise RuntimeError("expected either gazebo or ignition in options")

        template_path = os.path.join(
            get_package_share_directory('rmf_building_map_tools'),
            f'templates/{template_name}')
        tree = parse(template_path)
        sdf = tree.getroot()

        world_ele = sdf.find('world')

        world_export_plugin_ele = SubElement(
            world_ele,
            'plugin',
            {
                'name': 'ignition::gazebo::systems::ColladaWorldExporter',
                'filename': 'ignition-gazebo-collada-world-exporter-system'
            })

        for level_name, level in self.levels.items():
            for model in level.models:
                if model.lightmap == export_world_name:
                    model.generate(
                        world_ele,
                        level.transform,
                        level.elevation)

            level_include_ele = SubElement(world_ele, 'include')
            if export_world_name == '':
                level_model_name = f'{self.name}_{level_name}'
            else:
                level_model_name = f'{self.name}_{level_name}_{export_world_name}'
            name_ele = SubElement(level_include_ele, 'name')
            name_ele.text = level_model_name
            uri_ele = SubElement(level_include_ele, 'uri')
            uri_ele.text = f'model://{level_model_name}'
            pose_ele = SubElement(level_include_ele, 'pose')
            pose_ele.text = f'0 0 {level.elevation} 0 0 0'

        return sdf


    def generate_sdf_models(self, models_path, filter_world = ''):
        for level_name, level in self.levels.items():
            if filter_world != '':
                model_name = f'{self.name}_{level_name}_{filter_world}'
            else:
                model_name = f'{self.name}_{level_name}'
            model_path = os.path.join(models_path, model_name)
            if not os.path.exists(model_path):
                os.makedirs(model_path)

            level.generate_sdf_model(model_name, model_path, filter_world)

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

            d['levels'] = {}
            for level_name, level_data in self.levels.items():
                d['levels'][level_name] = level_data.to_yaml()

            d['lifts'] = {}
            for lift_name, lift in self.lifts.items():
                d['lifts'][lift_name] = lift.to_yaml()

            yaml.dump(d, f)
