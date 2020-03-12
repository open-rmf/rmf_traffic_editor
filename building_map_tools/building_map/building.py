import os
from xml.etree.ElementTree import Element, SubElement, parse
from ament_index_python.packages import get_package_share_directory

from .level import Level


class Building:
    def __init__(self, yaml_node):
        if 'building_name' in yaml_node:
            self.name = yaml_node['building_name']
        else:
            self.name = yaml_node['name']
        print(f'building name: {self.name}')

        self.levels = {}
        for level_name, level_yaml in yaml_node['levels'].items():
            self.levels[level_name] = Level(level_yaml, level_name)

        if 'reference_level_name' in yaml_node:
            self.reference_level_name = yaml_node['reference_level_name']
        else:
            self.reference_level_name = next(iter(self.levels))
        self.ref_level = self.levels[self.reference_level_name]  # save typing

        self.calculate_level_offsets_and_scales()

    def __str__(self):
        s = ''
        for level_name, level in self.levels.items():
            s += f'{level_name}: ({len(level.vertices)} vertices) '
        return s

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
        for f_pair in fiducials:
            f0 = f_pair[0]
            f1 = f_pair[1]
            print(
                f'    ({f0.x:.5}, {f0.y:.5})'
                f' -> ({f1.x:.5}, {f1.y:.5})'
                f'   {f0.name}')

        # calculate the distances between each fiducial on their levels
        distances = []
        for f0_idx in range(0, len(fiducials)):
            for f1_idx in range(f0_idx + 1, len(fiducials)):
                f0_pair = fiducials[f0_idx]
                f1_pair = fiducials[f1_idx]
                print(f'    calc dist {f0_pair[0].name} <=> {f1_pair[0].name}')
                ref_dist = f0_pair[0].distance(f1_pair[0])
                target_dist = f0_pair[1].distance(f1_pair[1])
                distances.append((ref_dist, target_dist))
        print(distances)

        mean_rel_scale = 0.0
        for distance in distances:
            mean_rel_scale += distance[1] / distance[0]
        mean_rel_scale /= float(len(distances))
        print(f'mean relative scale: {mean_rel_scale}')
        level.scale = self.ref_level.scale / mean_rel_scale

        mean_translation = [0.0, 0.0]
        for f_pair in fiducials:
            mean_translation[0] += f_pair[1].x / mean_rel_scale - f_pair[0].x
            mean_translation[1] += f_pair[1].y / mean_rel_scale - f_pair[0].y

        if len(fiducials):
            mean_translation[0] *= self.ref_level.scale / float(len(fiducials))
            mean_translation[1] *= self.ref_level.scale / float(len(fiducials))
        print(
            f'translation: '
            f'({mean_translation[0]:.5}, '
            f'{mean_translation[1]:.5})')
        level.translation = mean_translation

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
        if 'gazebo' in options:
            template_name = 'gz_world.sdf'
        elif 'ignition' in options:
            template_name = 'ign_world.sdf'
        else:
            raise RuntimeError("expected either gazebo or ignition in options")

        template_path = os.path.join(
            get_package_share_directory('building_map_tools'),
            f'templates/{template_name}')
        tree = parse(template_path)
        sdf = tree.getroot()

        world = sdf.find('world')

        for level_name, level in self.levels.items():
            level.generate_sdf_models(world)  # todo: a better name
            level.generate_doors(world, options)

            level_include_ele = SubElement(world, 'include')
            level_model_name = f'{self.name}_{level_name}'
            name_ele = SubElement(level_include_ele, 'name')
            name_ele.text = level_model_name
            uri_ele = SubElement(level_include_ele, 'uri')
            uri_ele.text = f'model://{level_model_name}'
            pose_ele = SubElement(level_include_ele, 'pose')
            pose_ele.text = level.pose_string('flattened' in options)

        # add floor-toggle GUI plugin parameters
        if 'gazebo' in options:
            gui_ele = SubElement(world, 'gui')
            camera_ele = SubElement(gui_ele, 'camera', {'name': 'gui_camera'})
            pose_ele = SubElement(camera_ele, 'pose')
            c = self.center()
            pose_ele.text = f'{c[0]} {c[1]-20} 10 0 0.6 1.57'

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

        return sdf

    def generate_sdf_models(self, models_path):
        for level_name, level in self.levels.items():
            model_name = f'{self.name}_{level_name}'
            model_path = os.path.join(models_path, model_name)
            if not os.path.exists(model_path):
                os.makedirs(model_path)

            level.generate_sdf_model(model_name, model_path)

    def center(self):
        # todo: something smarter in the future. For now just the center
        # of the first level
        return self.levels[list(self.levels.keys())[0]].center()
