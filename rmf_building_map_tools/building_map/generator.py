import os
import yaml
from xml.etree.ElementTree import tostring as ElementToString
from .building import Building
from .etree_utils import indent_etree


class Generator:
    def __init__(self):
        pass

    def parse_editor_yaml(self, input_filename):
        if not os.path.isfile(input_filename):
            raise FileNotFoundError(f'input file {input_filename} not found')

        with open(input_filename, 'r') as f:
            y = yaml.safe_load(f)
            return Building(y)

    # Remove namespaces in models
    def trim_model_namespaces(self, building):
        for level_name, level in building.levels.items():
            for model in level.models:
                if "/" in model.model_name:
                    model.model_name = \
                        "/".join(model.model_name.split("/")[1:])

    def generate_sdf(
        self,
        input_filename,
        output_filename,
        output_models_dir,
        options
    ):
        print('generating {} from {}'.format(output_filename, input_filename))

        building = self.parse_editor_yaml(input_filename)

        # Remove namespaces in models
        self.trim_model_namespaces(building)
        
        if not os.path.exists(output_models_dir):
            os.makedirs(output_models_dir)

        building.generate_sdf_models(output_models_dir)

        # generate a top-level SDF for convenience
        sdf = building.generate_sdf_world(options)

        indent_etree(sdf)
        sdf_str = str(ElementToString(sdf), 'utf-8')
        with open(output_filename, 'w') as f:
            f.write(sdf_str)
        print(f'{len(sdf_str)} bytes written to {output_filename}')

    def get_prebaked_worlds(self, building):
        all_prebaked_worlds = set()
        delimiter = ';'

        for level_name, level in building.levels.items():
            for floor in level.floors:
                if 'lightmap' in floor.params:
                    floor_lightmap = floor.params['lightmap']
                    splits = floor_lightmap.value.split(delimiter)
                    # print(floor_lightmap.value)
                    for split in splits:
                        all_prebaked_worlds.add(split)

            for wall in level.walls:
                if 'lightmap' in wall.params:
                    splits = wall.params['lightmap'].value.split(';')
                    for split in splits:
                        all_prebaked_worlds.add(split)

            for model in level.models:
                worlds_split = model.lightmap.split(delimiter)
                # print(lightmaps_split)
                for lightmap in worlds_split:
                    all_prebaked_worlds.add(lightmap)

        return all_prebaked_worlds

    def generate_baked_worlds(self,
        input_filename,
        output_worlds_dir,
        output_baked_file,
        output_models_dir
    ):
        building = self.parse_editor_yaml(input_filename)
        self.trim_model_namespaces(building)

        all_prebaked_worlds = self.get_prebaked_worlds(building)
        print(f'all_prebaked_worlds: {all_prebaked_worlds}')

        if not os.path.exists(output_models_dir):
            os.makedirs(output_models_dir)

        if not os.path.exists(output_worlds_dir):
            os.makedirs(output_worlds_dir)

        for prebaked_world_name in all_prebaked_worlds:
            if prebaked_world_name == '':
                export_world_file = output_worlds_dir + "/default.world"
            else:
                export_world_file = output_worlds_dir + "/" + prebaked_world_name + ".world"

            print(export_world_file)

            # output walls and floors specific to the lightmap
            filter_world = prebaked_world_name
            building.generate_sdf_models(output_models_dir, filter_world)

            # generate a top-level SDF for export
            sdf = building.generate_sdf_world_for_dae_export(prebaked_world_name, 'ignition')

            indent_etree(sdf)
            sdf_str = str(ElementToString(sdf), 'utf-8')
            with open(export_world_file, 'w') as f:
                f.write(sdf_str)
            print(f'{len(sdf_str)} bytes written to {export_world_file}')

        # generate top level sdf
        baked_sdf = building.generate_sdf_world(['ignition'] + ['baked_assets'],
            all_prebaked_worlds)

        indent_etree(baked_sdf)
        baked_sdf_str = str(ElementToString(baked_sdf), 'utf-8')
        with open(output_baked_file, 'w') as f:
            f.write(baked_sdf_str)
        print(f'{len(baked_sdf_str)} bytes written to {output_baked_file}')

    def generate_gazebo_sdf(
        self,
        input_filename,
        output_filename,
        output_models_dir,
        options
    ):
        self.generate_sdf(
            input_filename,
            output_filename,
            output_models_dir,
            options + ['gazebo'])

    def generate_ignition_sdf(
        self,
        input_filename,
        output_filename,
        output_models_dir,
        options
    ):
        self.generate_sdf(
            input_filename,
            output_filename,
            output_models_dir,
            options + ['ignition'])

    def generate_ignition_sdf_with_baked_worlds(
        self,
        input_filename,
        output_worlds_dir,
        output_baked_file,
        output_models_dir
    ):
        self.generate_baked_worlds(
            input_filename, output_worlds_dir, output_baked_file, output_models_dir)

    def generate_nav(self, input_filename, output_dir):
        building = self.parse_editor_yaml(input_filename)
        nav_graphs = building.generate_nav_graphs()

        class CustomDumper(yaml.Dumper):
            def ignore_aliases(self, _):
                return True

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        for graph_name, graph_data in nav_graphs.items():
            output_filename = os.path.join(output_dir, f'{graph_name}.yaml')
            print(f'writing {output_filename}')
            with open(output_filename, 'w') as f:
                yaml.dump(
                    graph_data,
                    f,
                    default_flow_style=None,
                    Dumper=CustomDumper)
