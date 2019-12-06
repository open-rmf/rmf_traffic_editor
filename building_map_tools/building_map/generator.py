import math
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

    def generate_sdf(self, input_filename, output_filename, output_models_dir):
        print('generating {} from {}'.format(output_filename, input_filename))

        building = self.parse_editor_yaml(input_filename)

        if not os.path.exists(output_models_dir):
            os.makedirs(output_models_dir)
        building.generate_sdf_models(output_models_dir)
        
        # generate a top-level SDF for convenience
        sdf = building.generate_sdf_world()

        indent_etree(sdf)
        sdf_str = str(ElementToString(sdf), 'utf-8')
        with open(output_filename, 'w') as f:
            f.write(sdf_str)
        print(f'{len(sdf_str)} bytes written to {output_filename}')

    def generate_nav(self, input_filename, output_prefix):
        building = self.parse_editor_yaml(input_filename)

        nav_graphs = building.generate_nav_graphs()

        class CustomDumper(yaml.Dumper):
            def ignore_aliases(self, _):
                return True

        for graph_name, graph_data in nav_graphs.items():
            output_filename = f'{output_prefix}_{graph_name}.yaml'
            print(f'writing {output_filename}')
            with open(output_filename, 'w') as f:
                yaml.dump(
                    graph_data,
                    f,
                    default_flow_style=None,
                    Dumper=CustomDumper)
