import argparse

# Init overall parser and subparser
parser = argparse.ArgumentParser(
    prog="building_map_generator",
    description="Generate .world files, map models, and navigation maps"
                "for running in Gazebo or Ignition!"
)
subparsers = parser.add_subparsers(help="Commands:", dest="command")

# Init shared parser
sim_parser = argparse.ArgumentParser(add_help=False)
sim_parser.add_argument("INPUT", type=str,
                        help="Input building.yaml file to process")
sim_parser.add_argument("OUTPUT_WORLD", type=str,
                        help="Name of the .world file to output")
sim_parser.add_argument("OUTPUT_MODEL_DIR", type=str,
                        help="Path to output the map model files")
sim_parser.add_argument("-o", "--options", type=str, nargs='*', default=[],
                        help="Generator options")

# Create subparsers for Gazebo, Ignition, and Nav generation
gazebo_parser = subparsers.add_parser(
    'gazebo',
    help='Generate .world file and map model.sdf for Gazebo',
    parents=[sim_parser]
)
gazebo_parser.add_argument("-n", "--no_download", default=False,
                           const=True, action="store_const",
                           help="Do not download missing models from Fuel")
gazebo_parser.add_argument("-m", "--model_path", type=str,
                           help="Gazebo model path to check for models")
gazebo_parser.add_argument("-c", "--cache", type=str,
                           help="Path to pit_crew model cache")


ignition_parser = subparsers.add_parser(
    'ignition',
    help='Generate .world file and map model.sdf for Ignition',
    parents=[sim_parser]
)

nav_parser = subparsers.add_parser(
    'nav',
    help='Generate nav map .yaml file',
)
nav_parser.add_argument("INPUT", type=str,
                        help="Input building.yaml file to process")
nav_parser.add_argument("OUTPUT_DIR", type=str,
                        help="Path to output the nav .yaml files")
