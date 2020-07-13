import argparse

# Init overall parser and subparser
parser = argparse.ArgumentParser(
    prog="building_map_generator",
    description="Generate .world files, map models, and navigation maps"
                "for running in Gazebo or Ignition!"
)
subparsers = parser.add_subparsers(help="Commands:", dest="command")

# Init shared parser
shared_parser = argparse.ArgumentParser(add_help=False)
shared_parser.add_argument("INPUT", type=str,
                           help="Input building.yaml file to process")
shared_parser.add_argument("OUTPUT_WORLD", type=str,
                           help="Name of the .world file to output")
shared_parser.add_argument("OUTPUT_MODEL_DIR", type=str,
                           help="Path to output the map model files")
shared_parser.add_argument("-o", "--options", type=str, nargs='*', default=[],
                           help="Generator options")

# Create subparsers for Gazebo, Ignition, and Nav generation
gazebo_parser = subparsers.add_parser(
    'gazebo',
    help='Generate .world file and map model.sdf for Gazebo',
    parents=[shared_parser]
)

ignition_parser = subparsers.add_parser(
    'ignition',
    help='Generate .world file and map model.sdf for Ignition',
    parents=[shared_parser]
)

nav_parser = subparsers.add_parser(
    'nav',
    help='Generate nav map .yaml file',
)
nav_parser.add_argument("INPUT", type=str,
                        help="Input building.yaml file to process")
nav_parser.add_argument("OUTPUT_DIR", type=str,
                        help="Path to output the nav .yaml files")
