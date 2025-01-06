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
shared_parser.add_argument("--TEMPLATE_WORLD_FILE", type=str, default="",
                           help="Specify the template for"
                           + " the base simulation.")
shared_parser.add_argument("--SKIP_CAMERA_POSE", action="store_true",
                           help="Skips calculating and setting the initial "
                           + "camera view pose. This flag should only be used "
                           + "if the template SDF file already has the camera "
                           + "pose defined.")
# Create subparsers for Gazebo and Nav generation
gazebo_parser = subparsers.add_parser(
    'gazebo',
    help='Generate .world file and map model.sdf for Gazebo',
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

navgraph_visualization_parser = subparsers.add_parser(
    'navgraph_visualization',
    help='Generate SDF visualization of nav graph',
)

navgraph_visualization_parser.add_argument(
    "INPUT",
    type=str,
    help="Input building.yaml file to process")

navgraph_visualization_parser.add_argument(
    "OUTPUT_DIR",
    type=str,
    help="Path to output the navgraph visualization SDF models")
