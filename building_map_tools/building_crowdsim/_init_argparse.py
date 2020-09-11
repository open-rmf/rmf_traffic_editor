import argparse
import os

# Init overall parser and subparser
parser = argparse.ArgumentParser(
    prog="building_crowdsim",
    description="Generate navmesh.nav, behavior.xml, scene.xml required for running crowd simulation,"
    "and generating the <plugin> tag in .world for gazebo and ignition-gazebo")
subparsers = parser.add_subparsers(help="Commands:", dest="command")

# Init shared parser
shared_parser = argparse.ArgumentParser(add_help=False)
shared_parser.add_argument("INPUT", type=str,
                           help="Input building.yaml file to process")
shared_parser.add_argument("OUTPUT_DIR", type=str,
                           help="Output directory for generated files")
shared_parser.add_argument("-o", "--options", type=str, nargs='*', default=[],
                           help="Generator options")

# Create subparsers for navmesh
navmesh_parser = subparsers.add_parser(
    'navmesh',
    help='Generate navmesh.nav for crowdsimulation',
    parents=[shared_parser]
)
navmesh_parser.add_argument(
    "OUT_PREFIX", type=str,
    nargs='?', default="",
    help="Specifying outputfile prefix")

# Create subparsers for configfile
configfile_parser = subparsers.add_parser(
    'configfile',
    help='Generate behavior.xml, scene.xml file and <plugin> tag for crowd simulation',
    parents=[shared_parser])
configfile_parser.add_argument(
    "WORLD_FILE_PROCESSED", type=str,
    help="World file to be inserted with crowd simulation <plugin>"
)
configfile_parser.add_argument(
    "PLATFORM", type=str,
    nargs='?', default='gazebo',
    help="Specifying the platform for running the world file, either gazebo or ignition-gazebo"
)
