import argparse
import os

# Init overall parser and subparser
parser = argparse.ArgumentParser(
    prog="building_crowdsim",
    description="Generate navmesh.nav, behavior.xml, scene.xml" +
                "required for running crowd simulation," +
                "and generating the <plugin> tag in .world" +
                "for gazebo and ignition-gazebo")

# parser
parser = argparse.ArgumentParser(add_help=False)
parser.add_argument(
    "YAML_INPUT", type=str,
    help="Input building.yaml file to process")
parser.add_argument(
    "OUTPUT_DIR", type=str,
    help="Output directory for generated files")
parser.add_argument(
    "WORLD_FILE", type=str,
    help="World file to be inserted with crowd simulation <plugin>"
)
parser.add_argument(
    "-o", "--options", type=str, nargs='*', default=[],
    help="Generator options")
