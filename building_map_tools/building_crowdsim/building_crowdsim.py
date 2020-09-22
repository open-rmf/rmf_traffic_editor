"""
Author: github.com/FloodShao

Description:
    "building_crowdsim" is a package for generating the required files for
    corwd simulation using Menge lib (https://github.com/MengeCrowdSim/Menge).
    The required files are:
    * navmesh.nav: defines the walkable area for agents
    * behavior.xml: defines the finite state machine (FSM) for the global path
    plan
    * scene.xml: defines the properties for each agent for local collision
    avoidance
    * <plugin>-tag: inserted in the world.sdf to transfer the simulation
    results between Menge and crowd_sim_plugin

    This package requires a building.yaml as input (see example in :
    building_crowdsim/test/config_test.yaml)

    * './navmesh' contains the source code generating "navmesh.nav" from
    "human_lanes" defined in the building.yaml
    * './config' contains the source code generating the required above .xml
    and <plugin> element from 'crowd_sim' key-word in building.yaml
    * "building_yaml_parse.py" is depended on "building_map" package, which
    parses the vertices coordinates and collects the human_goals

You can check Menge working details in
https://github.com/FloodShao/crowd_simulation/tree/master/crowd_simulation_doc
"""

import os
import sys

from .config.configfile_generator import configfile_main
from .navmesh.navmesh_generator import navmesh_main

from ._init_argparse import parser


def main():
    args = parser.parse_args()

    navmesh_main(
        args.YAML_INPUT,
        args.OUTPUT_DIR)
    configfile_main(
        args.YAML_INPUT,
        args.OUTPUT_DIR,
        args.WORLD_FILE)
