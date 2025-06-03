#!/usr/bin/env python3

import pit_crew

from pprint import pprint
import argparse
import logging
import sys
import shutil
import os
import yaml


__all__ = [
    "update_cache"
]

handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(pit_crew.PitCrewFormatter())
logger = logging.getLogger()
logger.addHandler(handler)
logger.setLevel(logging.INFO)


# Init overall parser
parser = argparse.ArgumentParser(
    prog="building_map_model_downloader",
    description="Parse traffic_editor building files to find missing models "
                "and download them from Fuel using pit_crew. "
                "Necessary only if you are using Gazebo with Fuel models."
)
parser.add_argument("-c", "--cache", type=str,
                    default="~/.pit_crew/model_cache.json",
                    help="Path to pit_crew model cache")


def update_cache(cache):
    """Update the model cache"""
    pit_crew.build_and_update_cache(cache)

def main():
    args = parser.parse_args()
    update_cache(args.cache)

if __name__ == "__main__":
    main()
