#!/usr/bin/env python3

from building_map_model_downloader import download_models
import subprocess
import logging
import argparse
import glob
import os

# Init overall parser
parser = argparse.ArgumentParser(
    prog="model_downloader",
    description="Convenience script to download Fuel models for simulator "
                "map packages with maps made in traffic_editor. "
                "(Allows for downloading of  models for all maps in the "
                "package, or just specific maps.)"
)
parser.add_argument("MAP_PACKAGE", type=str,
                    help="Map package name you want to download from")
parser.add_argument("-l", "--list", action="store_true",
                    help="List all available maps in package")
parser.add_argument("-s", "--select-maps", nargs="+",
                    help="Specify a subset of maps to download models for")
parser.add_argument("-m", "--model-path", type=str,
                    default="~/.gazebo/models/",
                    help="Path to check models from and download models to")
parser.add_argument("-c", "--cache", type=str,
                    default="~/.pit_crew/model_cache.json",
                    help="Path to pit_crew model cache")

# Init logger
logger = logging.getLogger(__name__)


def main():
    args = parser.parse_args()

    # OBTAIN PREFIX ==========================================================
    completed_process = subprocess.run(
        ["ros2", "pkg", "prefix", args.MAP_PACKAGE], stdout=subprocess.PIPE)
    prefix = completed_process.stdout.decode('utf-8').strip()

    if completed_process.returncode != 0:
        logger.error("Package does not exist! Prefix resolution failed!")
        return

    # GET BUILDING YAML PATHS ================================================
    # NOTE(CH3): Strongly resisting the urge to hilariously call this map_map
    map_dict = {
        os.path.basename(x).replace(".building.yaml", ""): x
        for x in glob.glob(prefix + "/**/*.building.yaml", recursive=True)
    }

    if not len(map_dict):
        logger.error("Package has no maps!")
        return

    # HANDLE --list ==========================================================
    if args.list:
        print(f"Available maps in {args.MAP_PACKAGE}:")
        for name, path in map_dict.items():
            print(f"- {name}")
        return

    # HANDLE --select-maps ===================================================
    if args.select_maps:
        selected_maps = []

        for name in args.select_maps:
            if name in map_dict:
                selected_maps.append(name)
            else:
                logger.warning(f"{name} does not exist in {args.MAP_PACKAGE}! "
                               "Ignoring!")

    # DOWNLOAD MODELS ========================================================
    if args.select_maps:  # Download models for selected maps
        for name in selected_maps:
            logger.info(f"\n== STARTING: Downloading models for {name} ==\n")
            download_models(map_dict[name], args.cache, None, args.model_path)
            logger.info(f"\n== COMPLETE: Downloaded models for {name} ==\n")
    else:  # Download models for all maps in package
        for name, yaml_path in map_dict.items():
            logger.info(f"\n== STARTING: Downloading models for {name} ==\n")
            download_models(yaml_path, args.cache, None, args.model_path)
            logger.info(f"\n== COMPLETE: Downloaded models for {name} ==\n")


if __name__ == "__main__":
    main()
