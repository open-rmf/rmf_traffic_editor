#!/usr/bin/env python3

from building_map.generator import Generator
import pit_crew

from pprint import pprint
import argparse
import logging
import sys
import os
import yaml

__all__ = [
    "download_models"
]

handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(pit_crew.PitCrewFormatter())
logger = logging.getLogger()
logger.addHandler(handler)
logger.setLevel(logging.INFO)

g = Generator()

# Init overall parser
parser = argparse.ArgumentParser(
    prog="building_map_model_downloader",
    description="Parse traffic_editor building files to find missing models "
                "and download them from Fuel using pit_crew. "
                "Necessary only if you are using Gazebo with Fuel models."
)
parser.add_argument("INPUT_YAML", type=str,
                    help="Input building.yaml file to process")
parser.add_argument("-m", "--model-path", type=str,
                    default="~/.gazebo/models/",
                    help="Path to check models from and download models to. \
                        Redundant if using ignition fuel tools flag")
parser.add_argument("-c", "--cache", type=str,
                    default="~/.pit_crew/model_cache.json",
                    help="Path to pit_crew model cache")
parser.add_argument("-i", "--include", type=str,
                    help="Search this directory first for models. "
                         "If -f flag is specified, then directory must "
                         "follow ignition gazebo directory structure.")
parser.add_argument("-f", "--fuel-tools", action='store_true',
                    help="Use ignition fuel tools to download models instead "
                         "of http")
parser.add_argument("-e", "--export-path", type=str, default=None,
                    help="Export model downloaded using ignition fuel tools "
                         "to a folder with classic gazebo directory structure."
                         " Only relevant if ignition fuel tools is used to "
                         "download models.")


def get_crowdsim_models(input_filename):
    if not os.path.isfile(input_filename):
        raise FileNotFoundError(f'input file {input_filename} not found')

    actor_names = []
    with open(input_filename, 'r') as f:
        y = yaml.safe_load(f)
        try:
            model_types = y["crowd_sim"]["model_types"]
            for model in model_types:
                name = model["model_uri"].split("://")[-1]
                actor_names.append(name)
            logger.info(f"Models: {actor_names} are used in crowd_sim")
        except Exception as e:
            logger.error(f"Could not get crowd_sim models, error: {e}."
                         " Ignore models in crowd_sim...")
        return actor_names


def download_models(
        input_yaml,
        model_path=None,
        cache=None,
        include=None,
        fuel_tools=False,
        export_path=None):
    """Download models for a given input building yaml."""
    # Construct model set
    model_set = set()
    stringent_dict = {}  # Dict to tighten download scope

    building = g.parse_editor_yaml(input_yaml)

    # models used for crowd sim
    actors = get_crowdsim_models(input_yaml)
    model_set.update(actors)

    for _, level in building.levels.items():
        for model in level.models:
            if "/" in model.model_name:
                model_name = "".join(model.model_name.split("/")[1:])
                author_name = model.model_name.split("/")[0]

                model_set.add((model_name, author_name))
                stringent_dict[model_name.lower()] = \
                    author_name.lower()
            else:
                model_set.add(model.model_name)

    # If we are using ignition fuel tools to download the models
    # and we do not need to parse them, it means the model directory follows
    # the ignition gazebo directory structure
    ign = fuel_tools
    logging.info("\nUsing Ignition Fuel directory struture : %s\n" % (ign))
    # Ignition fuel tools can only download to this folder, so we set the
    # model path to it
    if fuel_tools:
        model_path = "~/.ignition/fuel/"

    missing_models = pit_crew.get_missing_models(
        model_set,
        model_path=model_path,
        cache_file_path=cache,
        lower=True,
        priority_dir=include,
        ign=ign
    )

    logger.info("\n== REQUESTED MODEL REPORT ==")
    pprint(missing_models)

    logger.info("\n== THE FOLLOWING MODELS HAVE SPECIFIED AUTHORS ==")
    pprint(stringent_dict)
    print()

    missing_downloadables = missing_models.get('downloadable', [])
    for key, downloadable_model in enumerate(missing_downloadables):
        model_name, author_names = downloadable_model

        if model_name in stringent_dict:
            author_name = stringent_dict[model_name]
            logger.info("\nDownloading %s by %s from Fuel"
                        % (model_name, author_name))
        else:
            author_name = author_names[0]
            logger.info("\nDownloading %s by %s from Fuel"
                        % (model_name, author_name))

            logger.warning("No author specified for model '%s', "
                           "using first valid author '%s'" %
                           (model_name, author_name))

        logger.info("Downloading model %s / %s : %s" %
                    (key + 1, len(missing_downloadables), model_name))

        if fuel_tools:
            pit_crew.download_model_fuel_tools(
                model_name, author_name,
                sync_names=True, export_path=export_path)
        else:
            pit_crew.download_model(model_name, author_name, sync_names=True,
                                    download_path=model_path, ign=ign)

    if missing_models.get('missing', []):
        logger.warning("\nMissing models (not in local or Fuel):")
        pprint(missing_models['missing'])


def main():
    args = parser.parse_args()
    download_models(
        args.INPUT_YAML,
        args.model_path,
        args.cache,
        args.include,
        args.fuel_tools,
        args.export_path)


if __name__ == "__main__":
    main()
