#!/usr/bin/env python3

from building_map.generator import Generator
import pit_crew

from pprint import pprint
import argparse
import logging
import sys
import shutil
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


class HTTPDownloadDeprecated(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        logger.warn('DEPRECATED: The Options -f and -m is no longer in use. \
            Please remove these.')
        setattr(namespace, self.dest, values)


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
                    action=HTTPDownloadDeprecated,
                    help="Path to check models from and download models to. \
                        Redundant if using ignition fuel tools flag")
parser.add_argument("-c", "--cache", type=str,
                    default="~/.pit_crew/model_cache.json",
                    help="Path to pit_crew model cache")
parser.add_argument("-f", "--fuel-tools", action='store_true',
                    help="Use ignition fuel tools to download models instead "
                         "of http")
parser.add_argument("-i", "--include", type=str, default=None,
                    help="Search this directory first for models.")
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


def export_downloaded_model(model_path, export_path):
    if os.path.exists(export_path) or os.path.isdir(export_path):
        logger.warning("%s already exists, skipping..." % (export_path))
        return
    logger.info("Exporting to %s..." % (export_path))
    shutil.copytree(src=model_path, dst=export_path)


def download_models(
        input_yaml,
        cache=None,
        include=None,
        fuel_tools=True,
        export_path=None):
    """Download models for a given input building yaml."""
    # Construct model set

    IGN_FUEL_MODEL_PATH = "~/.ignition/fuel/"

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

    if fuel_tools:
        missing_models = pit_crew.get_missing_models(
            model_set,
            model_path=IGN_FUEL_MODEL_PATH,
            cache_file_path=cache,
            lower=True,
            priority_dir=include,
            ign=True,
            use_dir_as_name=True
        )
    else:
        missing_models = pit_crew.get_missing_models(
            model_set,
            cache_file_path=cache,
            lower=False,
            priority_dir=include,
            ign=False,
            use_dir_as_name=True
        )

    logger.info("\n== REQUESTED MODEL REPORT ==")
    pprint(missing_models)

    logger.info("\n== THE FOLLOWING MODELS HAVE SPECIFIED AUTHORS ==")
    pprint(stringent_dict)
    print()

    missing_downloadables = missing_models.get('downloadable', [])
    for key, downloadable_model in enumerate(missing_downloadables):
        model_name, author_names = downloadable_model

        if model_name.lower() in stringent_dict:
            author_name = stringent_dict[model_name.lower()]
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

        model_downloaded = False
        for i in range(5):
            model_downloaded = False
            if fuel_tools:
                model_downloaded = pit_crew.download_model_fuel_tools(
                    model_name, author_name,
                    sync_names=True, export_path=export_path)
            else:
                model_downloaded = pit_crew.download_model(
                    model_name, author_name, sync_names=True)
            if model_downloaded:
                break
            else:
                logger.warning("Retrying %d of 5 times...", i)

        if not model_downloaded:
            logger.error(
                "Failed to download model [%s/%s] after 5 attempts, "
                "exiting...",
                author_name,
                model_name)
            sys.exit(1)

    if missing_models.get('missing', []):
        logger.warning("\nMissing models (not in local or Fuel):")
        pprint(missing_models['missing'])

    if export_path is not None:
        available_models = missing_models.get('available', [])
        for model_name, paths in available_models:
            if paths is None:
                logger.warning("No path for exporting model %s, skipping..."
                               % (model_name))
                continue

            if len(paths) > 1:
                logger.warning("More than one path for model %s, selecting "
                               "first path that was found, %s"
                               % (model_name, paths[0]))

            export_model_path = os.path.join(export_path, model_name)
            export_model_path = os.path.expanduser(export_model_path)
            export_downloaded_model(paths[0], export_model_path)


def main():
    args = parser.parse_args()
    download_models(
        args.INPUT_YAML,
        args.cache,
        args.include,
        args.fuel_tools,
        args.export_path)


if __name__ == "__main__":
    main()
