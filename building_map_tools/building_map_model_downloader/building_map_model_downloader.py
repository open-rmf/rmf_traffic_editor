#!/usr/bin/env python3

from building_map.generator import Generator
import pit_crew

from pprint import pprint
import argparse
import logging
import sys

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
                    help="Path to check models from and download models to")
parser.add_argument("-c", "--cache", type=str,
                    default="~/.pit_crew/model_cache.json",
                    help="Path to pit_crew model cache")


def download_models(input_yaml, model_path=None, cache=None):
    """Download models for a given input building yaml."""
    # Construct model set
    model_set = set()
    stringent_dict = {}  # Dict to tighten download scope

    building = g.parse_editor_yaml(input_yaml)

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

    missing_models = pit_crew.get_missing_models(
        model_set,
        model_path=model_path,
        cache_file_path=cache,
        lower=True
    )

    logger.info("\n== REQUESTED MODEL REPORT ==")
    pprint(missing_models)

    logger.info("\n== THE FOLLOWING MODELS HAVE SPECIFIED AUTHORS ==")
    pprint(stringent_dict)
    print()

    for downloadable_model in missing_models.get('downloadable', []):
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

        pit_crew.download_model(model_name, author_name, sync_names=True,
                                download_path=model_path)

    if missing_models.get('missing', []):
        logger.warning("\nMissing models (not in local or Fuel):")
        pprint(missing_models['missing'])


def main():
    args = parser.parse_args()
    download_models(args.INPUT_YAML, args.model_path, args.cache)


if __name__ == "__main__":
    main()
