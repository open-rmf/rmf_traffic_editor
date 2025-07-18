#!/usr/bin/env python3

from collections import namedtuple
from pprint import pprint
import argparse
import json
import logging
import requests
import sys
import shutil
import os
import yaml


__all__ = [
    "update_cache"
]


class PitCrewFormatter(logging.Formatter):
    """Logging formatter for pit_crew."""

    FORMATS = {
        logging.ERROR: "ERROR::%(module)s.%(funcName)s():%(lineno)d: %(msg)s",
        logging.WARNING: "WARNING::%(module)s.%(funcName)s():%(lineno)d: "
                         "%(msg)s",
        logging.DEBUG: "DBG: %(module)s: %(lineno)d: %(msg)s",
        "DEFAULT": "%(msg)s",
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno, self.FORMATS['DEFAULT'])
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(PitCrewFormatter())
logger = logging.getLogger()
logger.addHandler(handler)
logger.setLevel(logging.INFO)

ModelNames = namedtuple("ModelNames", ["model_name", "author_name"])


# Init overall parser
parser = argparse.ArgumentParser(
    prog="building_map_model_downloader",
    description="Parse traffic_editor building files to find models on Fuel "
                "and make them refer to the Fuel URI using pit_crew. "
                "Necessary only if you are using Gazebo with Fuel models."
)
parser.add_argument(
    "-c", "--cache",
    type=str,
    default="~/.pit_crew/model_cache.json",
    help="Path to pit_crew model cache",
)
parser.add_argument(
    "--force",
    action='store_true',
    help="Force rebuilding the cache",
)

# Deprecated arguments. We keep these to avoid breaking backwards compatibility
# with existing map packages.
parser.add_argument(
    "INPUT_YAML",
    nargs='*',
    default=None,
    help="Deprecated argument - Do not use",
)
parser.add_argument(
    "-m", "--model-path",
    type=str,
    default=None,
    help="Deprecated argument - Do not use"
)
parser.add_argument(
    "-f", "--fuel-tools",
    action='store_true',
    help="Deprecated argument - Do not use"
)
parser.add_argument(
    "-i", "--include",
    type=str,
    default=None,
    help="Deprecated argument - Do not use",
)
parser.add_argument(
    "-e", "--export-path",
    type=str,
    default=None,
    help="Deprecated argument - Do not use",
)


def load_cache(cache_file_path: str):
    """
    Read local Fuel model listing cache.

    Args:
        cache_file_path (str): The path to the model cache file.

    Returns:
        dict: Cache dict, with keys 'model_cache' and 'fuel_cache'.
            model_cache will contain ModelNames tuples of
            (model_name, author_name).
            Whereas fuel_cache will contain JSON responses from Fuel.

    Notes:
        The model listing cache is local and used by pit_crew only.
    """
    try:
        cache_file_path = os.path.expanduser(cache_file_path)
        with open(cache_file_path, "r") as f:
            loaded_cache = json.loads(f.read())

            model_cache = set(
                ModelNames(*x) for x in loaded_cache.get("model_cache")
            )

            logger.info("Load success!\n")
            return {'model_cache': model_cache,
                    'fuel_cache': loaded_cache.get("fuel_cache", [])}
    except Exception as e:
        logger.error("Could not parse cache file: %s! %s"
                     % (cache_file_path, e))
        return {'model_cache': set(),
                'fuel_cache': []}


def update_cache(cache_file_path: str, rebuild: bool):
    """
    Build and/or update the local Gazebo Fuel model listing cache.

    Args:
        cache_file_path (str): The path to the model cache file.
        rebuild (bool): If True, deletes and rebuilds the cache.

    Notes:
        The model listing cache is local and used by pit_crew only.
    """
    cache_file_path = os.path.expanduser(cache_file_path)

    # Check if directory exists and make it otherwise
    dir_name = os.path.dirname(cache_file_path)

    if not os.path.exists(dir_name) and dir_name != "":
        logger.info("Cache directory does not exist! Creating it: %s"
                    % dir_name)
        os.makedirs(dir_name, exist_ok=True)

    if rebuild:
        old_cache = {'model_cache': set(), 'fuel_cache': []}
        logger.info("Rebuilding cache...")
    else:
        if os.path.exists(cache_file_path):
            old_cache = load_cache(cache_file_path)
            logger.info("Cache found! Model count: %d \nUpdating cache..."
                        % len(old_cache['model_cache']))
        else:
            old_cache = {'model_cache': set(), 'fuel_cache': []}
            logger.info("Cache not found! Rebuilding cache...")

    url_base = "https://fuel.gazebosim.org/1.0/models"
    break_flag = False
    page = 1
    new_cache_count = 0

    logger.info(f"Topping up Fuel (updating cache) from: {url_base}")

    # RFE: Doing this asynchronously will significantly speed this up.
    # Any solution must guarantee:
    #   - All new models are guaranteed to be added to the cache
    #   - Loop breaks as soon as we start pulling up already cached models
    while not break_flag:
        logger.info("Fetching page: %d" % page)

        resp = requests.get("%s?page=%d&per_page=100" % (url_base, page))
        page += 1

        if resp.status_code != 200:
            break

        for model in json.loads(resp.text):
            model_name = model.get("name", "")
            author_name = model.get("owner", "")

            # If a cached model was found, halt
            if (
                (model_name, author_name) in old_cache['model_cache']
                # this particular model is duplicated in fuel,
                # causing the cache to break early
                and model_name != "ur5_rg2" and author_name != "anni"
            ):
                logger.info("Cached model found! "
                            "Halting Fuel traversal...")
                break_flag = True
                break
            # Otherwise, add it to the cache
            else:
                new_cache_count += 1
                old_cache['model_cache'].add((model_name, author_name))
                old_cache['fuel_cache'].append(model)

    # Listify model_cache to allow for JSON serialisation
    old_cache['model_cache'] = list(old_cache['model_cache'])

    logger.info("Writing to cache: %s" % cache_file_path)
    with open(cache_file_path, "w") as f:
        f.write(json.dumps(old_cache, indent=2))

    logger.info("New models cached: %s\n" % new_cache_count)

    return old_cache


def main():
    args = parser.parse_args()

    using_deprecated_arg = (
        args.INPUT_YAML
        or args.model_path
        or args.fuel_tools
        or args.include
        or args.export_path
    )

    if using_deprecated_arg:
        logger.info(
            "A deprecated argument has been used when calling "
            "building_map_model_downloader"
        )

        # We print to stderr because By default colcon build will display
        # stderr output to users while suppressing stdout, so sending this to
        # stderr will make users more likely to see it.
        print(
            "DEPRECATED: The arguments for building_map_model_downloader have "
            "changed. If you are using it in the CMakeLists.txt of a map "
            "package, then you can remove all the command line arguments for "
            "it and run it a single time instead of running inside a foreach "
            "loop.",
            file=sys.stderr
        )
    update_cache(args.cache, args.force)


if __name__ == "__main__":
    main()
