"""
Author: github.com/methylDragon

     (   (                 (
     )⧹ ))⧹ ) .   )    (   )⧹ )    (  (
    (()/(()/` )  /(    )⧹ (()/((   )⧹))(   .
     /(_)/(_)( )(_)) (((_) /(_))⧹ ((_)()⧹ )
    (_))(_))(_(_())  )⧹___(_))((_)_(())⧹_)()
    | _ |_ _|_   _|  (/ __| _ | __⧹ ⧹((_)/ /
    |  _/| |  | |    | (__|   | _| ⧹ ⧹/⧹/ /
    |_| |___| |_|     ⧹___|_|_|___| ⧹_/⧹_/

     ~ take a REST, and top-up your Fuel ~

Description:
    pit_crew is an Ignition Fuel REST client and library that helps fetch and
    manage your model directory for Gazebo.

Features:
    - Cache building and updating for available models on Ignition Fuel
    - Finding missing models in your Gazebo models directory
    - Adding models from Fuel into your Gazebo models directory

See Also:
    pit_crew is very much like a Gazebo supporting variant of:
    https://github.com/ignitionrobotics/ign-fuel-tools

    In other words, pit_crew is not related to ign-fuel-tools, but fills the
    same feature-niche, except for Gazebo instead of Ignition Gazebo.

    Some implementation details also differ.
"""

import xml.etree.ElementTree as ET

import requests
import logging
import zipfile
import shutil
import json
import glob
import io
import os

__all__ = [
    "get_missing_models",
    "get_model_name_tuples",
    "get_model_name_tuple",
    "get_author_to_model_dict",
    "get_model_to_author_dict",
    "get_fuel_authors",
    "list_fuel_models",
    "download_model",
    "construct_license",
    "load_cache",
    "build_and_update_cache",
    "PitCrewFormatter"
]

# Init logger
logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())

###############################################################################
# MODEL PARSING
###############################################################################


def get_missing_models(model_names, model_path=None,
                       config_file="model.config",
                       cache_file_path=None, update_cache=True):
    """
    Classify models as missing, downloadable, or available from a model list.

    Args:
        model_names (list): List of model names to classify.
        model_path (str, optional): Overall path to Gazebo model directory.
            Defaults to None. If None, function will use "~/.gazebo/models".
        config_file (str, optional): Name of the config file to parse.
            Defaults to "model.config".
        cache_file_path (str, optional): The path to the model cache file.
            Defaults to None. If None, function will use
            "~/.pit_crew/model_cache.json".
        update_cache (bool, optional): If True, will update the cache.
            Defaults to True.

    Returns:
        dict: A dictionary of classified model names.
            - Available models are models that are already in your local
                directory.
            - Downloadable models are models that are available on Fuel
                but are not downloaded yet.
            - Missing models are models that are not in your local directory
                and also missing from Fuel.
    """
    if update_cache:
        cache = build_and_update_cache(cache_file_path=cache_file_path,
                                       write_to_cache=True)
    else:
        cache = load_cache(cache_file_path)

    fuel_models = get_model_to_author_dict(cache['model_cache'])
    local_models = set(x[0] for x in get_model_name_tuples(model_path))

    output = {'missing': [],
              'downloadable': [],
              'available': []}

    for model in model_names:
        if model in local_models:
            output['available'].append(model)
        elif model in fuel_models:
            output['downloadable'].append(model)
        else:
            output['missing'].append(model)

    return output


def get_model_name_tuples(path=None, config_file="model.config"):
    """
    Gets all model name tuples from a given overall model path.

    Args:
        path (str, optional): Overall path to Gazebo model directory.
            Defaults to None. If None, function will use "~/.gazebo/models".
        config_file (str, optional): Name of the config file to parse.
            Defaults to "model.config".

    Returns:
        set of (str, str): Set of unique model name tuples of
            (model_name, author_name). Each name will be lowercase only.
    """
    output = set()

    if path is None:
        path = os.path.expanduser("~/.gazebo/models")
        logger.warning("No path given! Searching %s instead!" % path)
    else:
        assert path.isdir(), "Path given must be a directory!"

    for model_path in glob.glob(path + "/*/"):
        if config_file in os.listdir(model_path):
            name_tuple = get_model_name_tuple(os.path.join(model_path,
                                                           config_file))

            if name_tuple in output:  # Throw a warning if duplicate found
                logger.warning("%s already exists! "
                               "%s seems to be a duplicate model. Ignoring"
                               % (str(name_tuple), model_path))
            else:
                output.add(
                    get_model_name_tuple(os.path.join(model_path, config_file))
                    )
        else:
            logger.warning("%s does not contain a valid config_file!"
                           "Skipping..." % model_path)

    return output


def get_model_name_tuple(config_file_path, config_file="model.config",
                         default_author_name=""):
    """
    Gets model and author name for a given model.config file.

    Args:
        config_file_path (str): Path to model.config file.
        config_file (str, optional): Name of the config file to parse.
            Defaults to "model.config".
        default_author_name (str, optional): The author name to use if no
            author name was specified in the model.config file. Defualts to "".

    Returns:
        (str, str): Model name tuple of (model_name, author_name). Each name
            will be lowercase only.

    Warnings:
        All name tuples will be lowercase!

    Note:
        This still works if you give a model directory instead! But it will
        throw a warning.
    """
    model_name, author_name = ("", default_author_name)

    try:
        # If supplied path is a directory, try parsing a config file anyway
        if os.path.isdir(config_file_path):
            config_file_path = os.path.join(config_file_path, config_file)
            logger.warning("Should have passed in the %s file! "
                           "Parsing file: %s" % (config_file,
                                                 config_file_path))
        # Otherwise, replace the config_file var with the targeted filename
        else:
            config_file = os.path.basename(config_file_path)
        tree = ET.parse(config_file_path)
        model_name = tree.find("name").text.lower()
        author_name = tree.find("author").find("name").text.lower()
    except Exception as e:
        logger.error("Could not parse %s file! %s"
                     % (config_file, e))

    return (model_name.lower(), author_name.lower())


def get_author_to_model_dict(model_name_tuples):
    """
    Get a dictionary of author names mapped to model names.

    Args:
        model_name_tuples (list or set): An iterable of model name tuples of
            (model_name, author_name). Each name will be lowercase only.

    Returns:
        dict: Dictionary mapping author names to model names.
    """
    output = {}

    for (model_name, author_name) in model_name_tuples:
        if author_name in output:
            output[author_name].append(model_name)
        else:
            output[author_name] = [model_name]

    return output


def get_model_to_author_dict(model_name_tuples):
    """
    Get a dictionary of model names mapped to author names.

    Args:
        model_name_tuples (list or set): An iterable of model name tuples of
            (model_name, author_name). Each name will be lowercase only.

    Returns:
        dict: Dictionary mapping model names to author names.
    """
    output = {}

    for (model_name, author_name) in model_name_tuples:
        if model_name in output:
            output[model_name].append(author_name)
        else:
            output[model_name] = [author_name]

    return output


###############################################################################
# FUEL CLIENT
###############################################################################

def get_fuel_authors(model_name, cache_file_path=None, update_cache=True):
    """Get all Fuel authors/owners for a given model name."""
    if update_cache:
        cache = build_and_update_cache(cache_file_path=cache_file_path,
                                       write_to_cache=True)
    else:
        cache = load_cache(cache_file_path)

    return get_model_to_author_dict(cache['model_cache']).get(model_name, [])


def list_fuel_models(cache_file_path=None, update_cache=True, model_limit=-1):
    """List all Fuel models."""
    if update_cache:
        cache = build_and_update_cache(cache_file_path=cache_file_path,
                                       write_to_cache=True)
    else:
        cache = load_cache(cache_file_path)

    sorted_authors = list(
        get_author_to_model_dict(cache['model_cache']).items()
    )
    sorted_authors.sort()
    print("\n[Models]\n")

    for (author_name, model_names) in sorted_authors:
        model_names.sort()
        print(author_name)

        for model_name in model_names[:min(len(model_names), model_limit) - 1]:
            print("├─", model_name)

        if model_limit == -1:
            print("└─", model_names[-1])
        else:
            print("└─", model_names[min(len(model_names), model_limit) - 1])


###############################################################################
# MODEL DOWNLOADING
###############################################################################

def download_model(model_name, author_name, version="tip",
                   download_path=None, overwrite=True, ign=False):
    """
    Fetch and download a model from Fuel.

    Supports both Gazebo and Ignition directory structures!

    Args:
        model_name (str): Model name as listed on Fuel. Case insensitive.
        author_name (str): Model Author/Owner as listed on Fuel. Case
            insensitive.
        version (int, optional): Version of the model to download. Defaults to
            "tip", which will download the latest model.
        download_path (str, optional): The root directory for downloading
            and unzipping the models into. Defaults to None. If None, function
            will use "~/.ignition/fuel/fuel.ignitionrobotics.org" or
            "~/.gazebo/models" depending on the state of the ign argument.
        overwrite (bool, optional): Overwrite existing model files when
            downloading. Defaults to True.
        ign (bool, optional): Use Ignition file directory structure and default
            paths. Defaults to False.

    Returns:
        bool, (dict or None): True if successful. False otherwise. The dict
            returned is the JSON Fuel metadata dict for the latest model.
    """
    try:
        if download_path is None:
            if ign:
                download_path = os.path.expanduser(
                    "~/.ignition/fuel/fuel.ignitionrobotics.org"
                )
            else:
                download_path = os.path.expanduser("~/.gazebo/models")
            logger.warning("No path given! Downloading to %s instead!"
                           % download_path)
        else:
            assert download_path.isdir(), "Path given must be a directory!"

        url_base = "https://fuel.ignitionrobotics.org/1.0"
        metadata = requests.get("%s/%s/models/%s/%s/%s"
                                % (url_base, author_name,
                                   model_name, version, model_name))

        assert metadata.status_code == 200, \
            "Model %s does not exist!" % model_name

        model = requests.get("%s/%s/models/%s/%s/%s.zip"
                             % (url_base, author_name,
                                model_name, version, model_name))

        metadata_dict = json.loads(metadata.text)
        print(metadata_dict)
        model_zipfile = zipfile.ZipFile((io.BytesIO(model.content)))

        assert metadata.status_code == 200, \
            "Model data for %s does not exist!" % model_name

        model_name = metadata_dict['name']
        author_name = metadata_dict['owner']

        if version == "tip":
            version = metadata_dict['version']

        if ign:
            extract_path = os.path.join(download_path,
                                        author_name, "models", model_name,
                                        str(version))
        else:
            extract_path = os.path.join(download_path, model_name)

        # Remove pre-existing model and extract latest model
        if overwrite:
            shutil.rmtree(extract_path)

        model_zipfile.extractall(path=extract_path)

        with open(os.path.join(extract_path, "LICENSE"), "w") as f:
            f.write(construct_license(metadata_dict))

        logger.info("%s downloaded to: %s" % (model_name, extract_path))
        return True, metadata_dict
    except Exception as e:
        logger.error("Could not download %s! %s" % (model_name, e))
        return False, None


def construct_license(fuel_metadata_dict):
    """Construct LICENSE string from an input Fuel metadata dictionary."""
    return ("License: %s\n"
            "License URL: %s\n"
            "License Image: %s"
            % (fuel_metadata_dict.get("license_name", ""),
               fuel_metadata_dict.get("license_url", ""),
               fuel_metadata_dict.get("license_image", "")))


###############################################################################
# CACHING
###############################################################################

def load_cache(cache_file_path=None):
    """
    Read local Ignition Fuel model listing cache.

    Args:
        cache_file_path (str, optional): The path to the model cache file.
            Defaults to None. If None, function will use
            "~/.pit_crew/model_cache.json".

    Returns:
        dict: Cache dict, with keys 'model_cache' and 'fuel_cache'.
            model_cache will contain model name tuples of
            (model_name, author_name). Each name will be lowercase only.
            Whereas fuel_cache will contain JSON responses from Fuel.

    Notes:
        The model listing cache is local and used by pit_crew only.
    """
    try:
        if cache_file_path is None:
            cache_file_path = os.path.expanduser(
                "~/.pit_crew/model_cache.json"
            )
            logger.warning("No path given! Using %s instead!"
                           % cache_file_path)

        with open(cache_file_path, "r") as f:
            loaded_cache = json.loads(f.read())
            model_cache = set(
                tuple(x) for x in loaded_cache.get("model_cache")
            )

            logger.info("Load success!")
            return {'model_cache': model_cache,
                    'fuel_cache': loaded_cache.get("fuel_cache", [])}
    except Exception as e:
        logger.error("Could not parse cache file: %s! %s"
                     % (cache_file_path, e))
        return {'model_cache': set(),
                'fuel_cache': []}


def build_and_update_cache(cache_file_path=None, write_to_cache=True):
    """
    Build and/or update the local Ignition Fuel model listing cache.

    Args:
        cache_file_path (str, optional): The path to the model cache file.
            Defaults to None. If None, function will use
            "~/.pit_crew/model_cache.json".
        write_to_cache (bool, optional): If True, writes to model cache.
            Defaults to True.

    Notes:
        The model listing cache is local and used by pit_crew only.
    """
    if cache_file_path is None:
        cache_file_path = os.path.expanduser("~/.pit_crew/model_cache.json")
        logger.warning("No path given! Searching %s instead!"
                       % cache_file_path)

    # Check if directory exists and make it otherwise
    if not os.path.exists(os.path.dirname(cache_file_path)):
        logger.info("Cache directory does not exist! Creating it: %s"
                    % os.path.dirname(cache_file_path))
        os.makedirs(os.path.dirname(cache_file_path), exist_ok=True)

    if os.path.exists(cache_file_path):
        old_cache = load_cache(cache_file_path)
        logger.info("Cache found! Model count: %d \nUpdating cache..."
                    % len(old_cache['model_cache']))
    else:
        old_cache = set()
        logger.info("Cache not found! Rebuilding cache...")

    url_base = "https://fuel.ignitionrobotics.org/1.0/models"
    status = 200
    break_flag = False
    page = 1
    new_cache_count = 0

    logger.info("Topping up Fuel (updating cache) from:"
                " https://fuel.ignitionrobotics.org/1.0/models")

    # RFE: Doing this asynchronously will significantly speed this up.
    # Any solution must guarantee:
    #   - All new models are guaranteed to be added to the cache
    #   - Loop breaks as soon as we start pulling up already cached models
    while status == 200 and not break_flag:
        logger.info("Fetching page: %d" % page)

        resp = requests.get("%s?page=%d" % (url_base, page))
        status = resp.status_code
        page += 1

        if status == 200:
            for model in json.loads(resp.text):
                model_name = model.get("name", "").lower()
                author_name = model.get("owner", "").lower()

                # If a cached model was found, halt
                if (model_name, author_name) in old_cache['model_cache']:
                    logger.info("Cached model found! "
                                "Halting Fuel traversal...")
                    break_flag = True
                    break
                # Otherwise, add it to the cache
                else:
                    new_cache_count += 1
                    old_cache['model_cache'].add((model_name, author_name))
                    old_cache['fuel_cache'].append(model)
        else:
            break

    # Listify model_cache to allow for JSON serialisation
    old_cache['model_cache'] = list(old_cache['model_cache'])

    if write_to_cache:
        logger.info("Writing to cache: %s" % cache_file_path)
        with open(cache_file_path, "w") as f:
            f.write(json.dumps(old_cache, indent=2))

        logger.info("New models cached: %s\n" % new_cache_count)

    return old_cache


###############################################################################
# MISC.
###############################################################################

class PitCrewFormatter(logging.Formatter):
    """Logging formatter for pit_crew."""

    FORMATS = {
        logging.ERROR: "ERROR (%(funcName)s()): %(msg)s",
        logging.WARNING: "WARNING (%(funcName)s()): %(msg)s",
        logging.DEBUG: "DBG: %(module)s: %(lineno)d: %(msg)s",
        "DEFAULT": "%(msg)s",
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno, self.FORMATS['DEFAULT'])
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)
