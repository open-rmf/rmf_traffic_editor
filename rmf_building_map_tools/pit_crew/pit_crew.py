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

from collections import namedtuple
import xml.etree.ElementTree as ET

from urllib import parse
import subprocess
import requests
import logging
import zipfile
import shutil
import string
import json
import glob
import sys
import io
import os
import re

__all__ = [
    "swag",
    "get_missing_models",
    "get_local_model_name_tuples",
    "get_model_name_tuple",
    "get_author_to_model_dict",
    "get_model_to_author_dict",
    "get_fuel_authors",
    "list_fuel_models",
    "download_model",
    "_construct_license",
    "load_cache",
    "build_and_update_cache",
    "init_logging",
    "PitCrewFormatter",
    "ModelNames",
    "download_model_fuel_tools",
    "sync_sdf"
]

# Init logger
logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())

ModelNames = namedtuple("ModelNames", ["model_name", "author_name"])


def remove_spaces(original):
    return ''.join([x for x in original if x not in string.whitespace])


def swag(print_swag=True):
    """Swag!"""
    output = ("""
     (   (                 (
     )⧹ ))⧹ ) .   )    (   )⧹ )    (  (
    (()/(()/` )  /(    )⧹ (()/((   )⧹))(   .
     /(_)/(_)( )(_)) (((_) /(_))⧹ ((_)()⧹ )
    (_))(_))(_(_())  )⧹___(_))((_)_(())⧹_)()
    | _ |_ _|_   _|  (/ __| _ | __⧹ ⧹((_)/ /
    |  _/| |  | |    | (__|   | _| ⧹ ⧹/⧹/ /
    |_| |___| |_|     ⧹___|_|_|___| ⧹_/⧹_/

     ~ take a REST, and top-up your Fuel ~
    """)

    if print_swag:
        print(output)

    return output


###############################################################################
# MODEL PARSING
###############################################################################
def get_missing_models(model_names, model_path=None,
                       config_file="model.config",
                       cache_file_path=None, update_cache=True, lower=True,
                       use_dir_as_name=False, ign=False,
                       priority_dir=None):
    """
    Classify models as missing, downloadable, or available from a model list.

    Args:
        model_names (iterable): Iterable of model names to classify.
            Also supports ModelNames tuples, or unnamed tuples of
            (model_name, author_name)!
        model_path (str, optional): Overall path to model directory.
            Defaults to None. If None, function will use "~/.gazebo/models" or
            "~/.ignition/fuel" depending on the value of ign.
        config_file (str, optional): Name of the config file to parse when
            checking local models. Defaults to "model.config".
        cache_file_path (str, optional): The path to the model cache file.
            Defaults to None. If None, function will use
            "~/.pit_crew/model_cache.json".
        update_cache (bool, optional): If True, will update the cache.
            Defaults to True.
        lower (bool, optional): Make all output names lower-case.
            Defaults to True.
        use_dir_as_name (bool, optional): If True, will use the model's folder
            name as its model_name. Defaults to False.
        ign (bool, optional): If True, will parse model directory as if it is
            following Ignition's directory structure. Defaults to False.
        priority_dir (str, optional): Check this directory first to see if the
            a model is there.

    Returns:
        dict: A dictionary of classified model names.
            - Available models are models that are already in your local
                directory.
            - Downloadable models are models that are available on Fuel
                but are not downloaded yet. Items in this list will have
                elements of (model_name, [eligible_author_names])
            - Missing models are models that are not in your local directory
                and also missing from Fuel.
    """
    for key, model_name in enumerate(model_names):
        if isinstance(model_name, ModelNames) or isinstance(model_name, tuple):
            assert len(model_name) == 2, \
                "Invalid model name tuple given: %s!" % model_name

    if update_cache:
        cache = build_and_update_cache(cache_file_path=cache_file_path,
                                       write_to_cache=True)
    else:
        cache = load_cache(cache_file_path, lower=lower)

    fuel_models = get_model_to_author_dict(cache['model_cache'], lower=lower)
    priority_models = {}
    local_models = {}

    if priority_dir is not None:
        logger.info("Will check '%s' directory first for models"
                    % (priority_dir))
        priority_models_dict = get_local_model_name_tuples(
            priority_dir, config_file=config_file, lower=lower,
            use_dir_as_name=use_dir_as_name, ign=ign)
        for priority_model, path in priority_models_dict.items():
            if priority_model[0] in priority_models:
                priority_models[priority_model[0]].append(path)
            else:
                priority_models[priority_model[0]] = [path]

    local_models_dict = get_local_model_name_tuples(
        model_path, config_file=config_file, lower=lower,
        use_dir_as_name=use_dir_as_name, ign=ign)
    for local_model, path in local_models_dict.items():
        if local_model[0] in local_models:
            local_models[local_model[0]].append(path)
        else:
            local_models[local_model[0]] = [path]

    output = {'missing': [],
              'downloadable': [],
              'available': []}

    for model in model_names:
        # Obtain model and author names
        if isinstance(model, str):
            model_name = model
            author_name = ""
        elif isinstance(model, tuple):
            model_name = model[0]
            author_name = model[1]
        elif isinstance(model, ModelNames):
            model_name = model.model_name
            author_name = model.author_name

        model_name_original = model_name

        if lower:
            model_name = model_name.lower()
            author_name = author_name.lower()

        if model_name in priority_models and model_name in local_models:
            logger.warning(
                "Model %s found in both '%s'and '%s'! "
                " Will use model in priority folder '%s'" %
                (model_name, priority_dir, model_path, priority_dir))

        if model_name in priority_models:
            output['available'].append(
                (model_name_original, ))
            if author_name:
                if author_name not in priority_models[model_name]:
                    logger.warning("Model %s in local model directory"
                                   " is not by the requested author %s!"
                                   % (model_name, author_name))
        elif model_name in local_models:
            output['available'].append(
                (model_name_original, local_models[model_name]))
            if author_name:
                if author_name not in local_models[model_name]:
                    logger.warning("Model %s in local model directory"
                                   " is not by the requested author %s!"
                                   % (model_name, author_name))
        elif model_name in fuel_models:
            output['downloadable'].append((model_name_original,
                                          fuel_models[model_name]))

            if author_name:
                if author_name not in fuel_models[model_name]:
                    logger.warning("No models %s in Fuel are "
                                   "by the requested author %s!"
                                   % (model_name, author_name))
        else:
            output['missing'].append(model_name_original)

    return output


def get_local_model_name_tuples(path=None, config_file="model.config",
                                default_author_name="", lower=True,
                                use_dir_as_name=False, ign=False):
    """
    Gets all ModelNames tuples from a given overall local model path.

    Args:
        path (str, optional): Overall path to model directory.
            Defaults to None. If None, function will use "~/.gazebo/models" or
            "~/.ignition/fuel" depending on the value of ign.
        config_file (str, optional): Name of the config file to parse.
            Defaults to "model.config".
        default_author_name (str, optional): The author name to use if no
            author name was specified in the model.config file. Defualts to "".
        lower (bool, optional): Make all output names lower-case.
            Defaults to True.
        use_dir_as_name (bool, optional): If True, will use the model's folder
            name as its model_name. Defaults to False.
        ign (bool, optional): If True, will parse model directory as if it is
            following Ignition's directory structure. Defaults to False.

    Returns:
        dictionary (ModelNames: str): Dictionary where ModelNames tuples are
        keys, and the model path is the value. Each name will be lower-case
        only unless lower is False.
    """
    output = {}

    if path is None:
        if ign:
            path = "~/.ignition/fuel/"
        else:
            path = "~/.gazebo/models/"
        logger.warning("No local model path given! Using default %s instead!"
                       % path)

    path = os.path.expanduser(path)

    if not path.endswith("/"):
        path += "/"

    if not os.path.isdir(path):
        logger.warning("Model directory specified does not exist! "
                       "Returning empty model set.")
        return output

    if ign:
        model_dir_iter = glob.glob(path + "*/*/models/*/")
    else:
        model_dir_iter = glob.glob(path + "*/")

    for model_path in model_dir_iter:
        if ign:
            latest_ver = max(dir for dir in os.listdir(model_path))
        else:
            latest_ver = ""

        if config_file in os.listdir(os.path.join(model_path, latest_ver)):
            latest_ver_path = os.path.join(model_path, latest_ver)
            name_tuple = get_model_name_tuple(
                os.path.join(latest_ver_path, config_file),
                default_author_name=default_author_name,
                lower=lower
            )

            if name_tuple in output:  # Throw a warning if duplicate found
                logger.warning("%s already exists! "
                               "%s seems to be a duplicate model. Ignoring"
                               % (str(name_tuple), model_path))
            else:
                if use_dir_as_name:  # Use directory as name instead
                    if model_path.endswith("/"):
                        model_path = model_path[:-1]

                    name_tuple = ModelNames(os.path.basename(model_path),
                                            name_tuple.author_name)

                output[name_tuple] = latest_ver_path
        else:
            logger.warning("%s does not contain a valid config_file! "
                           "Skipping..." % model_path)

    return output


def get_model_name_tuple(config_file_path, config_file="model.config",
                         default_author_name="", lower=True):
    """
    Gets model and author name for a given model.config file.

    Args:
        config_file_path (str): Path to model.config file or its directory.
        config_file (str, optional): Name of the config file to parse.
            Defaults to "model.config".
        default_author_name (str, optional): The author name to use if no
            author name was specified in the model.config file. Defualts to "".
        lower (bool, optional): Make all output names lower-case.
            Defaults to True.

    Returns:
        (str, str): ModelNames tuple of (model_name, author_name). Each name
            will be lower-case only unless lower is False.

    Warnings:
        All name tuples will be lower-case!

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
        model_name = tree.find("name").text
        author_name = tree.find("author").find("name").text
    except Exception as e:
        logger.error("Could not parse %s file! %s"
                     % (config_file, e))

    if author_name is None:
        author_name = default_author_name

    if lower:
        return ModelNames(model_name.lower(), author_name.lower())
    else:
        return ModelNames(model_name, author_name)


def get_author_to_model_dict(model_name_tuples, lower=True):
    """
    Get a dictionary of author names mapped to model names.

    Args:
        model_name_tuples (iterable): An iterable of ModelNames or unnamed
            tuples of (model_name, author_name).
        lower (bool, optional): Make all output names lower-case.
            Defaults to True.

    Returns:
        dict: Dictionary mapping author names to model names. Output will only
            be in lower-case.
    """
    output = {}

    for (model_name, author_name) in model_name_tuples:
        if lower:
            model_name = model_name.lower()
            author_name = author_name.lower()

        if author_name in output:
            output[author_name].append(model_name)
        else:
            output[author_name] = [model_name]

    return output


def get_model_to_author_dict(model_name_tuples, lower=True):
    """
    Get a dictionary of model names mapped to author names.

    Args:
        model_name_tuples (list or set): An iterable of ModelNames tuples of
            (model_name, author_name). Each name will be lower-case only.
        lower (bool, optional): Make all output names lower-case.
            Defaults to True.

    Returns:
        dict: Dictionary mapping model names to author names. Output will only
            be in lower-case.
    """
    output = {}

    for (model_name, author_name) in model_name_tuples:
        if lower:
            model_name = model_name.lower()
            author_name = author_name.lower()

        if model_name in output:
            output[model_name].append(author_name)
        else:
            output[model_name] = [author_name]

    return output


###############################################################################
# FUEL CLIENT
###############################################################################

def get_fuel_authors(model_name, cache_file_path=None, update_cache=True):
    """Get Fuel authors/owners for a given model name. (Case insensitive)"""
    if update_cache:
        cache = build_and_update_cache(cache_file_path=cache_file_path,
                                       write_to_cache=True)
    else:
        cache = load_cache(cache_file_path)

    if isinstance(model_name, ModelNames) or isinstance(model_name, tuple):
        assert len(model_name) == 2, "Invalid model name tuple given: %s!" \
            % model_name
        model_name = model_name[0]

    return get_model_to_author_dict(cache['model_cache']) \
        .get(model_name.lower(), [])


def list_fuel_models(cache_file_path=None, update_cache=True, model_limit=-1,
                     lower=False):
    """List all Fuel models."""
    if update_cache:
        cache = build_and_update_cache(cache_file_path=cache_file_path,
                                       write_to_cache=True)
    else:
        cache = load_cache(cache_file_path, lower=lower)

    sorted_authors = sorted(
        get_author_to_model_dict(cache['model_cache'], lower=lower).items()
    )
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
                   download_path=None, overwrite=True, sync_names=False,
                   ign=False,
                   dry_run=False):
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
        sync_names (bool, optional): Change downloaded model.sdf model name to
            match folder name. Defaults to False.
        overwrite (bool, optional): Overwrite existing model files when
            downloading. Defaults to True.
        ign (bool, optional): Use Ignition file directory structure and default
            paths. Defaults to False.
        dry_run (bool, optional): If dry_run, does not actually download the
            model.

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
            download_path = os.path.expanduser(download_path)

        # Make directory if missing
        if not os.path.isdir(download_path):
            os.makedirs(download_path, exist_ok=True)
            logger.warning("Download path does not exist! Created: %s"
                           % download_path)

        url_base = "https://fuel.ignitionrobotics.org/1.0"
        metadata = requests.get("%s/%s/models/%s/%s/%s"
                                % (url_base, author_name,
                                   model_name, version, model_name))

        assert metadata.status_code == 200, \
            "Model '%s' by requested author '%s' does not exist on Fuel!" \
            % (model_name, author_name)

        model = requests.get("%s/%s/models/%s/%s/%s.zip"
                             % (url_base, author_name,
                                model_name, version, model_name))

        metadata_dict = json.loads(metadata.text)
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

        # Overwrite pre-existing model with latest model fetched
        if overwrite:
            # If this fails it is ok, that means the directory is either
            # read only or doesn't exist
            try:
                shutil.rmtree(extract_path)
            except Exception:
                pass

        if not dry_run:
            model_zipfile.extractall(path=extract_path)

        with open(os.path.join(extract_path, "LICENSE"), "w") as f:
            f.write(_construct_license(metadata_dict))

        logger.info("%s downloaded to: %s" % (model_name, extract_path))

        if sync_names:
            sync_sdf(model_name=model_name, extract_path=extract_path)

        return True, metadata_dict
    except Exception as e:
        logger.error("Could not download model '%s'! %s" % (model_name, e))
        return False, None


def download_model_fuel_tools(model_name, author_name,
                              sync_names=False, export_path=None,
                              overwrite=True):
    """
    Fetch and download a model from Fuel using the official Fuel Tools API

    Supports exporting the downloaded models into a directory with
    Gazebo structure

    Args:
        model_name (str): Model name as listed on Fuel. Case insensitive.
        author_name (str): Model Author/Owner as listed on Fuel. Case
            insensitive.
        sync_names (bool, optional): Change downloaded model.sdf model name to
            match folder name. Defaults to False.
        export_path (string, optional): If not none, then will export
            downloaded models to specified directory with gazebo
            directory structure.
        overwrite (bool, optional) : Overwrite existing model files in the
            export_path directory. Only revelant if export_path is defined.
            Defaults to True.

    Returns:
        bool: True if successful. False otherwise.
    """

    try:
        # Currently, ignition fuel download can only download to this folder.
        # Fuel tools creates this folder if it does not yet exist
        download_path = os.path.expanduser(
            "~/.ignition/fuel/fuel.ignitionrobotics.org"
        )
        # Command line
        url_model_name = parse.quote(model_name)
        full_url = ("https://fuel.ignitionrobotics.org/1.0" +
                    '/' + author_name + '/models' + '/' + url_model_name)
        full_command = full_command = ("ign fuel download -u "
                                       + full_url + " -v 4")
        subprocess.call([full_command], shell=True)

        extract_path_base = os.path.join(
            download_path,
            author_name.lower(),
            "models")
        extract_path = os.path.join(
            extract_path_base,
            model_name.lower())

        if " " in model_name:
            if os.path.isdir(extract_path):
                shutil.rmtree(extract_path)
            os.rename(os.path.join(extract_path_base, url_model_name.lower()),
                      extract_path)

        # Get the latest version downloaded by looking at the subdirectories
        # with the largest numbered name
        sub_dirs = []
        for dirname, dirnames, filenames in os.walk(extract_path):
            for subdirname in dirnames:
                try:
                    n = int(subdirname)
                    sub_dirs.append(n)
                except ValueError as e:
                    logger.error(
                        "Ignoring subdirectory %s "
                        "which is not a version number" %
                        (subdirname))
                    pass
            break
        latest_ver = max(sub_dirs)
        extract_path = os.path.join(extract_path, str(latest_ver))

        if sync_names:
            sync_sdf(model_name=model_name.lower(), extract_path=extract_path)

        export_path = os.path.expanduser(export_path)
        if export_path is not None:
            # Make directory if missing
            if not os.path.isdir(export_path):
                os.makedirs(export_path, exist_ok=True)
                logger.warning("Export path does not exist! Created: %s"
                               % export_path)
            # Model does not currently exist
            if not os.path.isdir(os.path.join(export_path, model_name)):
                export_path = os.path.join(export_path, model_name)
                try:
                    shutil.copytree(src=extract_path, dst=export_path)
                    logger.info(
                        "Exporting %s to %s" %
                        (model_name, export_path))
                except Exception as e:
                    logger.error(
                        "Could not export %s to %s" %
                        (extract_path, export_path))
            else:
                if overwrite:
                    export_path = os.path.join(export_path, model_name)
                    logger.info("Overwriting model %s" % (export_path))
                    try:
                        shutil.rmtree(path=export_path)
                        shutil.copytree(src=extract_path, dst=export_path)
                    except Exception as e:
                        logger.error(
                            "Could not export %s to %s" %
                            (extract_path, export_path))
                else:
                    logger.info(
                        "Model %s already exists at %s" %
                        (model_name, export_path))

        return True
    except Exception as e:
        logger.error("Could not download model '%s'! %s" % (model_name, e))
        return False


def _construct_license(fuel_metadata_dict):
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

def load_cache(cache_file_path=None, lower=True):
    """
    Read local Ignition Fuel model listing cache.

    Args:
        cache_file_path (str, optional): The path to the model cache file.
            Defaults to None. If None, function will use
            "~/.pit_crew/model_cache.json".
        lower (bool, optional): Make all output names lower-case.
            Defaults to True.

    Returns:
        dict: Cache dict, with keys 'model_cache' and 'fuel_cache'.
            model_cache will contain ModelNames tuples of
            (model_name, author_name).
            Whereas fuel_cache will contain JSON responses from Fuel.

    Notes:
        The model listing cache is local and used by pit_crew only.
    """
    try:
        if cache_file_path is None:
            cache_file_path = "~/.pit_crew/model_cache.json"
            logger.warning("No path given! Using %s instead!"
                           % cache_file_path)

        cache_file_path = os.path.expanduser(cache_file_path)

        with open(cache_file_path, "r") as f:
            loaded_cache = json.loads(f.read())

            if lower:
                model_cache = set(
                    ModelNames(*[name.lower() for name in x])
                    for x in loaded_cache.get("model_cache")
                )
            else:
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


def build_and_update_cache(cache_file_path=None, write_to_cache=True,
                           rebuild=False):
    """
    Build and/or update the local Ignition Fuel model listing cache.

    Args:
        cache_file_path (str, optional): The path to the model cache file.
            Defaults to None. If None, function will use
            "~/.pit_crew/model_cache.json".
        write_to_cache (bool, optional): If True, writes to model cache.
            Defaults to True.
        rebuild (bool, optional): If True, deletes and rebuilds the cache.
            Defaults to False.

    Notes:
        The model listing cache is local and used by pit_crew only.
    """
    if cache_file_path is None:
        cache_file_path = "~/.pit_crew/model_cache.json"
        logger.warning("No pit_crew cache path given! "
                       "Using default %s instead!"
                       % cache_file_path)

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
            old_cache = load_cache(cache_file_path, lower=False)
            logger.info("Cache found! Model count: %d \nUpdating cache..."
                        % len(old_cache['model_cache']))
        else:
            old_cache = {'model_cache': set(), 'fuel_cache': []}
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
                model_name = model.get("name", "")
                author_name = model.get("owner", "")

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


def init_logging():
    """Initialise pit_crew styled logging."""
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(PitCrewFormatter())
    logger = logging.getLogger()
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)


def sync_sdf(model_name, extract_path):
    # Sync all instances of the model name in sdf to the folder name
    try:
        tree = ET.parse(os.path.join(extract_path, "model.sdf"))
        sdf = tree.findall("model")[0]
        old_name = sdf.attrib.get("name")

        if old_name != model_name:
            # Sync name attribute
            sdf.attrib['name'] = model_name

            # Sync instances of name in child tags
            for uri in tree.findall('.//uri'):
                old_text = uri.text
                replace_str = re.sub("model://%s/" % old_name,
                                     "model://%s/" % model_name,
                                     uri.text,
                                     flags=re.IGNORECASE)

                if old_text != replace_str:
                    uri.text = replace_str
                    logger.warning("Internal name reference for %s "
                                   "changed from %s to %s"
                                   % (model_name,
                                       old_text,
                                       replace_str))
            logger.warning("Synced SDF names for %s! "
                           "Changed from %s to %s"
                           % (model_name, old_name, model_name))

        # NOTE(CH3): Sanitise malformed SDFs
        # Replaces 'close-enough' names with the appropriate folder
        # name.
        #
        # E.g.: If folder name is Lamp Post, but internal reference
        #       uses lamp_post, replaces those with Lamp Post.
        #
        # This catches cases where the internal reference is not
        # replaced because it is simply not the same as the sdf model
        # name.
        #
        # (Usually in the case of http://models.gazebosim.org/ models)

        # NOTE(CH3): Introduces the edge case where Lamp Post
        # is interdependent on another model called lamp_post. But
        # that case should be so rare, and so bad in a coding standard
        # standpoint where it should more or less never occur.
        for uri in tree.findall('.//uri'):
            old_text = uri.text
            regex_search_exp = re.sub("[ _]",
                                      "[ _]",
                                      old_name)

            replace_str = re.sub(
                "model://%s/" % regex_search_exp,
                "model://%s/" % model_name,
                uri.text,
                flags=re.IGNORECASE
            )

            if old_text != replace_str:
                uri.text = replace_str
                logger.warning("Sanitising name reference for %s. "
                               "Changed from %s to %s"
                               % (model_name,
                                   old_text,
                                   replace_str))

        if old_name != model_name:
            logger.warning("Synced SDF names for %s! "
                           "Changed from %s to %s"
                           % (model_name, old_name, model_name))
        tree.write(os.path.join(extract_path, "model.sdf"))

    except Exception as e:
        logger.error("Syncing of names for %s failed! %s"
                     % (model_name, e))
