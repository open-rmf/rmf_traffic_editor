from pprint import pprint

import pit_crew
import os

###############################################################################
# Welcome to the pit_crew usage guide! Let's set some stuff up...
###############################################################################

print("Welcome to\n", pit_crew.swag(print_swag=False))

# Configure Logger
pit_crew.init_logging()

###############################################################################
# Usage guide begin!
###############################################################################

# Models are identified by namedtuples of (model_name, author_name)
pit_crew.ModelNames("Desk", "OpenRobotics")


# First build or update the model_cache
# Here we also force a rebuild!
pit_crew.build_and_update_cache(cache_file_path=None, write_to_cache=True,
                                rebuild=True)


# Fetch model cache (in case we want to use it)
# But most of the functions we'll call will already load the cache during
# their business logic, so there is no need to pass in the model_cache
model_cache = pit_crew.load_cache()
# Disable for case-sensitive stuff
# Note that the default behaviour is to lowercase all names to allow for easy
# membership searches! (Fuel's REST API is also case insensitive.)
model_cache_caps = pit_crew.load_cache(lower=False)


# And then look for models in our local model directory!
models = pit_crew.get_local_model_name_tuples(lower=False)

# Or your Gazebosim model directory!
ign_models = pit_crew.get_local_model_name_tuples(lower=False, ign=True)

# Or you can use the model's folder name as its name!
path_as_name_models = pit_crew.get_local_model_name_tuples(
    lower=False,
    use_dir_as_name=True
)

print("Here's up to 10 models we've retrieved!")
pprint(list(x for x in models)[:10])


# You can also pass in a single model's path to find its ModelNames
print(
    pit_crew.get_model_name_tuple(
        os.path.expanduser("~/.gazebo/models/gazebo"),
        lower=False
    )
)


# You can get dicts that map model_name to author_name, or vice versa!
model_keyed_dict = pit_crew.get_model_to_author_dict(models)
author_keyed_dict = pit_crew.get_author_to_model_dict(models)


# You can all Fuel authors that have produced a particular model name
# Let's just specify no cache updates for this too
print(pit_crew.get_fuel_authors("gazebo", update_cache=False))
print(pit_crew.get_fuel_authors("dragon", update_cache=False))  # None found!


# List fuel models (works like `ign fuel list --type model`)
# Limit models to display per author to 1 here
pit_crew.list_fuel_models(update_cache=False, model_limit=1)


# You can also pass in an iterable of models or ModelNames
# and get them classified as AVAILABLE in local, DOWNLOADABLE on Fuel,
# or MISSING from both!
pprint(
    pit_crew.get_missing_models(
        [
            "dragon",
            "missingboi96",
            "Desk",
            "Cave Corner 02 Lights Type A",
            "ambulance",
            pit_crew.ModelNames("gazebo", "wow"),
            pit_crew.ModelNames("beer", "openrobotics"),
        ]
    )
)


# Download a model! Returns (False, None) if failed
# (True, metadata_json) otherwise.
fail_model = pit_crew.ModelNames("gazebo", "wow")
download_model = pit_crew.ModelNames("gazebo", "openrobotics")

pprint(pit_crew.download_model(fail_model.model_name,
                               fail_model.author_name,
                               dry_run=True))  # Fail!

pprint(pit_crew.download_model(download_model.model_name,
                               download_model.author_name,
                               dry_run=True))  # Pass!


# And finally, construct a license file!
_, metadata = pit_crew.download_model(download_model.model_name,
                                      download_model.author_name,
                                      dry_run=True)

pprint(pit_crew._construct_license(metadata))
