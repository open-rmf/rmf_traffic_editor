#!/usr/bin/env python3

# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pit_crew

import sys
import yaml
import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'output_yaml',
        help='Output yaml file that the directory of models will be added to.'
    )
    parser.add_argument(
        '-d', '--model-dir', help='Directory of models to be added.'
    )
    parser.add_argument(
        '-b', '--blacklist-models', default='',
        help='Optional blacklisted models that will be excluded from your '
        'model directory'
    )
    parser.add_argument(
        '--meters-per-pixel', type=float, default=0.004,
        help='Scale of models in meters per pixel for the images.'
    )
    args = parser.parse_args(sys.argv[1:])

    y = {}
    y['models'] = []

    blacklist_yaml = None
    if args.blacklist_models != '':
        print('Blacklisted models:')
        with open(args.blacklist_models) as f:
            blacklist_yaml = yaml.load(f)
            print(blacklist_yaml)

    models = pit_crew.get_local_model_name_tuples(path=args.model_dir,
                                                  lower=False,
                                                  use_dir_as_name=True)

    for model in models:
        if blacklist_yaml is not None and \
                model.model_name in blacklist_yaml['blacklist']:
            print('ignoring {} because it is blacklisted'.format(
                    model.model_name))
            continue

        if model.author_name:
            y['models'].append("%s/%s" % (model.author_name,
                                          model.model_name))
        else:
            y['models'].append(model.model_name)

    assert len(y['models']) > 0, "No models found!"
    print('found {} models'.format(len(y['models'])))

    y['models'] = sorted(y['models'], key=lambda s: s.lower())
    y['meters_per_pixel'] = 0.004

    with open(args.output_yaml, 'w') as f:
        yaml.dump(y, f, default_flow_style=False)
