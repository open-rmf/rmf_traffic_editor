#!/usr/bin/env python

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

import os
import sys
import yaml
import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'destination',
        help='Destination list of models which the source will be '
        'merged into.'
    )
    parser.add_argument('-s', '--source', help='Source list of models.')
    args = parser.parse_args(sys.argv[1:])

    if not os.path.exists(args.source):
        print('Source model list not found: {}'.format(args.source))
        exit()
    if not os.path.exists(args.destination):
        print('Destination model list not found: {}'.format(args.destination))
        exit()

    source_yaml = None
    with open(args.source) as f:
        source_yaml = yaml.load(f)

    destination_yaml = None
    with open(args.destination) as f:
        destination_yaml = yaml.load(f)

    if abs(source_yaml['meters_per_pixel'] -
            destination_yaml['meters_per_pixel']) > 1e-6:
        print('Both model lists have different meters_per_pixel.')
        exit()

    for model_name in source_yaml['models']:
        if model_name in destination_yaml['models']:
            print('Skipping model {} because it already exists'.format(
                    model_name))
            continue
        destination_yaml['models'].append(model_name)

    destination_yaml['models'] = sorted(
            destination_yaml['models'], key=lambda s: s.lower())

    with open(args.destination, 'w') as f:
        yaml.dump(destination_yaml, f, default_flow_style=False)
