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

import os
import sys
import yaml
import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
            '-d', '--model-dir', help='Directory of models to be added.')
    parser.add_argument(
            '-b', '--blacklist-models', default='',
            help='Optional blacklisted models that will be excluded from your '
            'model directory')
    parser.add_argument(
            '-o', '--output-yaml',
            help='Output yaml file that the directory of models will be added '
            'to.')
    parser.add_argument(
            '--meters-per-pixel', type=float, default=0.004,
            help='Scale of models in meters per pixel for the images.')
    args = parser.parse_args(sys.argv[1:])

    y = {}
    y['models'] = []

    blacklist_yaml = None
    if args.blacklist_models != '':
        print('Blacklisted models:')
        with open(args.blacklist_models) as f:
            blacklist_yaml = yaml.load(f)
            print(blacklist_yaml)

    with os.scandir(args.model_dir) as it:
        for entry in it:
            if not entry.is_dir():
                continue
            if blacklist_yaml is not None and \
                    entry.name in blacklist_yaml['blacklist']:
                print('ignoring {} because it is blacklisted'.format(
                        entry.name))
                continue
            if entry.name in y['models']:
                print('skipping {} because it already exists'.format(
                        entry.name))
                continue
            y['models'].append(entry.name)

    print('found {} models'.format(len(y['models'])))
    y['models'] = sorted(y['models'], key=lambda s: s.lower())

    y['meters_per_pixel'] = 0.004

    with open(args.output_yaml, 'w') as f:
        yaml.dump(y, f, default_flow_style=False)
