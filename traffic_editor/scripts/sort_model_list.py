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
        'model_list',
        help='model_list.yaml file that needs to be sorted.'
    )
    args = parser.parse_args(sys.argv[1:])

    if not os.path.exists(args.model_list):
        print('Model list file not found: {}'.format(args.source))
        exit()

    model_list_yaml = None
    with open(args.model_list) as f:
        model_list_yaml = yaml.load(f)

    model_list_yaml['models'] = sorted(
            model_list_yaml['models'], key=lambda s: s.lower())

    with open(args.model_list, 'w') as f:
        yaml.dump(model_list_yaml, f, default_flow_style=False)
