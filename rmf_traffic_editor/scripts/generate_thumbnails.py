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


class ThumbnailsGenerator:
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
                'models_dir', default='~/.gazebo/models',
                help='Directory of models to be added')
        parser.add_argument(
                'model_list', default='../test/model_list.yaml',
                help='Path of model_list.yaml')
        parser.add_argument(
                'output_dir', default='.',
                help='Directory where the output images will be saved')
        # Optional arguments
        parser.add_argument(
                '--size', dest='img_size', default=4000,
                help='Output thumbnail Image pixel size')
        parser.add_argument(
                '--height', dest='cam_height', default=200,
                help='Scene camara height')
        parser.add_argument(
                '--hfov', dest='cam_hfov', default=0.08,
                help='Scene camera horizontal FOV')
        args = parser.parse_args(argv[1:])

        self.output_dir = args.output_dir
        self.models_dir = args.models_dir
        self.img_size = args.img_size
        self.cam_height = args.cam_height
        self.cam_hfov = args.cam_hfov

        self.yaml = None
        with open(args.model_list) as f:
            self.yaml = yaml.load(f)
        print(self.yaml)

    def run(self):
        for full_model_name in self.yaml['models']:
            model_name = full_model_name.split("/")[-1]
            sdf_path = "{}/{}/model.sdf".format(self.models_dir, model_name)

            if os.path.exists(sdf_path) is False:
                print(" Warn!! {} doesnt exist, skip!".format(model_name))
                continue

            command = "gzserver -s libthumbnail_generator.so \
                    empty.world --input '{}' --output '{}' \
                    --img-size {} --cam-height {} --cam-hfov {} ".format(
                    sdf_path, self.output_dir, self.img_size,
                    self.cam_height, self.cam_hfov)
            print("------------------------------------------------------")
            os.system(command)


if __name__ == '__main__':
    thumbnails_generator = ThumbnailsGenerator(sys.argv)
    thumbnails_generator.run()
    print("Done All")
