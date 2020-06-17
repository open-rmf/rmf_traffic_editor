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

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


xml = '''\
<sdf version="1.4">
  <world name='default'>
    <include>
      <uri>model://{}</uri>
      <static>true</static>
    </include>
  </world>
</sdf>'''


class TopViewGenerator:
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
                'model_list', default='../test/model_list.yaml',
                help='Path of model_list.yaml')
        parser.add_argument(
                'output_dir',
                help='Directory where the output images will be saved')
        args = parser.parse_args(argv[1:])

        self.output_dir = args.output_dir

        self.yaml = None
        with open(args.model_list) as f:
            self.yaml = yaml.load(f)
        print(self.yaml)

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        rospy.wait_for_service('gazebo/delete_model')
        self.spawn = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        self.delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        print('service clients created')

        rospy.Subscriber('/camera/image', Image, self.image_cb)
        self.last_image_msg = None

        self.cv_bridge = CvBridge()

    def image_cb(self, msg):
        self.last_image_msg = msg

    def run(self):
        for model_name in self.yaml['models']:
            # WARNING: Does not deal with the edge case of authorless
            # model with a "/" in the model name. Very rare though.
            if "/" in model_name:
                # Remove author name because model path won't be structured
                # that way
                model_xml = xml.format("/".join(model_name.split("/")[1:]))
                author_name = model_name.split("/")[0]
            else:
                model_xml = xml.format(model_name)
                author_name = ""

            try:
                if author_name:
                    os.makedirs(os.path.join(self.output_dir, author_name))
            except Exception as e:
                pass

            file_path = os.path.join(self.output_dir,
                                     '{}.png'.format(model_name))
            if os.path.exists(file_path):
                continue

            pose = Pose()
            pose.orientation.w = 1.0
            print('spawning {}'.format(model_name))
            self.spawn(
                    model_name='model', model_xml=model_xml,
                    robot_namespace='', initial_pose=pose, reference_frame='')
            rospy.sleep(3.0)

            # Grab the last-received image and save it
            cv_img = self.cv_bridge.imgmsg_to_cv2(self.last_image_msg, 'bgr8')
            cv2.imwrite(file_path, cv_img)
            print('wrote {}'.format(file_path))

            # for unknown reasons occasionally the 'delete' command doesn't
            # work so let's send it a few times
            for x in range(0, 4):
                self.delete(model_name='model')
                rospy.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node('top_view_generator')
    top_view_generator = TopViewGenerator(sys.argv)
    top_view_generator.run()
