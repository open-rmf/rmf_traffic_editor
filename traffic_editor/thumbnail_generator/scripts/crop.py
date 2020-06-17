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
import cv2
import sys
import yaml
import argparse


def crop(green_img, white_img):
    ''' Crop the image, keeping the center point still inthe center. '''

    # first find the crop mask on the green image
    print('green image size: {}x{}'.format(
            green_img.shape[1], green_img.shape[0]))
    mask = cv2.bitwise_not(cv2.inRange(green_img, (0, 200, 0), (30, 255, 30)))
    _, contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # now, apply that mask to the white image to avoid green-fringe effects
    img = cv2.cvtColor(white_img, cv2.COLOR_BGR2BGRA)
    img[mask == 0] = (0, 0, 0, 0)

    xbounds = [img.shape[1], 0]
    ybounds = [img.shape[0], 0]
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if x < xbounds[0]:
            xbounds[0] = x
        if y < ybounds[0]:
            ybounds[0] = y
        if x + w > xbounds[1]:
            xbounds[1] = x + w
        if y + h > ybounds[1]:
            ybounds[1] = y + h

    # we have to stay centered on the midpoint of the image
    left_extent = img.shape[1]/2 - xbounds[0]
    right_extent = xbounds[1] - img.shape[1]/2
    top_extent = img.shape[0]/2 - ybounds[0]
    bottom_extent = ybounds[1] - img.shape[0]/2
    if left_extent > right_extent:
        horiz_extent_div2 = left_extent
    else:
        horiz_extent_div2 = right_extent

    if top_extent > bottom_extent:
        vert_extent_div2 = top_extent
    else:
        vert_extent_div2 = bottom_extent

    ulx = img.shape[1]/2 - horiz_extent_div2
    uly = img.shape[0]/2 - vert_extent_div2
    lrx = img.shape[1]/2 + horiz_extent_div2
    lry = img.shape[1]/2 + vert_extent_div2

    print('cropped to {}x{}'.format(2*horiz_extent_div2, 2*vert_extent_div2))

    cropped = img[uly:lry, ulx:lrx]
    return cropped


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'model_list', default='../test/model_list.yaml',
        help='Path of model_list.yaml'
    )
    parser.add_argument(
        '-o', '--output_dir',
        default='../images/cropped/',
        help='Directory where the cropped output images will be saved'
    )
    parser.add_argument(
        '-g', '--green-img-dir', default='../images/green/',
        help='Directory filled with green screened model images'
    )
    parser.add_argument(
        '-w', '--white-img-dir', default='../images/white/',
        help='Directory filled with white screened model images'
    )
    args = parser.parse_args(sys.argv[1:])

    with open(args.model_list) as f:
        y = yaml.load(f)

    dirs = [os.path.expanduser(args.output_dir),
            os.path.expanduser(args.green_img_dir),
            os.path.expanduser(args.white_img_dir)]

    # Make dirs if they dont exist
    for dir in dirs:
        try:
            os.makedirs(dir)
            print("Made diretory: {}".format(dir))
        except Exception:
            pass

    for model_name in y['models']:
        if "/" in model_name:
            # There is an author name; a new folder should be created
            author_name = model_name.split("/")[0]
        else:
            author_name = ""

        try:
            if author_name:
                os.makedirs(
                    os.path.join(
                        os.path.expanduser(args.output_dir),
                        author_name
                    )
                )
        except Exception as e:
            pass

        green_img = cv2.imread(os.path.join(
            os.path.expanduser(args.green_img_dir),
            '{}.png'.format(model_name)
            )
        )
        white_img = cv2.imread(os.path.join(
            os.path.expanduser(args.white_img_dir),
            '{}.png'.format(model_name)
            )
        )

        output_filepath = os.path.join(
            os.path.expanduser(args.output_dir),
            '{}.png'.format(model_name)
        )

        print('generating {}'.format(output_filepath))

        white_img_cropped = crop(green_img, white_img)
        # check if we save the image properly
        if not cv2.imwrite(output_filepath, white_img_cropped):
            print('Failed to crop {}'.format(output_filepath))
