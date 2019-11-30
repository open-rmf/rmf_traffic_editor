import errno
import math
import os
import sys
import yaml

from numpy import inf

import rclpy
from rclpy.qos import qos_profile_default
from rclpy.qos import QoSDurabilityPolicy
from rclpy.node import Node

from building_map_msgs.srv import GetBuildingMap
from building_map_msgs.msg import BuildingMap
from building_map_msgs.msg import Level
from building_map_msgs.msg import Graph
from building_map_msgs.msg import GraphNode
from building_map_msgs.msg import GraphEdge
from building_map_msgs.msg import Place
from building_map_msgs.msg import AffineImage
from building_map_msgs.msg import Door
from building_map_msgs.msg import Lift

from building_map.building import Building

class BuildingMapServer(Node):
    def __init__(self, map_path):
        super().__init__('building_map_server')

        self.get_logger().info('loading map path: {}'.format(map_path))

        if not os.path.isfile(map_path):
            raise FileNotFoundError(
                errno.ENOENT, os.strerror(errno.ENOENT), map_path)
        self.map_dir = os.path.dirname(map_path)  # for calculating image paths

        with open(map_path, 'r') as f:
            self.building = Building(yaml.safe_load(f))

        self.map_msg = self.building_map_msg(self.building)

        self.get_building_map_srv = self.create_service(
            GetBuildingMap, 'get_building_map', self.get_building_map)

        qos_profile = qos_profile_default
        qos_profile.durability = \
            QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL

        self.building_map_pub = self.create_publisher(
            BuildingMap, 'map', qos_profile=qos_profile)

        self.get_logger().info('publishing map...')
        self.building_map_pub.publish(self.map_msg)

        self.get_logger().info(
            'ready to serve map: "{}"  Ctrl+C to exit...'.format(
                self.map_msg.name))

    def building_map_msg(self, building):
        msg = BuildingMap()
        msg.name = building.name
        for _, level_data in building.levels.items():
            msg.levels.append(self.level_msg(level_data))
        # todo: lifts
        return msg

    def level_msg(self, level):
        msg = Level()
        msg.name = level.name
        msg.elevation = 0.0  # todo: actually capture this in traffic-editor
        if level.drawing_name:
            image = AffineImage()
            image_filename = level.drawing_name
            image.encoding = image_filename.split('.')[-1]
            image.scale = level.scale
            image.x_offset = 0.0
            image.y_offset = 0.0
            image.yaw = 0.0

            image_path = os.path.join(self.map_dir, image_filename)

            print('opening: {}'.format(image_path))
            with open(image_path, 'rb') as image_file:
                image.data = image_file.read()
            print('read {} byte image: {}'.format(
                len(image.data), image_filename))
            msg.images.append(image)

        return msg

    def get_building_map(self, request, response):
        self.get_logger().info('get_building_map()')
        # todo: only include images/graphs if they are requested?
        response.building_map = self.map_msg
        return response


def main():
    if len(sys.argv) > 1:
        map_path = sys.argv[1]
    elif 'RMF_MAP_PATH' in os.environ:
        map_path = os.environ['RMF_MAP_PATH']
    else:
        print('map path must be provided in command line or RMF_MAP_PATH env')
        sys.exit(1)
        raise ValueError('Map path not provided')

    rclpy.init()
    n = BuildingMapServer(map_path)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    sys.exit(main())
