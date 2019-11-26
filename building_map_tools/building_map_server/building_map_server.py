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


class BuildingMapServer(Node):
    def __init__(self, map_path):
        super().__init__('building_map_server')

        self.building_map = self.load_map_from_path(map_path)

        self.map_srv = self.create_service(
            GetBuildingMap, 'get_building_map', self.get_building_map)

        qos_profile = qos_profile_default
        qos_profile.durability = \
            QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL

        self.map_pub = self.create_publisher(
            BuildingMap, 'map', qos_profile=qos_profile)

        self.get_logger().info('publishing map...')
        self.map_pub.publish(self.building_map)

    def parse_nav_graph(self, y):
        if not 'roads':
            raise ValueError('roads dictionary not found')

        g = NavGraph()

        for v in y['roads']['vertices']:
            n = NavGraphNode()
            n.x = float(v[0])
            n.y = float(v[1])
            n.z = float(v[2])
            g.vertices.append(n)

        for e in y['roads']['edges']:
            ge = GraphEdge()
            ge.v1 = e[0]
            ge.v2 = e[1]
            ge.flags = e[2]
            g.edges.append(ge)

        return g

    # todo: factor this into some helper library somewhere (metapath_planner?)
    def yaw_to_quaternion(self, yaw):
        # trig copied from wikipedia article...
        roll = 0
        pitch = 0

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quat = Quaternion()
        quat.w = cy * cp * cr + sy * sp * sr
        quat.x = cy * cp * sr - sy * sp * cr
        quat.y = sy * cp * sr + cy * sp * cr
        quat.z = sy * cp * cr - cy * sp * sr
        return quat

    def parse_places(self, y):
        if 'places' not in y:
            raise ValueError('places dictionary not found')

        places = []
        for place_name, place_data in y['places'].items():
            place = Place()
            place.name = place_name
            pose = Pose()
            pose.position = Point(x=float(place_data[0]),
                                  y=float(place_data[1]),
                                  z=float(place_data[2]))
            pose.orientation = self.yaw_to_quaternion(float(place_data[3]))
            place.location = pose

            if len(place_data) > 4:
                place.translation_tolerance = float(place_data[4])
                place.orientation_tolerance = float(place_data[5])
            else:
                place.translation_tolerance = 0.0
                place.orientation_tolerance = inf

            places.append(place)

        return places

    def parse_lifts(self, y):
        if 'lifts' not in y:
            raise ValueError('lifts dictionary not found')

        lifts = []
        for lift_name, lift_data in y['lifts'].items():
            lift = Lift()
            lift.name = lift_name
            lift.available_floors = lift_data['floors']
            lifts.append(lift)

        return lifts

    def parse_doors(self, y):
        if 'doors' not in y:
            raise ValueError('doors dictionary not found')

        doors = []
        for door_name, door_data in y['doors'].items():
            d = Door()
            d.name = door_name
            door_type = door_data['type']
            if door_type == 'SingleSlidingDoor':
                d.door_type = d.DOOR_TYPE_SINGLE_SLIDING
            elif door_type == 'DoubleSlidingDoor':
                d.door_type = d.DOOR_TYPE_DOUBLE_SLIDING
            elif door_type == 'SingleSwingDoor':
                d.door_type = d.DOOR_TYPE_SINGLE_SWING
            elif door_type == 'DoubleSwingDoor':
                d.door_type = d.DOOR_TYPE_DOUBLE_SWING
            elif door_type == 'SingleTelescopicDoor':
                d.door_type = d.DOOR_TYPE_SINGLE_TELESCOPE
            elif door_type == 'DoubleTelescopicDoor':
                d.door_type = d.DOOR_TYPE_DOUBLE_TELESCOPE
            else:
                raise NotImplementedError(
                    'metamap_server does not handle door of type {}'.format(
                        door_type))

            d.closed_edge_location = Pose()
            d.closed_edge_location.position.x = \
                float(door_data['center'][0])
            d.closed_edge_location.position.y = \
                float(door_data['center'][1])
            d.closed_edge_location.position.z = \
                float(door_data['center'][2])
            yaw = math.atan2(
                door_data['motion_axis'][1],
                door_data['motion_axis'][0])
            d.closed_edge_location.orientation = \
                self.yaw_to_quaternion(yaw)
            d.motion_range = door_data['width'] / 2.0

            doors.append(d)
            
        return doors

    def parse_single_floor(self, floor_name, y, bpath):
        f = Floor()
        f.name = floor_name
        f.elevation = float(y['elevation'])

        image = AffineImage()
        image_filename = y['image_filename']
        image.encoding = image_filename.split('.')[-1]
        image.scale = float(y['image_scale'])
        image.pose.x = float(y['image_pose'][0])
        image.pose.y = float(y['image_pose'][1])
        image.pose.theta = float(y['image_pose'][2])

        image_path = os.path.join(bpath, image_filename)
        # print('opening: {}'.format(image_path))
        with open(image_path, 'rb') as image_file:
            image.data = image_file.read()
        print('read {} byte image: {}'.format(len(image.data), image_filename))
        f.image = image

        return f

    def parse_floors(self, y, bm, bpath):
        if 'floors' not in y:
            raise ValueError('floors dictionary not found')

        floors = []
        for floor_name, floor_data in y['floors'].items():
            floors.append(
                self.parse_single_floor(floor_name, floor_data, bpath))

        return floors

    def parse_building(self, y, bpath):
        bm = BuildingMap()
        bm.name = y['building_name']
        bm.floors = self.parse_floors(y, bm, bpath)
        bm.doors = self.parse_doors(y)
        bm.lifts = self.parse_lifts(y)
        bm.places = self.parse_places(y)
        bm.nav_graph = self.parse_nav_graph(y)
        return bm

    def get_building_map(self, request, response):
        self.get_logger().info('get_building_map()')
        # todo: only include obstacle images if they are requested
        response.building_map = self.building_map
        return response

    def load_map_from_path(self, path):
        self.get_logger().info('loading map path: {}'.format(path))

        if not os.path.isdir(path):
            raise FileNotFoundError(
                errno.ENOENT, os.strerror(errno.ENOENT), path)

        ypath = os.path.join(path, 'unified.yaml')
        if not os.path.isfile(ypath):
            self.get_logger().error('no unified.yaml in {}'.format(path))
            raise FileNotFoundError(
                errno.ENOENT, os.strerror(errno.ENOENT), ypath)

        with open(ypath, 'r') as ystream:
            y = yaml.load(ystream)
            building_map = self.parse_building(y, path)
            if building_map is None:
                raise ValueError("unable to parse map at {}".format(path))
            return building_map

    def main(self):
        self.get_logger().info(
            'ready to serve map: "{}"  Ctrl+C to exit...'.format(
                self.building_map.name))
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass


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
    n.main()


if __name__ == '__main__':
    sys.exit(main())
