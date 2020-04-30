#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from building_map_msgs.msg import BuildingMap
from building_map_msgs.msg import GraphEdge


class BuildingMapClient(Node):
    def __init__(self):
        super().__init__('building_map_client')

        qos = QoSProfile(
            history=History.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
            reliability=Reliability.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=Durability.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        self.create_subscription(
            BuildingMap, 'map', self.map_cb, qos_profile=qos)

    def print_door(self, door):
        print(f'        {door.door_name}, ' +
              f'v1:[{round(door.v1_x, 1)},{round(door.v1_y,1)}], ' +
              f'v2:[{round(door.v2_x, 1)},{round(door.v2_y,1)}], ' +
              f'type:{door.door_type}, range:{door.motion_range}, ' +
              f'dir:{door.motion_direction}')

    def map_cb(self, msg):
        print('received map!')
        print(f'building name: {msg.name}')
        print(f'{len(msg.lifts)} lifts')
        for lift in msg.lifts:
            print(f'  lift: {lift.name}')
            print(f'    {lift.levels} levels')
            print(f'    ({round(lift.ref_x, 1)},{round(lift.ref_y, 1)},' +
                  f'{round(lift.ref_yaw, 1)})')
            print(f'    width:{round(lift.width, 1)} ' +
                  f'depth:{round(lift.depth, 1)}')
            print(f'    {len(lift.doors)} doors')
            for door in lift.doors:
                self.print_door(door)
        print(f'{len(msg.levels)} levels')
        for level in msg.levels:
            print(f'  level: {level.name}')
            print(f'    {level.elevation} elevation')
            print(f'    {len(level.images)} images')
            print(f'    {len(level.places)} places')
            print(f'    {len(level.doors)} doors')
            for door in level.doors:
                self.print_door(door)
            print(f'    {len(level.nav_graphs)} navigation graphs')
            for nav_graph in level.nav_graphs:
                print(f'    graph {nav_graph.name}:')
                print(f'      {len(nav_graph.vertices)} vertices')
                for vertex in nav_graph.vertices:
                    print(f'        ({round(vertex.x, 1)}, ' +
                          f'{round(vertex.y, 1)}, "{vertex.name}")')

                print(f'      {len(nav_graph.edges)} edges')
                for edge in nav_graph.edges:
                    if edge.edge_type == GraphEdge.EDGE_TYPE_BIDIRECTIONAL:
                        arrow = '<=>'
                    else:
                        arrow = '=>'
                    print(f'        {edge.v1_idx} {arrow} {edge.v2_idx}')
        print('\npress Ctrl+C to exit...')


if __name__ == '__main__':
    rclpy.init()
    n = BuildingMapClient()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
