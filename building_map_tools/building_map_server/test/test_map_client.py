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

    def map_cb(self, msg):
        print('received map!')
        print(f'building name: {msg.name}')
        for level in msg.levels:
            print(f'  level: {level.name}')
            print(f'    {len(level.nav_graphs)} navigation graphs')
            for nav_graph in level.nav_graphs:
                print(f'    graph {nav_graph.name}:')
                print(f'      {len(nav_graph.vertices)} vertices')
                for vertex in nav_graph.vertices:
                    print(f'        ({vertex.x}, {vertex.y}, "{vertex.name}")')

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
